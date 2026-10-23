/**
  ******************************************************************************
  * @file           : tue_sts3215.c
  * @brief          : High-performance Feetech STS3215 serial bus servo driver
  *                   for STM32F407 (Half-duplex UART + DMA + Interrupt)
  ******************************************************************************
  * @attention
  *
  * Hardware mapping:
  * - SERVO_TX (PA9) configured in Half-Duplex Single-Wire mode (USART1).
  * - DMA2 Stream 7 (TX, Normal Mode).
  * - DMA2 Stream 2 (RX, Normal Mode).
  *
  * Communication features:
  * - Zero-latency directional bus switching inside UART TX Complete ISR.
  * - Reception via HAL_UARTEx_ReceiveToIdle_DMA (IDLE line trigger) for variable length
  *   packets and automatic frame termination.
  * - CMSIS-RTOS2 mutex and binary semaphores for thread safety and non-busy waiting.
  * - Fallback to millisecond polling before RTOS scheduler launch.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_sts3215.h"
#include <string.h>

#if defined(FreeRTOS) || defined(INC_FREERTOS_H)
#include "FreeRTOS.h"
#include "task.h"
#else
/* When using CMSIS-RTOS2 on FreeRTOS */
extern int xTaskGetSchedulerState(void);
#define taskSCHEDULER_RUNNING 2
#endif

/* Private Variables ---------------------------------------------------------*/
static tue_sts3215_t *s_default_dev = NULL;

/* Private Helper Function Prototypes ----------------------------------------*/
static uint8_t  sts3215_calc_checksum(const uint8_t *buf, uint16_t len);
static uint16_t sts3215_offset_to_raw(int16_t offset);
static int16_t  sts3215_raw_to_offset(uint16_t raw);
static uint16_t sts3215_pwm_to_raw(int16_t pwm);
static int16_t  sts3215_raw_to_pwm(uint16_t raw);
static bool     sts3215_is_rtos_running(const tue_sts3215_t *dev);

static tue_sts3215_status_t sts3215_transceive(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t inst,
    const uint8_t *params,
    uint16_t param_len,
    uint8_t *rx_data,
    uint8_t rx_expected_len
);

/* Driver Lifecycle & Initialization -----------------------------------------*/

tue_sts3215_status_t tue_sts3215_init(tue_sts3215_t *dev, UART_HandleTypeDef *huart)
{
    if (dev == NULL || huart == NULL) {
        return TUE_STS3215_ERR_PARAM;
    }

    memset(dev, 0, sizeof(tue_sts3215_t));
    dev->huart = huart;
    dev->timeout_ms = STS3215_DEFAULT_TIMEOUT_MS;
    dev->use_rtos = true;

    /* Create CMSIS-RTOS2 transaction mutex */
    const osMutexAttr_t mutex_attr = {
        .name = "sts3215_mtx",
        .attr_bits = osMutexPrioInherit
    };
    dev->mutex = osMutexNew(&mutex_attr);

    /* Create binary semaphores for DMA TX and RX synchronization */
    const osSemaphoreAttr_t tx_sem_attr = {
        .name = "sts3215_tx_sem"
    };
    dev->tx_sem = osSemaphoreNew(1U, 0U, &tx_sem_attr);

    const osSemaphoreAttr_t rx_sem_attr = {
        .name = "sts3215_rx_sem"
    };
    dev->rx_sem = osSemaphoreNew(1U, 0U, &rx_sem_attr);

    if (s_default_dev == NULL) {
        s_default_dev = dev;
    }

    return TUE_STS3215_OK;
}

tue_sts3215_t* tue_sts3215_get_default(void)
{
    return s_default_dev;
}

void tue_sts3215_set_timeout(tue_sts3215_t *dev, uint32_t timeout_ms)
{
    if (dev != NULL && timeout_ms > 0) {
        dev->timeout_ms = timeout_ms;
    }
}

void tue_sts3215_set_use_rtos(tue_sts3215_t *dev, bool use_rtos)
{
    if (dev != NULL) {
        dev->use_rtos = use_rtos;
    }
}

/* ISR Notification Callbacks ------------------------------------------------*/

void tue_sts3215_on_tx_cplt(UART_HandleTypeDef *huart)
{
    tue_sts3215_t *dev = (s_default_dev != NULL && s_default_dev->huart == huart) ? s_default_dev : NULL;
    if (dev == NULL) {
        return;
    }

    dev->tx_done = 1U;

    /* If a response packet is awaited, switch half-duplex bus direction to RX immediately */
    if (dev->awaiting_rx) {
        /* Clear any stale or overrun flags */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        (void)huart->Instance->SR;
        (void)huart->Instance->DR;

        /* Enable Receiver, disable Transmitter */
        HAL_HalfDuplex_EnableReceiver(huart);

        /* Arm reception to IDLE event using DMA */
        HAL_UARTEx_ReceiveToIdle_DMA(huart, dev->rx_buf, (uint16_t)sizeof(dev->rx_buf));
    }

    /* Unblock waiting task */
    if (dev->tx_sem != NULL && dev->use_rtos) {
        osSemaphoreRelease(dev->tx_sem);
    }
}

void tue_sts3215_on_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    tue_sts3215_t *dev = (s_default_dev != NULL && s_default_dev->huart == huart) ? s_default_dev : NULL;
    if (dev == NULL) {
        return;
    }

    dev->rx_len = size;
    dev->rx_done = 1U;
    dev->awaiting_rx = 0U;

    if (dev->rx_sem != NULL && dev->use_rtos) {
        osSemaphoreRelease(dev->rx_sem);
    }
}

void tue_sts3215_on_rx_cplt(UART_HandleTypeDef *huart)
{
    tue_sts3215_t *dev = (s_default_dev != NULL && s_default_dev->huart == huart) ? s_default_dev : NULL;
    if (dev == NULL) {
        return;
    }

    dev->rx_len = huart->RxXferSize;
    dev->rx_done = 1U;
    dev->awaiting_rx = 0U;

    if (dev->rx_sem != NULL && dev->use_rtos) {
        osSemaphoreRelease(dev->rx_sem);
    }
}

void tue_sts3215_on_error(UART_HandleTypeDef *huart)
{
    tue_sts3215_t *dev = (s_default_dev != NULL && s_default_dev->huart == huart) ? s_default_dev : NULL;
    if (dev == NULL) {
        return;
    }

    dev->last_hal_error = huart->ErrorCode;
    dev->rx_done = 1U;
    dev->awaiting_rx = 0U;

    if (dev->rx_sem != NULL && dev->use_rtos) {
        osSemaphoreRelease(dev->rx_sem);
    }
}

/* Weak default implementations for STM32 HAL Callbacks ---------------------*/

__weak void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    tue_sts3215_on_tx_cplt(huart);
}

__weak void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    tue_sts3215_on_rx_event(huart, Size);
}

__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    tue_sts3215_on_rx_cplt(huart);
}

__weak void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    tue_sts3215_on_error(huart);
}

/* Private Helpers -----------------------------------------------------------*/

static bool sts3215_is_rtos_running(const tue_sts3215_t *dev)
{
    return (dev->use_rtos && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING));
}

static uint8_t sts3215_calc_checksum(const uint8_t *buf, uint16_t len)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (uint8_t)((~sum) & 0xFFU);
}

static uint16_t sts3215_offset_to_raw(int16_t offset)
{
    if (offset < 0) {
        uint16_t mag = (uint16_t)(-offset);
        if (mag > 2047U) mag = 2047U;
        return (uint16_t)((1U << 11) | (mag & 0x07FFU));
    } else {
        uint16_t mag = (uint16_t)offset;
        if (mag > 2047U) mag = 2047U;
        return (uint16_t)(mag & 0x07FFU);
    }
}

static int16_t sts3215_raw_to_offset(uint16_t raw)
{
    int16_t val = (int16_t)(raw & 0x07FFU);
    if (raw & (1U << 11)) {
        return (int16_t)(-val);
    }
    return val;
}

static uint16_t sts3215_pwm_to_raw(int16_t pwm)
{
    if (pwm < 0) {
        int16_t mag = (int16_t)(-pwm);
        if (mag > 1000) mag = 1000;
        return (uint16_t)((1U << 10) | ((uint16_t)mag & 0x03FFU));
    } else {
        int16_t mag = pwm;
        if (mag > 1000) mag = 1000;
        return (uint16_t)((uint16_t)mag & 0x03FFU);
    }
}

static int16_t sts3215_raw_to_pwm(uint16_t raw)
{
    int16_t mag = (int16_t)(raw & 0x03FFU);
    if (raw & (1U << 10)) {
        return (int16_t)(-mag);
    }
    return mag;
}

/* Core Transceive Engine ----------------------------------------------------*/

static tue_sts3215_status_t sts3215_transceive(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t inst,
    const uint8_t *params,
    uint16_t param_len,
    uint8_t *rx_data,
    uint8_t rx_expected_len)
{
    if (dev == NULL || dev->huart == NULL) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint16_t total_tx_len = param_len + 6U; /* Header(2) + ID(1) + Len(1) + Inst(1) + Params + Checksum(1) */
    if (total_tx_len > STS3215_BUFFER_SIZE) {
        return TUE_STS3215_ERR_PARAM;
    }

    bool rtos = sts3215_is_rtos_running(dev);

    /* Acquire bus mutex */
    if (rtos && dev->mutex != NULL) {
        if (osMutexAcquire(dev->mutex, dev->timeout_ms) != osOK) {
            return TUE_STS3215_ERR_BUSY;
        }
    }

    /* Build instruction frame: 0xFF 0xFF ID LEN INST [PARAMS] CHECKSUM */
    dev->tx_buf[0] = STS3215_HEADER_BYTE;
    dev->tx_buf[1] = STS3215_HEADER_BYTE;
    dev->tx_buf[2] = id;
    dev->tx_buf[3] = (uint8_t)(param_len + 2U); /* LEN = Instruction(1) + Parameters(N) + Checksum(1) */
    dev->tx_buf[4] = inst;

    if (param_len > 0 && params != NULL) {
        memcpy(&dev->tx_buf[5], params, param_len);
    }

    /* Compute Checksum: ~(ID + LEN + INST + PARAMS) */
    dev->tx_buf[total_tx_len - 1U] = sts3215_calc_checksum(&dev->tx_buf[2], (uint16_t)(param_len + 3U));

    /* Prepare transfer state flags */
    dev->tx_done = 0U;
    dev->rx_done = 0U;
    dev->rx_len = 0U;
    dev->last_hal_error = HAL_UART_ERROR_NONE;
    dev->last_servo_status = 0U;

    /* A response packet is expected only if rx_expected_len > 0 and not broadcast */
    bool expect_resp = (rx_expected_len > 0 && id != STS3215_BROADCAST_ID);
    dev->awaiting_rx = expect_resp ? 1U : 0U;

    /* Drain semaphores if previously flagged */
    if (rtos) {
        if (dev->tx_sem != NULL) {
            while (osSemaphoreAcquire(dev->tx_sem, 0U) == osOK) {}
        }
        if (dev->rx_sem != NULL) {
            while (osSemaphoreAcquire(dev->rx_sem, 0U) == osOK) {}
        }
    }

    /* Enable Transmitter and initiate DMA TX */
    HAL_HalfDuplex_EnableTransmitter(dev->huart);
    HAL_StatusTypeDef hal_st = HAL_UART_Transmit_DMA(dev->huart, dev->tx_buf, total_tx_len);
    if (hal_st != HAL_OK) {
        dev->awaiting_rx = 0U;
        if (rtos && dev->mutex != NULL) {
            osMutexRelease(dev->mutex);
        }
        return TUE_STS3215_ERR_HAL;
    }

    /* Wait for Transmission Complete */
    uint32_t start_tick = HAL_GetTick();
    if (rtos && dev->tx_sem != NULL) {
        if (osSemaphoreAcquire(dev->tx_sem, 10U) != osOK) {
            HAL_UART_AbortTransmit(dev->huart);
            dev->awaiting_rx = 0U;
            if (dev->mutex != NULL) osMutexRelease(dev->mutex);
            return TUE_STS3215_ERR_TIMEOUT;
        }
    } else {
        while (!dev->tx_done && (HAL_GetTick() - start_tick) < 10U) {}
        if (!dev->tx_done) {
            HAL_UART_AbortTransmit(dev->huart);
            dev->awaiting_rx = 0U;
            return TUE_STS3215_ERR_TIMEOUT;
        }
    }

    /* If no response expected, return immediately */
    if (!expect_resp) {
        if (rtos && dev->mutex != NULL) {
            osMutexRelease(dev->mutex);
        }
        return TUE_STS3215_OK;
    }

    /* Wait for Reception Complete */
    start_tick = HAL_GetTick();
    if (rtos && dev->rx_sem != NULL) {
        if (osSemaphoreAcquire(dev->rx_sem, dev->timeout_ms) != osOK) {
            HAL_UART_AbortReceive(dev->huart);
            dev->awaiting_rx = 0U;
            if (dev->mutex != NULL) osMutexRelease(dev->mutex);
            return TUE_STS3215_ERR_TIMEOUT;
        }
    } else {
        while (!dev->rx_done && (HAL_GetTick() - start_tick) < dev->timeout_ms) {}
        if (!dev->rx_done) {
            HAL_UART_AbortReceive(dev->huart);
            dev->awaiting_rx = 0U;
            return TUE_STS3215_ERR_TIMEOUT;
        }
    }

    /* Check for physical UART errors */
    if (dev->last_hal_error != HAL_UART_ERROR_NONE) {
        if (rtos && dev->mutex != NULL) {
            osMutexRelease(dev->mutex);
        }
        return TUE_STS3215_ERR_HAL;
    }

    /* Find Start of Frame (0xFF 0xFF) */
    int h_idx = -1;
    for (int i = 0; i + 1 < (int)dev->rx_len; i++) {
        if (dev->rx_buf[i] == STS3215_HEADER_BYTE && dev->rx_buf[i + 1] == STS3215_HEADER_BYTE) {
            h_idx = i;
            break;
        }
    }

    if (h_idx < 0) {
        if (rtos && dev->mutex != NULL) osMutexRelease(dev->mutex);
        return TUE_STS3215_ERR_HEADER;
    }

    uint8_t *pkt = &dev->rx_buf[h_idx];
    uint16_t pkt_avail = (uint16_t)(dev->rx_len - (uint16_t)h_idx);

    /* Frame must have at least 6 bytes: 0xFF 0xFF ID LEN ERR CHECKSUM */
    if (pkt_avail < 6U) {
        if (rtos && dev->mutex != NULL) osMutexRelease(dev->mutex);
        return TUE_STS3215_ERR_LEN;
    }

    /* Verify ID */
    if (pkt[2] != id && id != STS3215_BROADCAST_ID) {
        if (rtos && dev->mutex != NULL) osMutexRelease(dev->mutex);
        return TUE_STS3215_ERR_ID;
    }

    uint8_t pkt_len = pkt[3];
    uint16_t total_expected_frame = (uint16_t)(pkt_len + 4U);
    if (pkt_avail < total_expected_frame) {
        if (rtos && dev->mutex != NULL) osMutexRelease(dev->mutex);
        return TUE_STS3215_ERR_LEN;
    }

    /* Verify Checksum */
    uint8_t calc_cs = sts3215_calc_checksum(&pkt[2], (uint16_t)(pkt_len + 1U));
    uint8_t rcv_cs  = pkt[total_expected_frame - 1U];
    if (calc_cs != rcv_cs) {
        if (rtos && dev->mutex != NULL) osMutexRelease(dev->mutex);
        return TUE_STS3215_ERR_CHECKSUM;
    }

    /* Capture hardware error status byte */
    dev->last_servo_status = pkt[4];

    /* Copy payload data if requested */
    uint8_t data_bytes = (pkt_len >= 2U) ? (uint8_t)(pkt_len - 2U) : 0U;
    if (rx_data != NULL && data_bytes > 0U) {
        uint8_t copy_bytes = (data_bytes < rx_expected_len) ? data_bytes : rx_expected_len;
        memcpy(rx_data, &pkt[5], copy_bytes);
    }

    if (rtos && dev->mutex != NULL) {
        osMutexRelease(dev->mutex);
    }

    /* If status byte reported error bits, notify caller */
    if (dev->last_servo_status != 0U) {
        return TUE_STS3215_ERR_STATUS;
    }

    return TUE_STS3215_OK;
}

/* Low-Level Protocol Functions ----------------------------------------------*/

tue_sts3215_status_t tue_sts3215_ping(tue_sts3215_t *dev, uint8_t id)
{
    /* PING expects standard 6-byte response packet (0 data bytes) */
    return sts3215_transceive(dev, id, STS3215_INST_PING, NULL, 0, NULL, 0);
}

tue_sts3215_status_t tue_sts3215_read_raw(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t reg_addr,
    uint8_t *data,
    uint8_t len)
{
    if (data == NULL || len == 0) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint8_t params[2] = { reg_addr, len };
    return sts3215_transceive(dev, id, STS3215_INST_READ, params, 2, data, len);
}

tue_sts3215_status_t tue_sts3215_write_raw(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t reg_addr,
    const uint8_t *data,
    uint8_t len)
{
    if (data == NULL || len == 0 || (len + 1U) > (STS3215_BUFFER_SIZE - 6U)) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint8_t params[STS3215_BUFFER_SIZE];
    params[0] = reg_addr;
    memcpy(&params[1], data, len);

    /* By default STS3215 sends 6-byte ACK response (0 data bytes) */
    uint8_t rx_expected = (id == STS3215_BROADCAST_ID) ? 0U : 1U;
    return sts3215_transceive(dev, id, STS3215_INST_WRITE, params, (uint16_t)(len + 1U), NULL, rx_expected);
}

tue_sts3215_status_t tue_sts3215_write_raw_no_ack(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t reg_addr,
    const uint8_t *data,
    uint8_t len)
{
    if (data == NULL || len == 0 || (len + 1U) > (STS3215_BUFFER_SIZE - 6U)) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint8_t params[STS3215_BUFFER_SIZE];
    params[0] = reg_addr;
    memcpy(&params[1], data, len);

    return sts3215_transceive(dev, id, STS3215_INST_WRITE, params, (uint16_t)(len + 1U), NULL, 0);
}

tue_sts3215_status_t tue_sts3215_reg_write_raw(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t reg_addr,
    const uint8_t *data,
    uint8_t len)
{
    if (data == NULL || len == 0 || (len + 1U) > (STS3215_BUFFER_SIZE - 6U)) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint8_t params[STS3215_BUFFER_SIZE];
    params[0] = reg_addr;
    memcpy(&params[1], data, len);

    uint8_t rx_expected = (id == STS3215_BROADCAST_ID) ? 0U : 1U;
    return sts3215_transceive(dev, id, STS3215_INST_REG_WRITE, params, (uint16_t)(len + 1U), NULL, rx_expected);
}

tue_sts3215_status_t tue_sts3215_action(tue_sts3215_t *dev, uint8_t id)
{
    /* ACTION triggers buffered REG_WRITE commands; no response is expected on broadcast */
    uint8_t rx_expected = (id == STS3215_BROADCAST_ID) ? 0U : 1U;
    return sts3215_transceive(dev, id, STS3215_INST_ACTION, NULL, 0, NULL, rx_expected);
}

tue_sts3215_status_t tue_sts3215_reset(tue_sts3215_t *dev, uint8_t id)
{
    uint8_t rx_expected = (id == STS3215_BROADCAST_ID) ? 0U : 1U;
    return sts3215_transceive(dev, id, STS3215_INST_RESET, NULL, 0, NULL, rx_expected);
}

tue_sts3215_status_t tue_sts3215_sync_write_raw(
    tue_sts3215_t *dev,
    uint8_t start_addr,
    uint8_t data_len,
    const uint8_t *param_data,
    uint16_t total_param_bytes)
{
    if (param_data == NULL || total_param_bytes == 0 || (total_param_bytes + 2U) > (STS3215_BUFFER_SIZE - 6U)) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint8_t params[STS3215_BUFFER_SIZE];
    params[0] = start_addr;
    params[1] = data_len;
    memcpy(&params[2], param_data, total_param_bytes);

    /* SYNC_WRITE is always sent to broadcast ID 0xFE; no reply is generated */
    return sts3215_transceive(
        dev,
        STS3215_BROADCAST_ID,
        STS3215_INST_SYNC_WRITE,
        params,
        (uint16_t)(total_param_bytes + 2U),
        NULL,
        0
    );
}

/* Primitive Typed Helpers ---------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_read_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t *val)
{
    if (val == NULL) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_read_raw(dev, id, reg_addr, val, 1);
}

tue_sts3215_status_t tue_sts3215_write_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t val)
{
    return tue_sts3215_write_raw(dev, id, reg_addr, &val, 1);
}

tue_sts3215_status_t tue_sts3215_read_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t *val)
{
    if (val == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, reg_addr, buf, 2);
    if (st == TUE_STS3215_OK) {
        *val = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_write_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)((val >> 8) & 0xFFU);
    return tue_sts3215_write_raw(dev, id, reg_addr, buf, 2);
}

tue_sts3215_status_t tue_sts3215_read_i16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, int16_t *val)
{
    return tue_sts3215_read_u16(dev, id, reg_addr, (uint16_t*)val);
}

tue_sts3215_status_t tue_sts3215_write_i16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, int16_t val)
{
    return tue_sts3215_write_u16(dev, id, reg_addr, (uint16_t)val);
}

tue_sts3215_status_t tue_sts3215_write_eeprom_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t val)
{
    /* Unlock EEPROM write lock */
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    /* Write value */
    st = tue_sts3215_write_u8(dev, id, reg_addr, val);
    if (st != TUE_STS3215_OK) return st;

    /* Relock EEPROM */
    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_write_eeprom_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t val)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    st = tue_sts3215_write_u16(dev, id, reg_addr, val);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

/* High-Level Functions (docs/MEMORY_TABLE.md) -------------------------------*/

/* EEPROM Lock Management (0x37) */

tue_sts3215_status_t tue_sts3215_lock_eeprom(tue_sts3215_t *dev, uint8_t id)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_LOCK_MARK, 1U);
}

tue_sts3215_status_t tue_sts3215_unlock_eeprom(tue_sts3215_t *dev, uint8_t id)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_LOCK_MARK, 0U);
}

tue_sts3215_status_t tue_sts3215_get_eeprom_lock(tue_sts3215_t *dev, uint8_t id, uint8_t *is_locked)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_LOCK_MARK, is_locked);
}

/* Versions & Device Markers (0x00..0x04) */

tue_sts3215_status_t tue_sts3215_get_fw_version(tue_sts3215_t *dev, uint8_t id, uint8_t *major, uint8_t *minor)
{
    if (major == NULL || minor == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, STS3215_REG_FW_MAJOR, buf, 2);
    if (st == TUE_STS3215_OK) {
        *major = buf[0];
        *minor = buf[1];
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_end_marker(tue_sts3215_t *dev, uint8_t id, uint8_t *end_val)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_END, end_val);
}

tue_sts3215_status_t tue_sts3215_get_servo_version(tue_sts3215_t *dev, uint8_t id, uint8_t *main_ver, uint8_t *sub_ver)
{
    if (main_ver == NULL || sub_ver == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, STS3215_REG_SERVO_MAIN_VER, buf, 2);
    if (st == TUE_STS3215_OK) {
        *main_ver = buf[0];
        *sub_ver  = buf[1];
    }
    return st;
}

/* Bus ID & Baud Rate (0x05, 0x06) */

tue_sts3215_status_t tue_sts3215_set_id(tue_sts3215_t *dev, uint8_t old_id, uint8_t new_id)
{
    if (new_id > 253U && new_id != STS3215_BROADCAST_ID) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, old_id, STS3215_REG_ID, new_id);
}

tue_sts3215_status_t tue_sts3215_get_id(tue_sts3215_t *dev, uint8_t id, uint8_t *current_id)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_ID, current_id);
}

tue_sts3215_status_t tue_sts3215_set_baud_rate(tue_sts3215_t *dev, uint8_t id, tue_sts3215_baud_t baud)
{
    if (baud > STS3215_BAUD_38400) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_BAUD_RATE, (uint8_t)baud);
}

tue_sts3215_status_t tue_sts3215_get_baud_rate(tue_sts3215_t *dev, uint8_t id, tue_sts3215_baud_t *baud)
{
    if (baud == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t val = 0;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_BAUD_RATE, &val);
    if (st == TUE_STS3215_OK) {
        *baud = (tue_sts3215_baud_t)val;
    }
    return st;
}

/* Return Delay & Response Level (0x07, 0x08) */

tue_sts3215_status_t tue_sts3215_set_return_delay(tue_sts3215_t *dev, uint8_t id, uint8_t delay_2us)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_RETURN_DELAY, delay_2us);
}

tue_sts3215_status_t tue_sts3215_get_return_delay(tue_sts3215_t *dev, uint8_t id, uint8_t *delay_2us)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_RETURN_DELAY, delay_2us);
}

tue_sts3215_status_t tue_sts3215_set_response_level(tue_sts3215_t *dev, uint8_t id, uint8_t level)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_RESPONSE_LEVEL, (level > 0U) ? 1U : 0U);
}

tue_sts3215_status_t tue_sts3215_get_response_level(tue_sts3215_t *dev, uint8_t id, uint8_t *level)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_RESPONSE_LEVEL, level);
}

/* Motion Travel Angle Limits (0x09, 0x0B) */

tue_sts3215_status_t tue_sts3215_set_angle_limits(tue_sts3215_t *dev, uint8_t id, uint16_t min_angle, uint16_t max_angle)
{
    if (min_angle >= max_angle || max_angle > 4095U) return TUE_STS3215_ERR_PARAM;
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[4];
    buf[0] = (uint8_t)(min_angle & 0xFFU);
    buf[1] = (uint8_t)((min_angle >> 8) & 0xFFU);
    buf[2] = (uint8_t)(max_angle & 0xFFU);
    buf[3] = (uint8_t)((max_angle >> 8) & 0xFFU);

    st = tue_sts3215_write_raw(dev, id, STS3215_REG_MIN_ANGLE_LIMIT, buf, 4);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_angle_limits(tue_sts3215_t *dev, uint8_t id, uint16_t *min_angle, uint16_t *max_angle)
{
    if (min_angle == NULL || max_angle == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[4];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, STS3215_REG_MIN_ANGLE_LIMIT, buf, 4);
    if (st == TUE_STS3215_OK) {
        *min_angle = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        *max_angle = (uint16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    }
    return st;
}

/* Temperature & Voltage Limits (0x0D, 0x0E, 0x0F) */

tue_sts3215_status_t tue_sts3215_set_max_temperature_limit(tue_sts3215_t *dev, uint8_t id, uint8_t max_temp_c)
{
    if (max_temp_c > 100U) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_MAX_TEMP_LIMIT, max_temp_c);
}

tue_sts3215_status_t tue_sts3215_get_max_temperature_limit(tue_sts3215_t *dev, uint8_t id, uint8_t *max_temp_c)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_MAX_TEMP_LIMIT, max_temp_c);
}

tue_sts3215_status_t tue_sts3215_set_voltage_limits(tue_sts3215_t *dev, uint8_t id, uint8_t min_0_1v, uint8_t max_0_1v)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[2] = { max_0_1v, min_0_1v };
    st = tue_sts3215_write_raw(dev, id, STS3215_REG_MAX_INPUT_VOLTAGE, buf, 2);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_voltage_limits(tue_sts3215_t *dev, uint8_t id, uint8_t *min_0_1v, uint8_t *max_0_1v)
{
    if (min_0_1v == NULL || max_0_1v == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, STS3215_REG_MAX_INPUT_VOLTAGE, buf, 2);
    if (st == TUE_STS3215_OK) {
        *max_0_1v = buf[0];
        *min_0_1v = buf[1];
    }
    return st;
}

/* Max Torque & Phase (0x10, 0x12) */

tue_sts3215_status_t tue_sts3215_set_max_torque(tue_sts3215_t *dev, uint8_t id, uint16_t max_torque_0_1pct)
{
    if (max_torque_0_1pct > 1000U) max_torque_0_1pct = 1000U;
    return tue_sts3215_write_eeprom_u16(dev, id, STS3215_REG_MAX_TORQUE, max_torque_0_1pct);
}

tue_sts3215_status_t tue_sts3215_get_max_torque(tue_sts3215_t *dev, uint8_t id, uint16_t *max_torque_0_1pct)
{
    return tue_sts3215_read_u16(dev, id, STS3215_REG_MAX_TORQUE, max_torque_0_1pct);
}

tue_sts3215_status_t tue_sts3215_set_phase(tue_sts3215_t *dev, uint8_t id, uint8_t phase)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_PHASE, phase);
}

tue_sts3215_status_t tue_sts3215_get_phase(tue_sts3215_t *dev, uint8_t id, uint8_t *phase)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_PHASE, phase);
}

/* Protection Unload & LED Alarm Conditions (0x13, 0x14) */

tue_sts3215_status_t tue_sts3215_set_unload_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t conditions_mask)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_UNLOAD_CONDITION, conditions_mask);
}

tue_sts3215_status_t tue_sts3215_get_unload_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t *conditions_mask)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_UNLOAD_CONDITION, conditions_mask);
}

tue_sts3215_status_t tue_sts3215_set_led_alarm_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t conditions_mask)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_LED_ALARM_CONDITION, conditions_mask);
}

tue_sts3215_status_t tue_sts3215_get_led_alarm_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t *conditions_mask)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_LED_ALARM_CONDITION, conditions_mask);
}

/* Position Loop PID Coefficients & Force (0x15, 0x16, 0x17, 0x18) */

tue_sts3215_status_t tue_sts3215_set_position_pid(tue_sts3215_t *dev, uint8_t id, uint8_t p, uint8_t d, uint8_t i)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[3] = { p, d, i };
    st = tue_sts3215_write_raw(dev, id, STS3215_REG_POS_P, buf, 3);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_position_pid(tue_sts3215_t *dev, uint8_t id, uint8_t *p, uint8_t *d, uint8_t *i)
{
    if (p == NULL || d == NULL || i == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[3];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, STS3215_REG_POS_P, buf, 3);
    if (st == TUE_STS3215_OK) {
        *p = buf[0];
        *d = buf[1];
        *i = buf[2];
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_set_min_starting_force(tue_sts3215_t *dev, uint8_t id, uint8_t force_0_1pct)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_MIN_START_FORCE, force_0_1pct);
}

tue_sts3215_status_t tue_sts3215_get_min_starting_force(tue_sts3215_t *dev, uint8_t id, uint8_t *force_0_1pct)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_MIN_START_FORCE, force_0_1pct);
}

/* Points Limit & Insensitive Deadband (0x19, 0x1A, 0x1B) */

tue_sts3215_status_t tue_sts3215_set_points_limit(tue_sts3215_t *dev, uint8_t id, uint8_t limit)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_POINTS_LIMIT, limit);
}

tue_sts3215_status_t tue_sts3215_get_points_limit(tue_sts3215_t *dev, uint8_t id, uint8_t *limit)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_POINTS_LIMIT, limit);
}

tue_sts3215_status_t tue_sts3215_set_deadband(tue_sts3215_t *dev, uint8_t id, uint8_t cw_deadband, uint8_t ccw_deadband)
{
    if (cw_deadband > 32U || ccw_deadband > 32U) return TUE_STS3215_ERR_PARAM;
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[2] = { cw_deadband, ccw_deadband };
    st = tue_sts3215_write_raw(dev, id, STS3215_REG_CW_DEADBAND, buf, 2);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_deadband(tue_sts3215_t *dev, uint8_t id, uint8_t *cw_deadband, uint8_t *ccw_deadband)
{
    if (cw_deadband == NULL || ccw_deadband == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, STS3215_REG_CW_DEADBAND, buf, 2);
    if (st == TUE_STS3215_OK) {
        *cw_deadband  = buf[0];
        *ccw_deadband = buf[1];
    }
    return st;
}

/* Protective Current, Resolution & Position Correction (0x1C, 0x1E, 0x1F) */

tue_sts3215_status_t tue_sts3215_set_protective_current(tue_sts3215_t *dev, uint8_t id, uint16_t current_6_5ma)
{
    if (current_6_5ma > 511U) current_6_5ma = 511U;
    return tue_sts3215_write_eeprom_u16(dev, id, STS3215_REG_PROTECTIVE_CURRENT, current_6_5ma);
}

tue_sts3215_status_t tue_sts3215_get_protective_current(tue_sts3215_t *dev, uint8_t id, uint16_t *current_6_5ma)
{
    return tue_sts3215_read_u16(dev, id, STS3215_REG_PROTECTIVE_CURRENT, current_6_5ma);
}

tue_sts3215_status_t tue_sts3215_set_angular_resolution(tue_sts3215_t *dev, uint8_t id, uint8_t resolution)
{
    if (resolution < 1U || resolution > 3U) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_ANGULAR_RESOLUTION, resolution);
}

tue_sts3215_status_t tue_sts3215_get_angular_resolution(tue_sts3215_t *dev, uint8_t id, uint8_t *resolution)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_ANGULAR_RESOLUTION, resolution);
}

tue_sts3215_status_t tue_sts3215_set_position_correction(tue_sts3215_t *dev, uint8_t id, int16_t offset_steps)
{
    uint16_t raw = sts3215_offset_to_raw(offset_steps);
    return tue_sts3215_write_eeprom_u16(dev, id, STS3215_REG_POSITION_CORRECTION, raw);
}

tue_sts3215_status_t tue_sts3215_get_position_correction(tue_sts3215_t *dev, uint8_t id, int16_t *offset_steps)
{
    if (offset_steps == NULL) return TUE_STS3215_ERR_PARAM;
    uint16_t raw = 0;
    tue_sts3215_status_t st = tue_sts3215_read_u16(dev, id, STS3215_REG_POSITION_CORRECTION, &raw);
    if (st == TUE_STS3215_OK) {
        *offset_steps = sts3215_raw_to_offset(raw);
    }
    return st;
}

/* Operating Mode (0x21) */

tue_sts3215_status_t tue_sts3215_set_operation_mode(tue_sts3215_t *dev, uint8_t id, tue_sts3215_mode_t mode)
{
    if (mode > STS3215_MODE_STEPPER) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_OPERATION_MODE, (uint8_t)mode);
}

tue_sts3215_status_t tue_sts3215_get_operation_mode(tue_sts3215_t *dev, uint8_t id, tue_sts3215_mode_t *mode)
{
    if (mode == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t val = 0;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_OPERATION_MODE, &val);
    if (st == TUE_STS3215_OK) {
        *mode = (tue_sts3215_mode_t)val;
    }
    return st;
}

/* Overload & Protective Torque / Time (0x22, 0x23, 0x24) */

tue_sts3215_status_t tue_sts3215_set_overload_protection(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t protective_torque_pct,
    uint8_t protection_time_10ms,
    uint8_t overload_torque_pct)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[3] = { protective_torque_pct, protection_time_10ms, overload_torque_pct };
    st = tue_sts3215_write_raw(dev, id, STS3215_REG_PROTECTIVE_TORQUE, buf, 3);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_overload_protection(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t *protective_torque_pct,
    uint8_t *protection_time_10ms,
    uint8_t *overload_torque_pct)
{
    if (protective_torque_pct == NULL || protection_time_10ms == NULL || overload_torque_pct == NULL) {
        return TUE_STS3215_ERR_PARAM;
    }
    uint8_t buf[3];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, STS3215_REG_PROTECTIVE_TORQUE, buf, 3);
    if (st == TUE_STS3215_OK) {
        *protective_torque_pct = buf[0];
        *protection_time_10ms  = buf[1];
        *overload_torque_pct   = buf[2];
    }
    return st;
}

/* Speed Loop PID & Overcurrent Protection Time (0x25, 0x26, 0x27) */

tue_sts3215_status_t tue_sts3215_set_speed_pid(tue_sts3215_t *dev, uint8_t id, uint8_t speed_p, uint8_t speed_i)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    st = tue_sts3215_write_u8(dev, id, STS3215_REG_SPEED_P, speed_p);
    if (st != TUE_STS3215_OK) return st;

    st = tue_sts3215_write_u8(dev, id, STS3215_REG_SPEED_I, speed_i);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_speed_pid(tue_sts3215_t *dev, uint8_t id, uint8_t *speed_p, uint8_t *speed_i)
{
    if (speed_p == NULL || speed_i == NULL) return TUE_STS3215_ERR_PARAM;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_SPEED_P, speed_p);
    if (st != TUE_STS3215_OK) return st;
    return tue_sts3215_read_u8(dev, id, STS3215_REG_SPEED_I, speed_i);
}

tue_sts3215_status_t tue_sts3215_set_overcurrent_time(tue_sts3215_t *dev, uint8_t id, uint8_t time_10ms)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_OVERCURRENT_TIME, time_10ms);
}

tue_sts3215_status_t tue_sts3215_get_overcurrent_time(tue_sts3215_t *dev, uint8_t id, uint8_t *time_10ms)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_OVERCURRENT_TIME, time_10ms);
}

/* Torque Enable & Zero-Point Calibration (0x28) */

tue_sts3215_status_t tue_sts3215_enable_torque(tue_sts3215_t *dev, uint8_t id, bool enable)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_TORQUE_ENABLE, enable ? 1U : 0U);
}

tue_sts3215_status_t tue_sts3215_calibrate_position_to_2048(tue_sts3215_t *dev, uint8_t id)
{
    /* Writing 128 (0x80) corrects the current angle position to 2048 */
    return tue_sts3215_write_u8(dev, id, STS3215_REG_TORQUE_ENABLE, 128U);
}

tue_sts3215_status_t tue_sts3215_get_torque_enable(tue_sts3215_t *dev, uint8_t id, bool *enabled)
{
    if (enabled == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t val = 0;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_TORQUE_ENABLE, &val);
    if (st == TUE_STS3215_OK) {
        *enabled = (val == 1U);
    }
    return st;
}

/* Acceleration, Position & Speed Control (0x29, 0x2A, 0x2C, 0x2E, 0x30) */

tue_sts3215_status_t tue_sts3215_set_acceleration(tue_sts3215_t *dev, uint8_t id, uint8_t accel_100steps_s2)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_ACCELERATION, accel_100steps_s2);
}

tue_sts3215_status_t tue_sts3215_get_acceleration(tue_sts3215_t *dev, uint8_t id, uint8_t *accel_100steps_s2)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_ACCELERATION, accel_100steps_s2);
}

tue_sts3215_status_t tue_sts3215_set_goal_position(tue_sts3215_t *dev, uint8_t id, int16_t position)
{
    return tue_sts3215_write_i16(dev, id, STS3215_REG_TARGET_POSITION, position);
}

tue_sts3215_status_t tue_sts3215_set_goal_position_ex(
    tue_sts3215_t *dev,
    uint8_t id,
    int16_t position,
    int16_t speed,
    uint8_t accel)
{
    /* Writing registers 0x29..0x2F (7 bytes: Accel, Position, Runtime, Speed) */
    uint8_t buf[7];
    buf[0] = accel;                                    /* 0x29: Accel */
    buf[1] = (uint8_t)((uint16_t)position & 0xFFU);       /* 0x2A: Pos Low */
    buf[2] = (uint8_t)(((uint16_t)position >> 8) & 0xFFU);/* 0x2B: Pos High */
    buf[3] = 0U;                                       /* 0x2C: Runtime Low */
    buf[4] = 0U;                                       /* 0x2D: Runtime High */
    buf[5] = (uint8_t)((uint16_t)speed & 0xFFU);          /* 0x2E: Speed Low */
    buf[6] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFU);   /* 0x2F: Speed High */

    return tue_sts3215_write_raw(dev, id, STS3215_REG_ACCELERATION, buf, 7);
}

tue_sts3215_status_t tue_sts3215_reg_write_goal_position_ex(
    tue_sts3215_t *dev,
    uint8_t id,
    int16_t position,
    int16_t speed,
    uint8_t accel)
{
    uint8_t buf[7];
    buf[0] = accel;
    buf[1] = (uint8_t)((uint16_t)position & 0xFFU);
    buf[2] = (uint8_t)(((uint16_t)position >> 8) & 0xFFU);
    buf[3] = 0U;
    buf[4] = 0U;
    buf[5] = (uint8_t)((uint16_t)speed & 0xFFU);
    buf[6] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFU);

    return tue_sts3215_reg_write_raw(dev, id, STS3215_REG_ACCELERATION, buf, 7);
}

tue_sts3215_status_t tue_sts3215_set_pwm_runtime(tue_sts3215_t *dev, uint8_t id, int16_t pwm_per_thousand)
{
    uint16_t raw = sts3215_pwm_to_raw(pwm_per_thousand);
    return tue_sts3215_write_u16(dev, id, STS3215_REG_RUNTIME, raw);
}

tue_sts3215_status_t tue_sts3215_get_pwm_runtime(tue_sts3215_t *dev, uint8_t id, int16_t *pwm_per_thousand)
{
    if (pwm_per_thousand == NULL) return TUE_STS3215_ERR_PARAM;
    uint16_t raw = 0;
    tue_sts3215_status_t st = tue_sts3215_read_u16(dev, id, STS3215_REG_RUNTIME, &raw);
    if (st == TUE_STS3215_OK) {
        *pwm_per_thousand = sts3215_raw_to_pwm(raw);
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_set_running_speed(tue_sts3215_t *dev, uint8_t id, int16_t speed_steps_s)
{
    return tue_sts3215_write_i16(dev, id, STS3215_REG_RUNNING_SPEED, speed_steps_s);
}

tue_sts3215_status_t tue_sts3215_get_running_speed(tue_sts3215_t *dev, uint8_t id, int16_t *speed_steps_s)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_RUNNING_SPEED, speed_steps_s);
}

tue_sts3215_status_t tue_sts3215_set_torque_limit(tue_sts3215_t *dev, uint8_t id, uint16_t torque_0_1pct)
{
    if (torque_0_1pct > 1000U) torque_0_1pct = 1000U;
    return tue_sts3215_write_u16(dev, id, STS3215_REG_TORQUE_LIMIT, torque_0_1pct);
}

tue_sts3215_status_t tue_sts3215_get_torque_limit(tue_sts3215_t *dev, uint8_t id, uint16_t *torque_0_1pct)
{
    return tue_sts3215_read_u16(dev, id, STS3215_REG_TORQUE_LIMIT, torque_0_1pct);
}

tue_sts3215_status_t tue_sts3215_stop(tue_sts3215_t *dev, uint8_t id)
{
    /* Stopping motion: set running speed to 0 */
    return tue_sts3215_set_running_speed(dev, id, 0);
}

/* Feedback & Telemetry (0x38..0x45) -----------------------------------------*/

tue_sts3215_status_t tue_sts3215_get_current_position(tue_sts3215_t *dev, uint8_t id, int16_t *position)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_CURRENT_POSITION, position);
}

tue_sts3215_status_t tue_sts3215_get_current_speed(tue_sts3215_t *dev, uint8_t id, int16_t *speed)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_CURRENT_SPEED, speed);
}

tue_sts3215_status_t tue_sts3215_get_current_load(tue_sts3215_t *dev, uint8_t id, int16_t *load_0_1pct)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_CURRENT_LOAD, load_0_1pct);
}

tue_sts3215_status_t tue_sts3215_get_current_voltage(tue_sts3215_t *dev, uint8_t id, uint8_t *voltage_0_1v)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_CURRENT_VOLTAGE, voltage_0_1v);
}

tue_sts3215_status_t tue_sts3215_get_voltage_volts(tue_sts3215_t *dev, uint8_t id, float *volts)
{
    if (volts == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t raw = 0;
    tue_sts3215_status_t st = tue_sts3215_get_current_voltage(dev, id, &raw);
    if (st == TUE_STS3215_OK) {
        *volts = (float)raw * 0.1f;
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_current_temperature(tue_sts3215_t *dev, uint8_t id, int8_t *temp_c)
{
    return tue_sts3215_read_raw(dev, id, STS3215_REG_CURRENT_TEMP, (uint8_t*)temp_c, 1);
}

tue_sts3215_status_t tue_sts3215_get_async_write_flag(tue_sts3215_t *dev, uint8_t id, uint8_t *flag)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_ASYNC_WRITE_FLAG, flag);
}

tue_sts3215_status_t tue_sts3215_get_servo_status(tue_sts3215_t *dev, uint8_t id, uint8_t *status_mask)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_SERVO_STATUS, status_mask);
}

tue_sts3215_status_t tue_sts3215_is_moving(tue_sts3215_t *dev, uint8_t id, bool *is_moving)
{
    if (is_moving == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t val = 0;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_MOVING_FLAG, &val);
    if (st == TUE_STS3215_OK) {
        *is_moving = (val != 0U);
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_motor_current_raw(tue_sts3215_t *dev, uint8_t id, int16_t *current_6_5ma)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_CURRENT_CURRENT, current_6_5ma);
}

tue_sts3215_status_t tue_sts3215_get_motor_current_ma(tue_sts3215_t *dev, uint8_t id, float *current_ma)
{
    if (current_ma == NULL) return TUE_STS3215_ERR_PARAM;
    int16_t raw = 0;
    tue_sts3215_status_t st = tue_sts3215_get_motor_current_raw(dev, id, &raw);
    if (st == TUE_STS3215_OK) {
        *current_ma = (float)raw * 6.5f;
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_feedback(tue_sts3215_t *dev, uint8_t id, tue_sts3215_feedback_t *feedback)
{
    if (feedback == NULL) return TUE_STS3215_ERR_PARAM;

    /* Burst read 15 continuous bytes starting at address 0x38 up to 0x46:
     * 0x38..0x39: Position (2B)
     * 0x3A..0x3B: Speed (2B)
     * 0x3C..0x3D: Load (2B)
     * 0x3E:       Voltage (1B)
     * 0x3F:       Temperature (1B)
     * 0x40:       Async write flag (1B)
     * 0x41:       Servo status (1B)
     * 0x42:       Moving flag (1B)
     * 0x43..0x44: Undefined (2B)
     * 0x45..0x46: Current (2B)
     */
    uint8_t raw[15];
    tue_sts3215_status_t st = tue_sts3215_read_raw(dev, id, STS3215_REG_CURRENT_POSITION, raw, 15);
    if (st != TUE_STS3215_OK && st != TUE_STS3215_ERR_STATUS) {
        return st;
    }

    feedback->id               = id;
    feedback->position         = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    feedback->speed            = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    feedback->load             = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
    feedback->voltage_raw      = raw[6];
    feedback->voltage_v        = (float)raw[6] * 0.1f;
    feedback->temperature_degc = (int8_t)raw[7];
    feedback->async_flag       = raw[8];
    feedback->status_mask      = raw[9];
    feedback->is_moving        = raw[10];
    feedback->current_raw      = (int16_t)((uint16_t)raw[13] | ((uint16_t)raw[14] << 8));
    feedback->current_ma       = (float)feedback->current_raw * 6.5f;

    return st;
}

/* Multi-Servo Synchronized Commands -----------------------------------------*/

tue_sts3215_status_t tue_sts3215_sync_write_position(
    tue_sts3215_t *dev,
    const tue_sts3215_sync_pos_t *targets,
    uint8_t count)
{
    if (targets == NULL || count == 0) return TUE_STS3215_ERR_PARAM;

    /* Starting register: STS3215_REG_ACCELERATION (0x29)
     * Length per servo: 7 bytes (Accel(1), Pos(2), Runtime(2), Speed(2))
     * Each entry format: [ID, Accel, Pos_L, Pos_H, Runtime_L, Runtime_H, Spd_L, Spd_H]
     */
    uint8_t payload[STS3215_BUFFER_SIZE];
    uint16_t idx = 0;

    for (uint8_t i = 0; i < count; i++) {
        if ((idx + 8U) > (sizeof(payload) - 6U)) {
            return TUE_STS3215_ERR_PARAM;
        }

        payload[idx++] = targets[i].id;
        payload[idx++] = targets[i].accel;
        payload[idx++] = (uint8_t)((uint16_t)targets[i].position & 0xFFU);
        payload[idx++] = (uint8_t)(((uint16_t)targets[i].position >> 8) & 0xFFU);
        payload[idx++] = 0U; /* Runtime L */
        payload[idx++] = 0U; /* Runtime H */
        payload[idx++] = (uint8_t)((uint16_t)targets[i].speed & 0xFFU);
        payload[idx++] = (uint8_t)(((uint16_t)targets[i].speed >> 8) & 0xFFU);
    }

    return tue_sts3215_sync_write_raw(dev, STS3215_REG_ACCELERATION, 7U, payload, idx);
}

tue_sts3215_status_t tue_sts3215_sync_write_speed(
    tue_sts3215_t *dev,
    const tue_sts3215_sync_speed_t *targets,
    uint8_t count)
{
    if (targets == NULL || count == 0) return TUE_STS3215_ERR_PARAM;

    /* Starting register: STS3215_REG_RUNNING_SPEED (0x2E)
     * Length per servo: 2 bytes (Speed_L, Speed_H)
     * Each entry: [ID, Speed_L, Speed_H]
     */
    uint8_t payload[STS3215_BUFFER_SIZE];
    uint16_t idx = 0;

    for (uint8_t i = 0; i < count; i++) {
        if ((idx + 3U) > (sizeof(payload) - 6U)) {
            return TUE_STS3215_ERR_PARAM;
        }

        payload[idx++] = targets[i].id;
        payload[idx++] = (uint8_t)((uint16_t)targets[i].speed & 0xFFU);
        payload[idx++] = (uint8_t)(((uint16_t)targets[i].speed >> 8) & 0xFFU);
    }

    return tue_sts3215_sync_write_raw(dev, STS3215_REG_RUNNING_SPEED, 2U, payload, idx);
}

tue_sts3215_status_t tue_sts3215_sync_write_torque_enable(
    tue_sts3215_t *dev,
    const uint8_t *ids,
    const uint8_t *enables,
    uint8_t count)
{
    if (ids == NULL || enables == NULL || count == 0) return TUE_STS3215_ERR_PARAM;

    /* Starting register: STS3215_REG_TORQUE_ENABLE (0x28)
     * Length per servo: 1 byte
     * Entry: [ID, Enable]
     */
    uint8_t payload[STS3215_BUFFER_SIZE];
    uint16_t idx = 0;

    for (uint8_t i = 0; i < count; i++) {
        if ((idx + 2U) > (sizeof(payload) - 6U)) {
            return TUE_STS3215_ERR_PARAM;
        }

        payload[idx++] = ids[i];
        payload[idx++] = enables[i] ? 1U : 0U;
    }

    return tue_sts3215_sync_write_raw(dev, STS3215_REG_TORQUE_ENABLE, 1U, payload, idx);
}
