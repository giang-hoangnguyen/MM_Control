/**
  ******************************************************************************
  * @file           : tue_usb.c
  * @brief          : TUE USB binary protocol implementation for STM32F407
  ******************************************************************************
  * @attention
  *
  * Robust, modular binary request/response protocol over USB CDC.
  * Handles streaming fragmentation/concatenation, CRC-16 verification,
  * sequence numbers, dynamic/static command table dispatch, and
  * decoupled FreeRTOS task execution.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_usb.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "tue_sts3215.h"
#include <string.h>

/* Private Defines -----------------------------------------------------------*/
#define RX_STREAM_BUFFER_SIZE       1024U
#define RX_STREAM_TRIGGER_LEVEL     1U
#define TUE_MAX_COMMANDS            48U
#define TX_TIMEOUT_MS               100U
#define TX_WAIT_MS                  50U

/* STM32F4 Unique Device ID Base Address (96 bits = 12 bytes) */
#ifndef UID_BASE
#define UID_BASE                    0x1FFF7A10UL
#endif

/* Private Typedefs ----------------------------------------------------------*/
typedef enum {
    PARSE_STATE_WAIT_SOF1 = 0,
    PARSE_STATE_WAIT_SOF2,
    PARSE_STATE_SEQ,
    PARSE_STATE_CMD,
    PARSE_STATE_STATUS,
    PARSE_STATE_LEN_L,
    PARSE_STATE_LEN_H,
    PARSE_STATE_PAYLOAD,
    PARSE_STATE_CRC_L,
    PARSE_STATE_CRC_H
} parse_state_t;

typedef struct {
    uint8_t            cmd_id;
    uint16_t           min_req_len;
    uint16_t           max_req_len;
    tue_cmd_handler_t  handler;
} cmd_entry_t;

/* Private Constants: CRC-16/CCITT-FALSE Lookup Table ------------------------*/
static const uint16_t s_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/* Private Variables ---------------------------------------------------------*/
static StreamBufferHandle_t   s_rx_stream_buffer = NULL;
static osMutexId_t            s_tx_mutex = NULL;
static osThreadId_t           s_tue_usb_task_handle = NULL;

/* Parser State */
static parse_state_t          s_parse_state = PARSE_STATE_WAIT_SOF1;
static tue_frame_t            s_current_frame;
static uint16_t               s_payload_recv_count = 0;

/* Command Registry Table */
static cmd_entry_t            s_cmd_table[TUE_MAX_COMMANDS];
static uint8_t                s_cmd_count = 0;

/* Static TX Buffers */
static uint8_t                s_tx_frame_buffer[TUE_USB_MAX_FRAME_SIZE];
static uint8_t                s_resp_payload_buffer[TUE_USB_MAX_PAYLOAD];

/* Motor Subsystem State Variables */
typedef struct {
    uint8_t  direction;
    uint16_t duty;
    float    kp;
    float    ki;
    float    kd;
} dc_motor_state_t;

typedef struct {
    bool     enabled;
    int32_t  current_steps;
    int32_t  target_steps;
    uint32_t speed_hz;
    uint32_t accel;
    bool     is_moving;
} step_motor_state_t;

static dc_motor_state_t   s_dc_motors[4];
static step_motor_state_t s_steppers[4];
static tue_sts3215_t      s_servo_bus;

/* Private Function Prototypes -----------------------------------------------*/
static void         tue_usb_task_entry(void *argument);
static void         tue_usb_process_frame(const tue_frame_t *req);
static uint16_t     tue_usb_calculate_frame_crc(const tue_frame_t *frame);
static tue_status_t handle_sys_ping(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_sys_get_info(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_sys_led_ctrl(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);

/* Motor Command Handlers */
static tue_status_t handle_dc_set_duty(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_dc_stop(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_dc_get_status(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_dc_set_pid(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);

static tue_status_t handle_step_enable(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_step_move(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_step_stop(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_step_get_pos(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_step_set_homing(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);

static tue_status_t handle_servo_ping(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_servo_read_reg(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_servo_write_reg(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_servo_set_pos(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_servo_get_feedback(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);
static tue_status_t handle_servo_torque_enable(const uint8_t *req, uint16_t req_len, uint8_t *resp, uint16_t *resp_len, uint16_t max_resp);

/* Public Functions ----------------------------------------------------------*/

uint16_t tue_usb_crc16(const uint8_t *data, size_t len, uint16_t init_crc)
{
    uint16_t crc = init_crc;
    for (size_t i = 0; i < len; i++) {
        crc = (uint16_t)((crc << 8) ^ s_crc16_table[((crc >> 8) ^ data[i]) & 0xFFU]);
    }
    return crc;
}

tue_status_t tue_usb_register_command(
    uint8_t cmd_id,
    uint16_t min_req_len,
    uint16_t max_req_len,
    tue_cmd_handler_t handler)
{
    if (handler == NULL) {
        return TUE_STATUS_ERR_INVALID_PARAM;
    }

    /* Check if already registered (replace) */
    for (uint8_t i = 0; i < s_cmd_count; i++) {
        if (s_cmd_table[i].cmd_id == cmd_id) {
            s_cmd_table[i].min_req_len = min_req_len;
            s_cmd_table[i].max_req_len = max_req_len;
            s_cmd_table[i].handler = handler;
            return TUE_STATUS_OK;
        }
    }

    if (s_cmd_count >= TUE_MAX_COMMANDS) {
        return TUE_STATUS_ERR_BUSY;
    }

    s_cmd_table[s_cmd_count].cmd_id = cmd_id;
    s_cmd_table[s_cmd_count].min_req_len = min_req_len;
    s_cmd_table[s_cmd_count].max_req_len = max_req_len;
    s_cmd_table[s_cmd_count].handler = handler;
    s_cmd_count++;

    return TUE_STATUS_OK;
}

void tue_usb_init(void)
{
    /* 1. Create FreeRTOS stream buffer for non-blocking ISR ingestion */
    if (s_rx_stream_buffer == NULL) {
        s_rx_stream_buffer = xStreamBufferCreate(RX_STREAM_BUFFER_SIZE, RX_STREAM_TRIGGER_LEVEL);
    }

    /* 2. Create CMSIS-RTOS2 mutex for safe USB transmissions */
    if (s_tx_mutex == NULL) {
        const osMutexAttr_t mutex_attr = {
            .name = "tue_tx_mutex",
            .attr_bits = osMutexPrioInherit
        };
        s_tx_mutex = osMutexNew(&mutex_attr);
    }

    /* 3. Register default built-in commands */
    tue_usb_register_command(TUE_CMD_SYS_PING,     0, TUE_USB_MAX_PAYLOAD, handle_sys_ping);
    tue_usb_register_command(TUE_CMD_SYS_GET_INFO, 0, 0,                   handle_sys_get_info);
    tue_usb_register_command(TUE_CMD_SYS_LED_CTRL, 2, 4,                   handle_sys_led_ctrl);

    /* Initialize motor subsystems */
    tue_motor_init();

    /* Register DC motor commands (0x10–0x13) */
    tue_usb_register_command(TUE_CMD_DC_SET_DUTY,          4, 4, handle_dc_set_duty);
    tue_usb_register_command(TUE_CMD_DC_STOP,              2, 2, handle_dc_stop);
    tue_usb_register_command(TUE_CMD_DC_GET_STATUS,        1, 1, handle_dc_get_status);
    tue_usb_register_command(TUE_CMD_DC_SET_PID,          13, 14, handle_dc_set_pid);

    /* Register Stepper motor commands (0x20–0x24) */
    tue_usb_register_command(TUE_CMD_STEP_ENABLE,          2, 2, handle_step_enable);
    tue_usb_register_command(TUE_CMD_STEP_MOVE,           14, 14, handle_step_move);
    tue_usb_register_command(TUE_CMD_STEP_STOP,            2, 2, handle_step_stop);
    tue_usb_register_command(TUE_CMD_STEP_GET_POS,         1, 1, handle_step_get_pos);
    tue_usb_register_command(TUE_CMD_STEP_SET_HOMING,      2, 2, handle_step_set_homing);

    /* Register Servo STS3215 commands (0x30–0x35) */
    tue_usb_register_command(TUE_CMD_SERVO_PING,           1, 1, handle_servo_ping);
    tue_usb_register_command(TUE_CMD_SERVO_READ_REG,       3, 3, handle_servo_read_reg);
    tue_usb_register_command(TUE_CMD_SERVO_WRITE_REG,      3, TUE_USB_MAX_PAYLOAD, handle_servo_write_reg);
    tue_usb_register_command(TUE_CMD_SERVO_SET_POS,        7, 7, handle_servo_set_pos);
    tue_usb_register_command(TUE_CMD_SERVO_GET_FEEDBACK,   1, 1, handle_servo_get_feedback);
    tue_usb_register_command(TUE_CMD_SERVO_TORQUE_ENABLE,  2, 2, handle_servo_torque_enable);

    /* 4. Spawn protocol worker task */
    if (s_tue_usb_task_handle == NULL) {
        const osThreadAttr_t task_attr = {
            .name = "tue_usb_task",
            .stack_size = 512 * 4, /* 2048 bytes */
            .priority = (osPriority_t)osPriorityAboveNormal
        };
        s_tue_usb_task_handle = osThreadNew(tue_usb_task_entry, NULL, &task_attr);
    }
}

void tue_usb_on_cdc_rx(const uint8_t *data, uint32_t len)
{
    if (s_rx_stream_buffer != NULL && data != NULL && len > 0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xStreamBufferSendFromISR(s_rx_stream_buffer, data, (size_t)len, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

tue_status_t tue_usb_send_response(
    uint8_t seq,
    uint8_t cmd,
    tue_status_t status,
    const uint8_t *payload,
    uint16_t len)
{
    if (len > TUE_USB_MAX_PAYLOAD) {
        return TUE_STATUS_ERR_INVALID_LEN;
    }

    if (s_tx_mutex == NULL) {
        return TUE_STATUS_ERR_BUSY;
    }

    /* Acquire transmission mutex */
    if (osMutexAcquire(s_tx_mutex, TX_TIMEOUT_MS) != osOK) {
        return TUE_STATUS_ERR_BUSY;
    }

    /* Verify USB CDC is configured */
    extern USBD_HandleTypeDef hUsbDeviceFS;
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        osMutexRelease(s_tx_mutex);
        return TUE_STATUS_ERR_BUSY;
    }

    /* Wait if previous transmission still pending */
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) {
        osMutexRelease(s_tx_mutex);
        return TUE_STATUS_ERR_BUSY;
    }

    uint32_t start_tick = osKernelGetTickCount();
    while (hcdc->TxState != 0) {
        if ((osKernelGetTickCount() - start_tick) > TX_WAIT_MS) {
            osMutexRelease(s_tx_mutex);
            return TUE_STATUS_ERR_BUSY;
        }
        osDelay(1);
    }

    /* Build Frame: SOF1, SOF2, SEQ, CMD, STATUS, LEN_L, LEN_H, PAYLOAD..., CRC_L, CRC_H */
    s_tx_frame_buffer[0] = TUE_USB_SOF1;
    s_tx_frame_buffer[1] = TUE_USB_SOF2;
    s_tx_frame_buffer[2] = seq;
    s_tx_frame_buffer[3] = cmd;
    s_tx_frame_buffer[4] = (uint8_t)status;
    s_tx_frame_buffer[5] = (uint8_t)(len & 0xFFU);
    s_tx_frame_buffer[6] = (uint8_t)((len >> 8) & 0xFFU);

    if (payload != NULL && len > 0) {
        memcpy(&s_tx_frame_buffer[7], payload, len);
    }

    /* Calculate CRC over SEQ through end of PAYLOAD */
    uint16_t crc = tue_usb_crc16(&s_tx_frame_buffer[2], (size_t)(5U + len), 0xFFFFU);
    uint16_t crc_offset = 7U + len;
    s_tx_frame_buffer[crc_offset]     = (uint8_t)(crc & 0xFFU);
    s_tx_frame_buffer[crc_offset + 1] = (uint8_t)((crc >> 8) & 0xFFU);

    uint16_t total_len = crc_offset + 2U;

    uint8_t tx_result = CDC_Transmit_FS(s_tx_frame_buffer, total_len);

    /* Wait briefly for endpoint to accept */
    if (tx_result == USBD_OK) {
        uint32_t wait_start = osKernelGetTickCount();
        while (hcdc->TxState != 0) {
            if ((osKernelGetTickCount() - wait_start) > TX_WAIT_MS) {
                break;
            }
            osDelay(1);
        }
    }

    osMutexRelease(s_tx_mutex);
    return (tx_result == USBD_OK) ? TUE_STATUS_OK : TUE_STATUS_ERR_BUSY;
}

void tue_usb_feed_byte(uint8_t byte)
{
    switch (s_parse_state) {
        case PARSE_STATE_WAIT_SOF1:
            if (byte == TUE_USB_SOF1) {
                s_parse_state = PARSE_STATE_WAIT_SOF2;
            }
            break;

        case PARSE_STATE_WAIT_SOF2:
            if (byte == TUE_USB_SOF2) {
                s_parse_state = PARSE_STATE_SEQ;
            } else if (byte == TUE_USB_SOF1) {
                /* Consecutive 0xAA markers */
                s_parse_state = PARSE_STATE_WAIT_SOF2;
            } else {
                s_parse_state = PARSE_STATE_WAIT_SOF1;
            }
            break;

        case PARSE_STATE_SEQ:
            s_current_frame.seq = byte;
            s_parse_state = PARSE_STATE_CMD;
            break;

        case PARSE_STATE_CMD:
            s_current_frame.cmd = byte;
            s_parse_state = PARSE_STATE_STATUS;
            break;

        case PARSE_STATE_STATUS:
            s_current_frame.status = byte;
            s_parse_state = PARSE_STATE_LEN_L;
            break;

        case PARSE_STATE_LEN_L:
            s_current_frame.len = (uint16_t)byte;
            s_parse_state = PARSE_STATE_LEN_H;
            break;

        case PARSE_STATE_LEN_H:
            s_current_frame.len |= ((uint16_t)byte << 8);
            if (s_current_frame.len > TUE_USB_MAX_PAYLOAD) {
                /* Length exceeds buffer limit, drop frame and recover */
                s_parse_state = PARSE_STATE_WAIT_SOF1;
            } else if (s_current_frame.len == 0) {
                s_parse_state = PARSE_STATE_CRC_L;
            } else {
                s_payload_recv_count = 0;
                s_parse_state = PARSE_STATE_PAYLOAD;
            }
            break;

        case PARSE_STATE_PAYLOAD:
            s_current_frame.payload[s_payload_recv_count++] = byte;
            if (s_payload_recv_count >= s_current_frame.len) {
                s_parse_state = PARSE_STATE_CRC_L;
            }
            break;

        case PARSE_STATE_CRC_L:
            s_current_frame.crc = (uint16_t)byte;
            s_parse_state = PARSE_STATE_CRC_H;
            break;

        case PARSE_STATE_CRC_H:
            s_current_frame.crc |= ((uint16_t)byte << 8);

            /* Frame received completely. Verify CRC. */
            uint16_t computed_crc = tue_usb_calculate_frame_crc(&s_current_frame);
            if (computed_crc == s_current_frame.crc) {
                tue_usb_process_frame(&s_current_frame);
            } else {
                /* Report CRC error */
                tue_usb_send_response(
                    s_current_frame.seq,
                    s_current_frame.cmd,
                    TUE_STATUS_ERR_CRC,
                    NULL,
                    0
                );
            }
            s_parse_state = PARSE_STATE_WAIT_SOF1;
            break;

        default:
            s_parse_state = PARSE_STATE_WAIT_SOF1;
            break;
    }
}

/* Private Worker Functions --------------------------------------------------*/

static uint16_t tue_usb_calculate_frame_crc(const tue_frame_t *frame)
{
    uint8_t header_buf[5];
    header_buf[0] = frame->seq;
    header_buf[1] = frame->cmd;
    header_buf[2] = frame->status;
    header_buf[3] = (uint8_t)(frame->len & 0xFFU);
    header_buf[4] = (uint8_t)((frame->len >> 8) & 0xFFU);

    uint16_t crc = tue_usb_crc16(header_buf, 5U, 0xFFFFU);
    if (frame->len > 0) {
        crc = tue_usb_crc16(frame->payload, frame->len, crc);
    }
    return crc;
}

static void tue_usb_process_frame(const tue_frame_t *req)
{
    /* Find command handler in registered table */
    const cmd_entry_t *entry = NULL;
    for (uint8_t i = 0; i < s_cmd_count; i++) {
        if (s_cmd_table[i].cmd_id == req->cmd) {
            entry = &s_cmd_table[i];
            break;
        }
    }

    if (entry == NULL) {
        tue_usb_send_response(req->seq, req->cmd, TUE_STATUS_ERR_UNKNOWN_CMD, NULL, 0);
        return;
    }

    if (req->len < entry->min_req_len || req->len > entry->max_req_len) {
        tue_usb_send_response(req->seq, req->cmd, TUE_STATUS_ERR_INVALID_LEN, NULL, 0);
        return;
    }

    uint16_t resp_len = 0;
    tue_status_t result = entry->handler(
        req->payload,
        req->len,
        s_resp_payload_buffer,
        &resp_len,
        TUE_USB_MAX_PAYLOAD
    );

    tue_usb_send_response(req->seq, req->cmd, result, s_resp_payload_buffer, resp_len);
}

static void tue_usb_task_entry(void *argument)
{
    (void)argument;
    uint8_t rx_chunk[64];

    for (;;) {
        size_t bytes_read = xStreamBufferReceive(
            s_rx_stream_buffer,
            rx_chunk,
            sizeof(rx_chunk),
            portMAX_DELAY
        );

        for (size_t i = 0; i < bytes_read; i++) {
            tue_usb_feed_byte(rx_chunk[i]);
        }
    }
}

/* Built-in Command Handlers -------------------------------------------------*/

static tue_status_t handle_sys_ping(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len > max_resp) {
        return TUE_STATUS_ERR_INVALID_LEN;
    }
    if (req_len > 0 && req != NULL) {
        memcpy(resp, req, req_len);
    }
    *resp_len = req_len;
    return TUE_STATUS_OK;
}

static tue_status_t handle_sys_get_info(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    (void)req;
    (void)req_len;

    if (max_resp < 19) {
        return TUE_STATUS_ERR_EXEC;
    }

    /* Board signature "MM01" */
    resp[0] = 'M';
    resp[1] = 'M';
    resp[2] = '0';
    resp[3] = '1';

    /* Firmware version 1.0.0 */
    resp[4] = 1U; /* Major */
    resp[5] = 0U; /* Minor */
    resp[6] = 0U; /* Patch */

    /* STM32 Unique 96-bit ID (12 bytes) */
    const uint8_t *uid = (const uint8_t *)UID_BASE;
    memcpy(&resp[7], uid, 12);

    *resp_len = 19;
    return TUE_STATUS_OK;
}

static tue_status_t handle_sys_led_ctrl(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 2 || max_resp < 2) {
        return TUE_STATUS_ERR_INVALID_LEN;
    }

    uint8_t led_index = req[0];
    uint8_t mode      = req[1];

    /* This board features one MCU status LED: LED_MCU on PE5 */
    if (led_index != 0) {
        return TUE_STATUS_ERR_INVALID_PARAM;
    }

    switch (mode) {
        case TUE_LED_MODE_OFF:
            HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, GPIO_PIN_RESET);
            break;

        case TUE_LED_MODE_ON:
            HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, GPIO_PIN_SET);
            break;

        case TUE_LED_MODE_TOGGLE:
            HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
            break;

        case TUE_LED_MODE_QUERY:
            /* Query only, do not alter pin */
            break;

        default:
            return TUE_STATUS_ERR_INVALID_PARAM;
    }

    /* Read back current pin state */
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(LED_MCU_GPIO_Port, LED_MCU_Pin);

    resp[0] = led_index;
    resp[1] = (pin_state == GPIO_PIN_SET) ? 1U : 0U;
    *resp_len = 2;

    return TUE_STATUS_OK;
}

/* Motor Subsystem Implementations -------------------------------------------*/

static inline uint8_t get_dc_fault(uint8_t motor_id)
{
    GPIO_PinState st = GPIO_PIN_SET;
    switch (motor_id) {
        case 1: st = HAL_GPIO_ReadPin(DC_F1_GPIO_Port, DC_F1_Pin); break;
        case 2: st = HAL_GPIO_ReadPin(DC_F2_GPIO_Port, DC_F2_Pin); break;
        case 3: st = HAL_GPIO_ReadPin(DC_F3_GPIO_Port, DC_F3_Pin); break;
        case 4: st = HAL_GPIO_ReadPin(DC_F4_GPIO_Port, DC_F4_Pin); break;
        default: break;
    }
    return (st == GPIO_PIN_RESET) ? 1U : 0U;
}

void tue_motor_init(void)
{
    /* Initialize STS3215 serial bus servo driver on USART1 (PA9) */
    tue_sts3215_init(&s_servo_bus, &huart1);

    /* Start PWM output on TIM4 (DC Motors 1 & 2) and TIM3 (DC Motors 3 & 4) */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* Disable stepper motor driver by default (Active-Low STEP_ENABLE) */
    HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_SET);
}

tue_status_t tue_motor_dc_set_duty(uint8_t motor_id, uint8_t direction, uint16_t duty)
{
    if (motor_id < 1 || motor_id > 4 || direction > 3 || duty > 10000) {
        return TUE_STATUS_ERR_INVALID_PARAM;
    }

    TIM_HandleTypeDef *htim;
    uint32_t ch_a, ch_b;

    switch (motor_id) {
        case 1: htim = &htim4; ch_a = TIM_CHANNEL_1; ch_b = TIM_CHANNEL_2; break;
        case 2: htim = &htim4; ch_a = TIM_CHANNEL_3; ch_b = TIM_CHANNEL_4; break;
        case 3: htim = &htim3; ch_a = TIM_CHANNEL_1; ch_b = TIM_CHANNEL_2; break;
        case 4: htim = &htim3; ch_a = TIM_CHANNEL_3; ch_b = TIM_CHANNEL_4; break;
        default: return TUE_STATUS_ERR_INVALID_PARAM;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t pulse = ((uint32_t)duty * arr) / 10000U;
    if (pulse > arr) pulse = arr;

    uint32_t val_a = 0, val_b = 0;
    switch (direction) {
        case TUE_DC_DIR_COAST:
            val_a = 0; val_b = 0;
            break;
        case TUE_DC_DIR_CW:
            val_a = pulse; val_b = 0;
            break;
        case TUE_DC_DIR_CCW:
            val_a = 0; val_b = pulse;
            break;
        case TUE_DC_DIR_BRAKE:
            val_a = arr; val_b = arr;
            break;
    }

    __HAL_TIM_SET_COMPARE(htim, ch_a, val_a);
    __HAL_TIM_SET_COMPARE(htim, ch_b, val_b);

    s_dc_motors[motor_id - 1].direction = direction;
    s_dc_motors[motor_id - 1].duty = duty;

    return TUE_STATUS_OK;
}

tue_status_t tue_motor_dc_stop(uint8_t motor_id, uint8_t brake_mode)
{
    uint8_t dir = (brake_mode == TUE_DC_BRAKE_ACTIVE) ? TUE_DC_DIR_BRAKE : TUE_DC_DIR_COAST;
    if (motor_id == 0) {
        for (uint8_t m = 1; m <= 4; m++) {
            tue_motor_dc_set_duty(m, dir, 0);
        }
        return TUE_STATUS_OK;
    }
    return tue_motor_dc_set_duty(motor_id, dir, 0);
}

tue_status_t tue_motor_dc_get_status(uint8_t motor_id, uint8_t *direction, uint16_t *duty, uint8_t *fault)
{
    if (motor_id < 1 || motor_id > 4) {
        return TUE_STATUS_ERR_INVALID_PARAM;
    }
    if (direction) *direction = s_dc_motors[motor_id - 1].direction;
    if (duty)      *duty      = s_dc_motors[motor_id - 1].duty;
    if (fault)     *fault     = get_dc_fault(motor_id);

    return TUE_STATUS_OK;
}

tue_status_t tue_motor_dc_set_pid(uint8_t motor_id, float kp, float ki, float kd)
{
    if (motor_id < 1 || motor_id > 4) {
        return TUE_STATUS_ERR_INVALID_PARAM;
    }
    s_dc_motors[motor_id - 1].kp = kp;
    s_dc_motors[motor_id - 1].ki = ki;
    s_dc_motors[motor_id - 1].kd = kd;
    return TUE_STATUS_OK;
}

tue_status_t tue_motor_step_enable(uint8_t stepper_id, bool enable)
{
    if (stepper_id > 4) {
        return TUE_STATUS_ERR_INVALID_PARAM;
    }

    HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if (stepper_id == 0) {
        for (int i = 0; i < 4; i++) {
            s_steppers[i].enabled = enable;
        }
    } else {
        s_steppers[stepper_id - 1].enabled = enable;
    }

    return TUE_STATUS_OK;
}

tue_status_t tue_motor_step_move(
    uint8_t stepper_id,
    bool is_relative,
    int32_t target_steps,
    uint32_t speed_hz,
    uint32_t accel)
{
    if (stepper_id < 1 || stepper_id > 4) {
        return TUE_STATUS_ERR_INVALID_PARAM;
    }

    uint8_t idx = stepper_id - 1;
    int32_t final_target = is_relative ? (s_steppers[idx].current_steps + target_steps) : target_steps;
    int32_t delta = final_target - s_steppers[idx].current_steps;

    GPIO_PinState dir_state = (delta >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    switch (stepper_id) {
        case 1: HAL_GPIO_WritePin(STEP_DIR1_GPIO_Port, STEP_DIR1_Pin, dir_state); break;
        case 2: HAL_GPIO_WritePin(STEP_DIR2_GPIO_Port, STEP_DIR2_Pin, dir_state); break;
        case 3: HAL_GPIO_WritePin(STEP_DIR3_GPIO_Port, STEP_DIR3_Pin, dir_state); break;
        case 4: HAL_GPIO_WritePin(STEP_DIR4_GPIO_Port, STEP_DIR4_Pin, dir_state); break;
    }

    s_steppers[idx].target_steps  = final_target;
    s_steppers[idx].speed_hz      = speed_hz;
    s_steppers[idx].accel         = accel;
    s_steppers[idx].is_moving     = (delta != 0);
    s_steppers[idx].current_steps = final_target;

    return TUE_STATUS_OK;
}

tue_status_t tue_motor_step_stop(uint8_t stepper_id, bool emergency_stop)
{
    (void)emergency_stop;
    if (stepper_id > 4) return TUE_STATUS_ERR_INVALID_PARAM;

    if (stepper_id == 0) {
        for (int i = 0; i < 4; i++) {
            s_steppers[i].is_moving = false;
        }
    } else {
        s_steppers[stepper_id - 1].is_moving = false;
    }
    return TUE_STATUS_OK;
}

tue_status_t tue_motor_step_get_pos(uint8_t stepper_id, int32_t *current_steps, bool *is_moving)
{
    if (stepper_id < 1 || stepper_id > 4) return TUE_STATUS_ERR_INVALID_PARAM;
    uint8_t idx = stepper_id - 1;
    if (current_steps) *current_steps = s_steppers[idx].current_steps;
    if (is_moving)     *is_moving     = s_steppers[idx].is_moving;
    return TUE_STATUS_OK;
}

tue_status_t tue_motor_step_set_homing(uint8_t stepper_id, uint8_t direction)
{
    (void)direction;
    if (stepper_id < 1 || stepper_id > 4) return TUE_STATUS_ERR_INVALID_PARAM;
    s_steppers[stepper_id - 1].current_steps = 0;
    s_steppers[stepper_id - 1].target_steps  = 0;
    s_steppers[stepper_id - 1].is_moving     = false;
    return TUE_STATUS_OK;
}

tue_status_t tue_motor_servo_ping(uint8_t servo_id)
{
    tue_sts3215_status_t st = tue_sts3215_ping(&s_servo_bus, servo_id);
    return (st == TUE_STS3215_OK) ? TUE_STATUS_OK : TUE_STATUS_ERR_EXEC;
}

tue_status_t tue_motor_servo_set_pos(uint8_t servo_id, int16_t position, int16_t speed, uint8_t accel)
{
    tue_sts3215_status_t st = tue_sts3215_set_goal_position_ex(&s_servo_bus, servo_id, position, speed, accel);
    return (st == TUE_STS3215_OK) ? TUE_STATUS_OK : TUE_STATUS_ERR_EXEC;
}

tue_status_t tue_motor_servo_torque_enable(uint8_t servo_id, uint8_t enable)
{
    tue_sts3215_status_t st;
    if (enable == 128U) {
        st = tue_sts3215_calibrate_position_to_2048(&s_servo_bus, servo_id);
    } else {
        st = tue_sts3215_enable_torque(&s_servo_bus, servo_id, enable != 0U);
    }
    return (st == TUE_STS3215_OK) ? TUE_STATUS_OK : TUE_STATUS_ERR_EXEC;
}

/* DC Motor Protocol Command Handlers ----------------------------------------*/

static tue_status_t handle_dc_set_duty(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 4 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t motor_id  = req[0];
    uint8_t direction = req[1];
    uint16_t duty     = (uint16_t)((uint16_t)req[2] | ((uint16_t)req[3] << 8));

    tue_status_t st = tue_motor_dc_set_duty(motor_id, direction, duty);

    resp[0] = motor_id;
    resp[1] = (st == TUE_STATUS_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return st;
}

static tue_status_t handle_dc_stop(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 2 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t motor_id   = req[0];
    uint8_t brake_mode = req[1];

    tue_status_t st = tue_motor_dc_stop(motor_id, brake_mode);

    resp[0] = motor_id;
    resp[1] = (st == TUE_STATUS_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return st;
}

static tue_status_t handle_dc_get_status(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 1 || max_resp < 8) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t motor_id = req[0];
    if (motor_id < 1 || motor_id > 4) return TUE_STATUS_ERR_INVALID_PARAM;

    uint8_t dir = 0, fault = 0;
    uint16_t duty = 0;
    tue_motor_dc_get_status(motor_id, &dir, &duty, &fault);

    resp[0] = motor_id;
    resp[1] = dir;
    resp[2] = (uint8_t)(duty & 0xFFU);
    resp[3] = (uint8_t)((duty >> 8) & 0xFFU);
    resp[4] = fault;
    resp[5] = 0U; /* Current LSB */
    resp[6] = 0U; /* Current MSB */
    resp[7] = 0U; /* Reserved */

    *resp_len = 8;
    return TUE_STATUS_OK;
}

static tue_status_t handle_dc_set_pid(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 13 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t motor_id = req[0];
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    memcpy(&kp, &req[1], 4);
    memcpy(&ki, &req[5], 4);
    memcpy(&kd, &req[9], 4);

    tue_status_t st = tue_motor_dc_set_pid(motor_id, kp, ki, kd);

    resp[0] = motor_id;
    resp[1] = (st == TUE_STATUS_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return st;
}

/* Stepper Motor Protocol Command Handlers -----------------------------------*/

static tue_status_t handle_step_enable(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 2 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t stepper_id = req[0];
    uint8_t enable     = req[1];

    tue_status_t st = tue_motor_step_enable(stepper_id, enable != 0U);

    resp[0] = stepper_id;
    resp[1] = (st == TUE_STATUS_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return st;
}

static tue_status_t handle_step_move(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 14 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t stepper_id   = req[0];
    uint8_t is_relative  = req[1];
    int32_t target_steps = 0;
    uint32_t speed_hz    = 0;
    uint32_t accel       = 0;

    memcpy(&target_steps, &req[2], 4);
    memcpy(&speed_hz, &req[6], 4);
    memcpy(&accel, &req[10], 4);

    tue_status_t st = tue_motor_step_move(stepper_id, is_relative != 0U, target_steps, speed_hz, accel);

    resp[0] = stepper_id;
    resp[1] = (st == TUE_STATUS_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return st;
}

static tue_status_t handle_step_stop(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 2 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t stepper_id = req[0];
    uint8_t hard_stop  = req[1];

    tue_status_t st = tue_motor_step_stop(stepper_id, hard_stop != 0U);

    resp[0] = stepper_id;
    resp[1] = (st == TUE_STATUS_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return st;
}

static tue_status_t handle_step_get_pos(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 1 || max_resp < 9) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t stepper_id = req[0];
    int32_t current_steps = 0;
    bool is_moving = false;

    tue_status_t st = tue_motor_step_get_pos(stepper_id, &current_steps, &is_moving);
    if (st != TUE_STATUS_OK) return st;

    resp[0] = stepper_id;
    memcpy(&resp[1], &current_steps, 4);
    resp[5] = is_moving ? 1U : 0U;
    resp[6] = 0U; /* Fault flags */
    resp[7] = 0U; /* Reserved LSB */
    resp[8] = 0U; /* Reserved MSB */

    *resp_len = 9;
    return TUE_STATUS_OK;
}

static tue_status_t handle_step_set_homing(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 2 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t stepper_id = req[0];
    uint8_t direction  = req[1];

    tue_status_t st = tue_motor_step_set_homing(stepper_id, direction);

    resp[0] = stepper_id;
    resp[1] = (st == TUE_STATUS_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return st;
}

/* Servo STS3215 Protocol Command Handlers -----------------------------------*/

static tue_status_t handle_servo_ping(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 1 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t servo_id = req[0];

    tue_sts3215_status_t st = tue_sts3215_ping(&s_servo_bus, servo_id);

    resp[0] = servo_id;
    resp[1] = (st == TUE_STS3215_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return (st == TUE_STS3215_OK) ? TUE_STATUS_OK : TUE_STATUS_ERR_EXEC;
}

static tue_status_t handle_servo_read_reg(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 3) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t servo_id = req[0];
    uint8_t reg_addr = req[1];
    uint8_t read_len = req[2];

    if ((uint16_t)(2U + read_len) > max_resp) return TUE_STATUS_ERR_INVALID_PARAM;

    resp[0] = servo_id;
    resp[1] = reg_addr;

    tue_sts3215_status_t st = tue_sts3215_read_raw(&s_servo_bus, servo_id, reg_addr, &resp[2], read_len);
    if (st != TUE_STS3215_OK) {
        *resp_len = 2;
        return TUE_STATUS_ERR_EXEC;
    }

    *resp_len = (uint16_t)(2U + read_len);
    return TUE_STATUS_OK;
}

static tue_status_t handle_servo_write_reg(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 3 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t servo_id = req[0];
    uint8_t reg_addr = req[1];
    uint8_t data_len = (uint8_t)(req_len - 2U);

    tue_sts3215_status_t st = tue_sts3215_write_raw(&s_servo_bus, servo_id, reg_addr, &req[2], data_len);

    resp[0] = servo_id;
    resp[1] = (st == TUE_STS3215_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return (st == TUE_STS3215_OK) ? TUE_STATUS_OK : TUE_STATUS_ERR_EXEC;
}

static tue_status_t handle_servo_set_pos(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 7 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t servo_id = req[0];
    int16_t pos   = (int16_t)((uint16_t)req[1] | ((uint16_t)req[2] << 8));
    int16_t speed = (int16_t)((uint16_t)req[3] | ((uint16_t)req[4] << 8));
    uint8_t accel = req[5];

    tue_sts3215_status_t st = tue_sts3215_set_goal_position_ex(&s_servo_bus, servo_id, pos, speed, accel);

    resp[0] = servo_id;
    resp[1] = (st == TUE_STS3215_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return (st == TUE_STS3215_OK) ? TUE_STATUS_OK : TUE_STATUS_ERR_EXEC;
}

static tue_status_t handle_servo_get_feedback(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 1 || max_resp < 13) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t servo_id = req[0];

    tue_sts3215_feedback_t fb;
    tue_sts3215_status_t st = tue_sts3215_get_feedback(&s_servo_bus, servo_id, &fb);
    if (st != TUE_STS3215_OK && st != TUE_STS3215_ERR_STATUS) {
        resp[0] = servo_id;
        resp[1] = (uint8_t)st;
        *resp_len = 2;
        return TUE_STATUS_ERR_EXEC;
    }

    /* Serialize explicitly Little-Endian according to protocol_payloads.md Table 3.4 */
    resp[0]  = servo_id;
    resp[1]  = (uint8_t)((uint16_t)fb.position & 0xFFU);
    resp[2]  = (uint8_t)(((uint16_t)fb.position >> 8) & 0xFFU);
    resp[3]  = (uint8_t)((uint16_t)fb.speed & 0xFFU);
    resp[4]  = (uint8_t)(((uint16_t)fb.speed >> 8) & 0xFFU);
    resp[5]  = (uint8_t)((uint16_t)fb.load & 0xFFU);
    resp[6]  = (uint8_t)(((uint16_t)fb.load >> 8) & 0xFFU);
    resp[7]  = fb.voltage_raw;
    resp[8]  = (uint8_t)fb.temperature_degc;
    resp[9]  = fb.status_mask;
    resp[10] = (uint8_t)((uint16_t)fb.current_raw & 0xFFU);
    resp[11] = (uint8_t)(((uint16_t)fb.current_raw >> 8) & 0xFFU);
    resp[12] = fb.is_moving ? 1U : 0U;

    *resp_len = 13;
    return TUE_STATUS_OK;
}

static tue_status_t handle_servo_torque_enable(
    const uint8_t *req,
    uint16_t req_len,
    uint8_t *resp,
    uint16_t *resp_len,
    uint16_t max_resp)
{
    if (req_len < 2 || max_resp < 2) return TUE_STATUS_ERR_INVALID_LEN;
    uint8_t servo_id = req[0];
    uint8_t enable   = req[1];

    tue_sts3215_status_t st;
    if (enable == 128U) {
        st = tue_sts3215_calibrate_position_to_2048(&s_servo_bus, servo_id);
    } else {
        st = tue_sts3215_enable_torque(&s_servo_bus, servo_id, enable != 0U);
    }

    resp[0] = servo_id;
    resp[1] = (st == TUE_STS3215_OK) ? 0x00U : 0x01U;
    *resp_len = 2;
    return (st == TUE_STS3215_OK) ? TUE_STATUS_OK : TUE_STATUS_ERR_EXEC;
}

