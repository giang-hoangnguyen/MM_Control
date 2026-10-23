/**
  ******************************************************************************
  * @file           : tue_sts3215.h
  * @brief          : High-performance Feetech STS3215 serial bus servo driver
  *                   for STM32F407 (Half-duplex UART + DMA + Interrupt)
  ******************************************************************************
  * @attention
  *
  * Hardware Interface:
  * - SERVO_TX (PA9) configured in Half-Duplex Single-Wire mode (USART1).
  * - DMA2 Stream 7 Channel 4 (TX, Normal Mode).
  * - DMA2 Stream 2 Channel 4 (RX, Normal Mode).
  * - Half-Duplex transceiver switching between TX and RX is handled
  *   automatically with zero-latency turnaround.
  * - Fully thread-safe under CMSIS-RTOS2 / FreeRTOS, while also supporting
  *   bare-metal polling before scheduler initialization.
  *
  * Covers all registers and functions defined in docs/MEMORY_TABLE.md.
  *
  ******************************************************************************
  */

#ifndef TUE_STS3215_H_
#define TUE_STS3215_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"
#include "usart.h"
#include "cmsis_os2.h"

/* Protocol Constants --------------------------------------------------------*/
#define STS3215_HEADER_BYTE             0xFFU
#define STS3215_BROADCAST_ID            0xFEU
#define STS3215_DEFAULT_ID              0x01U

#define STS3215_BUFFER_SIZE             128U
#define STS3215_DEFAULT_TIMEOUT_MS      20U

/* Instructions --------------------------------------------------------------*/
#define STS3215_INST_PING               0x01U
#define STS3215_INST_READ               0x02U
#define STS3215_INST_WRITE              0x03U
#define STS3215_INST_REG_WRITE          0x04U
#define STS3215_INST_ACTION             0x05U
#define STS3215_INST_RESET              0x06U
#define STS3215_INST_SYNC_READ          0x82U
#define STS3215_INST_SYNC_WRITE         0x83U

/* Memory Table Registers (docs/MEMORY_TABLE.md) -----------------------------*/

/* --- EEPROM Area (Read-Only) --- */
#define STS3215_REG_FW_MAJOR            0x00U /* 1 Byte, Firmware major version */
#define STS3215_REG_FW_MINOR            0x01U /* 1 Byte, Firmware minor version */
#define STS3215_REG_END                 0x02U /* 1 Byte, END marker */
#define STS3215_REG_SERVO_MAIN_VER      0x03U /* 1 Byte, Servo main version */
#define STS3215_REG_SERVO_VER           0x04U /* 1 Byte, Servo version */

/* --- EEPROM Area (Read/Write, Requires EEPROM Unlock 0x37) --- */
#define STS3215_REG_ID                  0x05U /* 1 Byte, Servo ID (0-253, 0xFE broadcast) */
#define STS3215_REG_BAUD_RATE           0x06U /* 1 Byte, Baud rate index (0:1M..7:38.4k) */
#define STS3215_REG_RETURN_DELAY        0x07U /* 1 Byte, Response return delay (0-254 * 2us) */
#define STS3215_REG_RESPONSE_LEVEL      0x08U /* 1 Byte, Phase 2 status level (0:ping/read only, 1:all) */
#define STS3215_REG_MIN_ANGLE_LIMIT     0x09U /* 2 Bytes, Min travel limit (0-4094 steps) */
#define STS3215_REG_MAX_ANGLE_LIMIT     0x0BU /* 2 Bytes, Max travel limit (1-4095 steps) */
#define STS3215_REG_MAX_TEMP_LIMIT      0x0DU /* 1 Byte, Max temperature limit (°C) */
#define STS3215_REG_MAX_INPUT_VOLTAGE   0x0EU /* 1 Byte, Max input voltage (0.1V) */
#define STS3215_REG_MIN_INPUT_VOLTAGE   0x0FU /* 1 Byte, Min input voltage (0.1V) */
#define STS3215_REG_MAX_TORQUE          0x10U /* 2 Bytes, Max output torque (0.1%) */
#define STS3215_REG_PHASE               0x12U /* 1 Byte, Phase parameter */
#define STS3215_REG_UNLOAD_CONDITION    0x13U /* 1 Byte, Protection uninstallation flags */
#define STS3215_REG_LED_ALARM_CONDITION 0x14U /* 1 Byte, LED flashing alarm flags */
#define STS3215_REG_POS_P               0x15U /* 1 Byte, Position loop proportional P */
#define STS3215_REG_POS_D               0x16U /* 1 Byte, Position loop derivative D */
#define STS3215_REG_POS_I               0x17U /* 1 Byte, Position loop integral I */
#define STS3215_REG_MIN_START_FORCE     0x18U /* 1 Byte, Minimum starting torque (0.1%) */
#define STS3215_REG_POINTS_LIMIT        0x19U /* 1 Byte, Score/points limit (*4) */
#define STS3215_REG_CW_DEADBAND         0x1AU /* 1 Byte, Clockwise insensitive area (0-32 steps) */
#define STS3215_REG_CCW_DEADBAND        0x1BU /* 1 Byte, CCW insensitive area (0-32 steps) */
#define STS3215_REG_PROTECTIVE_CURRENT  0x1CU /* 2 Bytes, Current threshold (6.5mA) */
#define STS3215_REG_ANGULAR_RESOLUTION  0x1EU /* 1 Byte, Sensor magnification (1-3) */
#define STS3215_REG_POSITION_CORRECTION 0x1FU /* 2 Bytes, Angle offset (-2047..2047, bit11=dir) */
#define STS3215_REG_OPERATION_MODE      0x21U /* 1 Byte, Mode (0:Pos, 1:Speed, 2:PWM, 3:Step) */
#define STS3215_REG_PROTECTIVE_TORQUE   0x22U /* 1 Byte, Torque after overload (1.0%) */
#define STS3215_REG_PROTECTION_TIME     0x23U /* 1 Byte, Overload duration before reset (10ms) */
#define STS3215_REG_OVERLOAD_TORQUE     0x24U /* 1 Byte, Overload torque threshold (1.0%) */
#define STS3215_REG_SPEED_P             0x25U /* 1 Byte, Speed loop proportional P */
#define STS3215_REG_OVERCURRENT_TIME    0x26U /* 1 Byte, Overcurrent duration (10ms) */
#define STS3215_REG_SPEED_I             0x27U /* 1 Byte, Speed loop integral I (1/10) */

/* --- SRAM Area (Read/Write Control & Volatile Settings) --- */
#define STS3215_REG_TORQUE_ENABLE       0x28U /* 1 Byte, 0:Off, 1:On, 128:Correct pos to 2048 */
#define STS3215_REG_ACCELERATION        0x29U /* 1 Byte, Accel/decel rate (100 steps/s²) */
#define STS3215_REG_TARGET_POSITION     0x2AU /* 2 Bytes, Absolute goal position (steps) */
#define STS3215_REG_RUNTIME             0x2CU /* 2 Bytes, PWM open-loop speed (0-1000, bit10=dir) */
#define STS3215_REG_RUNNING_SPEED       0x2EU /* 2 Bytes, Goal velocity (steps/s) */
#define STS3215_REG_TORQUE_LIMIT        0x30U /* 2 Bytes, Dynamic torque limit (0.1%) */
#define STS3215_REG_LOCK_MARK           0x37U /* 1 Byte, EEPROM write lock (0:unlock, 1:lock) */

/* --- SRAM Area (Read-Only Telemetry / Feedback) --- */
#define STS3215_REG_CURRENT_POSITION    0x38U /* 2 Bytes, Current angle step feedback */
#define STS3215_REG_CURRENT_SPEED       0x3AU /* 2 Bytes, Current angular velocity (steps/s) */
#define STS3215_REG_CURRENT_LOAD        0x3CU /* 2 Bytes, Current drive duty (0.1%) */
#define STS3215_REG_CURRENT_VOLTAGE     0x3EU /* 1 Byte, Operating voltage (0.1V) */
#define STS3215_REG_CURRENT_TEMP        0x3FU /* 1 Byte, Operating temperature (°C) */
#define STS3215_REG_ASYNC_WRITE_FLAG    0x40U /* 1 Byte, Reg write pending flag */
#define STS3215_REG_SERVO_STATUS        0x41U /* 1 Byte, Hardware error status bitmask */
#define STS3215_REG_MOVING_FLAG         0x42U /* 1 Byte, 1:in motion, 0:stationary */
#define STS3215_REG_CURRENT_CURRENT     0x45U /* 2 Bytes, Motor current (6.5mA) */

/* Hardware Status Error Bitmasks (Register 0x41) ----------------------------*/
#define STS3215_STATUS_VOLTAGE_ERR      (1U << 0) /* Bit 0: Voltage out of range */
#define STS3215_STATUS_SENSOR_ERR       (1U << 1) /* Bit 1: Magnetic sensor error */
#define STS3215_STATUS_TEMP_ERR         (1U << 2) /* Bit 2: Overheat / temperature warning */
#define STS3215_STATUS_CURRENT_ERR      (1U << 3) /* Bit 3: Overcurrent error */
#define STS3215_STATUS_ANGLE_ERR        (1U << 4) /* Bit 4: Angle command exceeds limits */
#define STS3215_STATUS_OVERLOAD_ERR     (1U << 5) /* Bit 5: Overload protection triggered */
#define STS3215_STATUS_CHECKSUM_ERR     (1U << 6) /* Bit 6: Protocol checksum error */

/* Unload / Alarm Condition Masks (Registers 0x13, 0x14) ---------------------*/
#define STS3215_PROTECT_VOLTAGE         (1U << 0)
#define STS3215_PROTECT_SENSOR          (1U << 1)
#define STS3215_PROTECT_TEMP            (1U << 2)
#define STS3215_PROTECT_CURRENT         (1U << 3)
#define STS3215_PROTECT_ANGLE_LIMIT     (1U << 4)
#define STS3215_PROTECT_OVERLOAD        (1U << 5)

/* Operating Modes (Register 0x21) -------------------------------------------*/
typedef enum {
    STS3215_MODE_POSITION = 0, /* Position servo mode (0-4095 absolute) */
    STS3215_MODE_SPEED    = 1, /* Constant speed closed-loop rotation */
    STS3215_MODE_PWM      = 2, /* PWM open-loop speed control */
    STS3215_MODE_STEPPER  = 3  /* Multi-turn continuous stepper servo mode */
} tue_sts3215_mode_t;

/* Baud Rate Indices (Register 0x06) -----------------------------------------*/
typedef enum {
    STS3215_BAUD_1000000 = 0,  /* 1,000,000 baud (Default) */
    STS3215_BAUD_500000  = 1,  /* 500,000 baud */
    STS3215_BAUD_250000  = 2,  /* 250,000 baud */
    STS3215_BAUD_128000  = 3,  /* 128,000 baud */
    STS3215_BAUD_115200  = 4,  /* 115,200 baud */
    STS3215_BAUD_76800   = 5,  /* 76,800 baud */
    STS3215_BAUD_57600   = 6,  /* 57,600 baud */
    STS3215_BAUD_38400   = 7   /* 38,400 baud */
} tue_sts3215_baud_t;

/* Driver Return Status Codes ------------------------------------------------*/
typedef enum {
    TUE_STS3215_OK              = 0,  /* Success */
    TUE_STS3215_ERR_TIMEOUT     = 1,  /* No response within timeout */
    TUE_STS3215_ERR_CHECKSUM    = 2,  /* Response checksum mismatch */
    TUE_STS3215_ERR_HEADER      = 3,  /* Response frame header (0xFF 0xFF) invalid */
    TUE_STS3215_ERR_LEN         = 4,  /* Frame length incorrect */
    TUE_STS3215_ERR_ID          = 5,  /* Response ID did not match request */
    TUE_STS3215_ERR_STATUS      = 6,  /* Servo reported hardware error bit in status byte */
    TUE_STS3215_ERR_PARAM       = 7,  /* Out of range or NULL parameter */
    TUE_STS3215_ERR_HAL         = 8,  /* Underlying STM32 HAL / DMA error */
    TUE_STS3215_ERR_BUSY        = 9   /* Mutex or UART transfer busy */
} tue_sts3215_status_t;

/* Comprehensive Telemetry Feedback Structure --------------------------------*/
typedef struct {
    uint8_t  id;                /* Responding servo ID */
    int16_t  position;          /* Current angular position (steps) */
    int16_t  speed;             /* Current angular velocity (steps/s) */
    int16_t  load;              /* Motor output drive duty (0.1%) */
    uint8_t  voltage_raw;       /* Raw voltage register value (0.1V) */
    float    voltage_v;         /* Supply voltage in Volts */
    int8_t   temperature_degc;  /* Internal motor temperature in °C */
    uint8_t  async_flag;        /* Reg write pending flag */
    uint8_t  status_mask;       /* Hardware status error flags */
    uint8_t  is_moving;         /* 1: in motion, 0: stationary */
    int16_t  current_raw;       /* Raw motor current (6.5mA units) */
    float    current_ma;        /* Motor current in mA */
} tue_sts3215_feedback_t;

/* Sync Write Entry for Position Control -------------------------------------*/
typedef struct {
    uint8_t  id;                /* Servo ID */
    int16_t  position;          /* Target position (steps) */
    int16_t  speed;             /* Goal speed (steps/s) */
    uint8_t  accel;             /* Acceleration rate (100 steps/s²) */
} tue_sts3215_sync_pos_t;

/* Sync Write Entry for Velocity Control -------------------------------------*/
typedef struct {
    uint8_t  id;                /* Servo ID */
    int16_t  speed;             /* Goal speed (steps/s) */
    uint8_t  accel;             /* Acceleration rate (100 steps/s²) */
} tue_sts3215_sync_speed_t;

/* Driver Handle Structure ---------------------------------------------------*/
typedef struct {
    UART_HandleTypeDef *huart;              /* Pointer to USART1 handle */
    uint8_t             tx_buf[STS3215_BUFFER_SIZE];
    uint8_t             rx_buf[STS3215_BUFFER_SIZE];
    volatile uint16_t   rx_len;             /* Number of received bytes */
    volatile uint8_t    tx_done;            /* TX completion flag */
    volatile uint8_t    rx_done;            /* RX completion flag */
    volatile uint8_t    awaiting_rx;        /* Receiver arming flag */
    uint32_t            timeout_ms;         /* Response timeout in milliseconds */
    uint32_t            last_hal_error;     /* Captured HAL UART error code */
    uint8_t             last_servo_status;  /* Captured servo hardware status byte */
    bool                use_rtos;           /* True if CMSIS-RTOS2 synchronization is used */
    osMutexId_t         mutex;              /* Bus transaction mutex */
    osSemaphoreId_t     tx_sem;             /* DMA TX completion semaphore */
    osSemaphoreId_t     rx_sem;             /* DMA RX completion semaphore */
} tue_sts3215_t;

/* Core Driver Lifecycle & Configuration -------------------------------------*/

/**
  * @brief  Initialize the STS3215 servo driver instance.
  * @param  dev    Pointer to tue_sts3215_t driver structure.
  * @param  huart  Pointer to initialized UART handle (e.g. &huart1).
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_init(tue_sts3215_t *dev, UART_HandleTypeDef *huart);

/**
  * @brief  Retrieve pointer to the global default driver instance.
  * @retval Pointer to default tue_sts3215_t instance.
  */
tue_sts3215_t* tue_sts3215_get_default(void);

/**
  * @brief  Configure response timeout in milliseconds.
  * @param  dev         Driver handle.
  * @param  timeout_ms  Timeout in milliseconds (default: 20 ms).
  */
void tue_sts3215_set_timeout(tue_sts3215_t *dev, uint32_t timeout_ms);

/**
  * @brief  Enable or disable RTOS synchronization primitives.
  * @param  dev       Driver handle.
  * @param  use_rtos  true to use RTOS mutex/semaphores, false for polling.
  */
void tue_sts3215_set_use_rtos(tue_sts3215_t *dev, bool use_rtos);

/* ISR Notification Hooks ----------------------------------------------------*/

/**
  * @brief  Hook to be called from HAL_UART_TxCpltCallback.
  *         Automatically arms the Half-Duplex receiver for DMA reception.
  */
void tue_sts3215_on_tx_cplt(UART_HandleTypeDef *huart);

/**
  * @brief  Hook to be called from HAL_UARTEx_RxEventCallback.
  */
void tue_sts3215_on_rx_event(UART_HandleTypeDef *huart, uint16_t size);

/**
  * @brief  Hook to be called from HAL_UART_RxCpltCallback.
  */
void tue_sts3215_on_rx_cplt(UART_HandleTypeDef *huart);

/**
  * @brief  Hook to be called from HAL_UART_ErrorCallback.
  */
void tue_sts3215_on_error(UART_HandleTypeDef *huart);

/* Low-Level Protocol Commands -----------------------------------------------*/

/**
  * @brief  Ping a servo on the bus (INST_PING, 0x01).
  * @param  dev  Driver handle.
  * @param  id   Servo ID (0-253).
  * @retval TUE_STS3215_OK if servo responded, error code otherwise.
  */
tue_sts3215_status_t tue_sts3215_ping(tue_sts3215_t *dev, uint8_t id);

/**
  * @brief  Read arbitrary register bytes from a servo (INST_READ, 0x02).
  * @param  dev       Driver handle.
  * @param  id        Servo ID (0-253).
  * @param  reg_addr  Starting memory address.
  * @param  data      Buffer to receive read bytes.
  * @param  len       Number of bytes to read.
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_read_raw(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t reg_addr,
    uint8_t *data,
    uint8_t len
);

/**
  * @brief  Write arbitrary register bytes to a servo with status ACK (INST_WRITE, 0x03).
  * @param  dev       Driver handle.
  * @param  id        Servo ID (0-253, or 0xFE broadcast).
  * @param  reg_addr  Starting memory address.
  * @param  data      Pointer to data bytes to write.
  * @param  len       Number of bytes to write.
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_write_raw(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t reg_addr,
    const uint8_t *data,
    uint8_t len
);

/**
  * @brief  Write arbitrary register bytes without waiting for an ACK response.
  * @param  dev       Driver handle.
  * @param  id        Servo ID (0-253, or 0xFE broadcast).
  * @param  reg_addr  Starting memory address.
  * @param  data      Pointer to data bytes to write.
  * @param  len       Number of bytes to write.
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_write_raw_no_ack(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t reg_addr,
    const uint8_t *data,
    uint8_t len
);

/**
  * @brief  Asynchronously buffer a write instruction into servo (INST_REG_WRITE, 0x04).
  *         Executed only upon subsequent ACTION command.
  */
tue_sts3215_status_t tue_sts3215_reg_write_raw(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t reg_addr,
    const uint8_t *data,
    uint8_t len
);

/**
  * @brief  Trigger execution of buffered REG_WRITE commands (INST_ACTION, 0x05).
  * @param  dev  Driver handle.
  * @param  id   Servo ID (usually STS3215_BROADCAST_ID = 0xFE).
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_action(tue_sts3215_t *dev, uint8_t id);

/**
  * @brief  Restore servo EEPROM and parameters to factory defaults (INST_RESET, 0x06).
  * @param  dev  Driver handle.
  * @param  id   Servo ID (0-253).
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_reset(tue_sts3215_t *dev, uint8_t id);

/**
  * @brief  Broadcast synchronized write to multiple servos (INST_SYNC_WRITE, 0x83).
  * @param  dev                Driver handle.
  * @param  start_addr         Starting register address.
  * @param  data_len           Number of parameter bytes per servo.
  * @param  param_data         Array containing [ID, Data0..DataN] for each servo.
  * @param  total_param_bytes  Length of param_data array ((data_len + 1) * count).
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_sync_write_raw(
    tue_sts3215_t *dev,
    uint8_t start_addr,
    uint8_t data_len,
    const uint8_t *param_data,
    uint16_t total_param_bytes
);

/* Primitive Read / Write Helpers --------------------------------------------*/
tue_sts3215_status_t tue_sts3215_read_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t *val);
tue_sts3215_status_t tue_sts3215_write_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t val);
tue_sts3215_status_t tue_sts3215_read_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t *val);
tue_sts3215_status_t tue_sts3215_write_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t val);
tue_sts3215_status_t tue_sts3215_read_i16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, int16_t *val);
tue_sts3215_status_t tue_sts3215_write_i16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, int16_t val);

/* EEPROM Safe Write Helpers (Handles Unlock 0x37 Automatically) -------------*/
tue_sts3215_status_t tue_sts3215_write_eeprom_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t val);
tue_sts3215_status_t tue_sts3215_write_eeprom_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t val);

/* High-Level Functions (docs/MEMORY_TABLE.md) -------------------------------*/

/* EEPROM Lock Management (0x37) */
tue_sts3215_status_t tue_sts3215_lock_eeprom(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_unlock_eeprom(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_get_eeprom_lock(tue_sts3215_t *dev, uint8_t id, uint8_t *is_locked);

/* Versions & Device Markers (0x00..0x04) */
tue_sts3215_status_t tue_sts3215_get_fw_version(tue_sts3215_t *dev, uint8_t id, uint8_t *major, uint8_t *minor);
tue_sts3215_status_t tue_sts3215_get_end_marker(tue_sts3215_t *dev, uint8_t id, uint8_t *end_val);
tue_sts3215_status_t tue_sts3215_get_servo_version(tue_sts3215_t *dev, uint8_t id, uint8_t *main_ver, uint8_t *sub_ver);

/* Bus ID & Baud Rate (0x05, 0x06) */
tue_sts3215_status_t tue_sts3215_set_id(tue_sts3215_t *dev, uint8_t old_id, uint8_t new_id);
tue_sts3215_status_t tue_sts3215_get_id(tue_sts3215_t *dev, uint8_t id, uint8_t *current_id);
tue_sts3215_status_t tue_sts3215_set_baud_rate(tue_sts3215_t *dev, uint8_t id, tue_sts3215_baud_t baud);
tue_sts3215_status_t tue_sts3215_get_baud_rate(tue_sts3215_t *dev, uint8_t id, tue_sts3215_baud_t *baud);

/* Return Delay & Response Level (0x07, 0x08) */
tue_sts3215_status_t tue_sts3215_set_return_delay(tue_sts3215_t *dev, uint8_t id, uint8_t delay_2us);
tue_sts3215_status_t tue_sts3215_get_return_delay(tue_sts3215_t *dev, uint8_t id, uint8_t *delay_2us);
tue_sts3215_status_t tue_sts3215_set_response_level(tue_sts3215_t *dev, uint8_t id, uint8_t level);
tue_sts3215_status_t tue_sts3215_get_response_level(tue_sts3215_t *dev, uint8_t id, uint8_t *level);

/* Motion Travel Angle Limits (0x09, 0x0B) */
tue_sts3215_status_t tue_sts3215_set_angle_limits(tue_sts3215_t *dev, uint8_t id, uint16_t min_angle, uint16_t max_angle);
tue_sts3215_status_t tue_sts3215_get_angle_limits(tue_sts3215_t *dev, uint8_t id, uint16_t *min_angle, uint16_t *max_angle);

/* Temperature & Voltage Limits (0x0D, 0x0E, 0x0F) */
tue_sts3215_status_t tue_sts3215_set_max_temperature_limit(tue_sts3215_t *dev, uint8_t id, uint8_t max_temp_c);
tue_sts3215_status_t tue_sts3215_get_max_temperature_limit(tue_sts3215_t *dev, uint8_t id, uint8_t *max_temp_c);
tue_sts3215_status_t tue_sts3215_set_voltage_limits(tue_sts3215_t *dev, uint8_t id, uint8_t min_0_1v, uint8_t max_0_1v);
tue_sts3215_status_t tue_sts3215_get_voltage_limits(tue_sts3215_t *dev, uint8_t id, uint8_t *min_0_1v, uint8_t *max_0_1v);

/* Max Torque & Phase (0x10, 0x12) */
tue_sts3215_status_t tue_sts3215_set_max_torque(tue_sts3215_t *dev, uint8_t id, uint16_t max_torque_0_1pct);
tue_sts3215_status_t tue_sts3215_get_max_torque(tue_sts3215_t *dev, uint8_t id, uint16_t *max_torque_0_1pct);
tue_sts3215_status_t tue_sts3215_set_phase(tue_sts3215_t *dev, uint8_t id, uint8_t phase);
tue_sts3215_status_t tue_sts3215_get_phase(tue_sts3215_t *dev, uint8_t id, uint8_t *phase);

/* Protection Unload & LED Alarm Conditions (0x13, 0x14) */
tue_sts3215_status_t tue_sts3215_set_unload_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t conditions_mask);
tue_sts3215_status_t tue_sts3215_get_unload_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t *conditions_mask);
tue_sts3215_status_t tue_sts3215_set_led_alarm_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t conditions_mask);
tue_sts3215_status_t tue_sts3215_get_led_alarm_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t *conditions_mask);

/* Position Loop PID Coefficients & Force (0x15, 0x16, 0x17, 0x18) */
tue_sts3215_status_t tue_sts3215_set_position_pid(tue_sts3215_t *dev, uint8_t id, uint8_t p, uint8_t d, uint8_t i);
tue_sts3215_status_t tue_sts3215_get_position_pid(tue_sts3215_t *dev, uint8_t id, uint8_t *p, uint8_t *d, uint8_t *i);
tue_sts3215_status_t tue_sts3215_set_min_starting_force(tue_sts3215_t *dev, uint8_t id, uint8_t force_0_1pct);
tue_sts3215_status_t tue_sts3215_get_min_starting_force(tue_sts3215_t *dev, uint8_t id, uint8_t *force_0_1pct);

/* Points Limit & Insensitive Deadband (0x19, 0x1A, 0x1B) */
tue_sts3215_status_t tue_sts3215_set_points_limit(tue_sts3215_t *dev, uint8_t id, uint8_t limit);
tue_sts3215_status_t tue_sts3215_get_points_limit(tue_sts3215_t *dev, uint8_t id, uint8_t *limit);
tue_sts3215_status_t tue_sts3215_set_deadband(tue_sts3215_t *dev, uint8_t id, uint8_t cw_deadband, uint8_t ccw_deadband);
tue_sts3215_status_t tue_sts3215_get_deadband(tue_sts3215_t *dev, uint8_t id, uint8_t *cw_deadband, uint8_t *ccw_deadband);

/* Protective Current, Resolution & Position Correction (0x1C, 0x1E, 0x1F) */
tue_sts3215_status_t tue_sts3215_set_protective_current(tue_sts3215_t *dev, uint8_t id, uint16_t current_6_5ma);
tue_sts3215_status_t tue_sts3215_get_protective_current(tue_sts3215_t *dev, uint8_t id, uint16_t *current_6_5ma);
tue_sts3215_status_t tue_sts3215_set_angular_resolution(tue_sts3215_t *dev, uint8_t id, uint8_t resolution);
tue_sts3215_status_t tue_sts3215_get_angular_resolution(tue_sts3215_t *dev, uint8_t id, uint8_t *resolution);
tue_sts3215_status_t tue_sts3215_set_position_correction(tue_sts3215_t *dev, uint8_t id, int16_t offset_steps);
tue_sts3215_status_t tue_sts3215_get_position_correction(tue_sts3215_t *dev, uint8_t id, int16_t *offset_steps);

/* Operating Mode (0x21) */
tue_sts3215_status_t tue_sts3215_set_operation_mode(tue_sts3215_t *dev, uint8_t id, tue_sts3215_mode_t mode);
tue_sts3215_status_t tue_sts3215_get_operation_mode(tue_sts3215_t *dev, uint8_t id, tue_sts3215_mode_t *mode);

/* Overload & Protective Torque / Time (0x22, 0x23, 0x24) */
tue_sts3215_status_t tue_sts3215_set_overload_protection(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t protective_torque_pct,
    uint8_t protection_time_10ms,
    uint8_t overload_torque_pct
);
tue_sts3215_status_t tue_sts3215_get_overload_protection(
    tue_sts3215_t *dev,
    uint8_t id,
    uint8_t *protective_torque_pct,
    uint8_t *protection_time_10ms,
    uint8_t *overload_torque_pct
);

/* Speed Loop PID & Overcurrent Protection Time (0x25, 0x26, 0x27) */
tue_sts3215_status_t tue_sts3215_set_speed_pid(tue_sts3215_t *dev, uint8_t id, uint8_t speed_p, uint8_t speed_i);
tue_sts3215_status_t tue_sts3215_get_speed_pid(tue_sts3215_t *dev, uint8_t id, uint8_t *speed_p, uint8_t *speed_i);
tue_sts3215_status_t tue_sts3215_set_overcurrent_time(tue_sts3215_t *dev, uint8_t id, uint8_t time_10ms);
tue_sts3215_status_t tue_sts3215_get_overcurrent_time(tue_sts3215_t *dev, uint8_t id, uint8_t *time_10ms);

/* Torque Enable & Zero-Point Calibration (0x28) */
tue_sts3215_status_t tue_sts3215_enable_torque(tue_sts3215_t *dev, uint8_t id, bool enable);
tue_sts3215_status_t tue_sts3215_calibrate_position_to_2048(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_get_torque_enable(tue_sts3215_t *dev, uint8_t id, bool *enabled);

/* Acceleration, Position & Speed Control (0x29, 0x2A, 0x2C, 0x2E, 0x30) */
tue_sts3215_status_t tue_sts3215_set_acceleration(tue_sts3215_t *dev, uint8_t id, uint8_t accel_100steps_s2);
tue_sts3215_status_t tue_sts3215_get_acceleration(tue_sts3215_t *dev, uint8_t id, uint8_t *accel_100steps_s2);
tue_sts3215_status_t tue_sts3215_set_goal_position(tue_sts3215_t *dev, uint8_t id, int16_t position);
tue_sts3215_status_t tue_sts3215_set_goal_position_ex(
    tue_sts3215_t *dev,
    uint8_t id,
    int16_t position,
    int16_t speed,
    uint8_t accel
);
tue_sts3215_status_t tue_sts3215_reg_write_goal_position_ex(
    tue_sts3215_t *dev,
    uint8_t id,
    int16_t position,
    int16_t speed,
    uint8_t accel
);
tue_sts3215_status_t tue_sts3215_set_pwm_runtime(tue_sts3215_t *dev, uint8_t id, int16_t pwm_per_thousand);
tue_sts3215_status_t tue_sts3215_get_pwm_runtime(tue_sts3215_t *dev, uint8_t id, int16_t *pwm_per_thousand);
tue_sts3215_status_t tue_sts3215_set_running_speed(tue_sts3215_t *dev, uint8_t id, int16_t speed_steps_s);
tue_sts3215_status_t tue_sts3215_get_running_speed(tue_sts3215_t *dev, uint8_t id, int16_t *speed_steps_s);
tue_sts3215_status_t tue_sts3215_set_torque_limit(tue_sts3215_t *dev, uint8_t id, uint16_t torque_0_1pct);
tue_sts3215_status_t tue_sts3215_get_torque_limit(tue_sts3215_t *dev, uint8_t id, uint16_t *torque_0_1pct);
tue_sts3215_status_t tue_sts3215_stop(tue_sts3215_t *dev, uint8_t id);

/* Feedback & Telemetry (0x38..0x45) */
tue_sts3215_status_t tue_sts3215_get_current_position(tue_sts3215_t *dev, uint8_t id, int16_t *position);
tue_sts3215_status_t tue_sts3215_get_current_speed(tue_sts3215_t *dev, uint8_t id, int16_t *speed);
tue_sts3215_status_t tue_sts3215_get_current_load(tue_sts3215_t *dev, uint8_t id, int16_t *load_0_1pct);
tue_sts3215_status_t tue_sts3215_get_current_voltage(tue_sts3215_t *dev, uint8_t id, uint8_t *voltage_0_1v);
tue_sts3215_status_t tue_sts3215_get_voltage_volts(tue_sts3215_t *dev, uint8_t id, float *volts);
tue_sts3215_status_t tue_sts3215_get_current_temperature(tue_sts3215_t *dev, uint8_t id, int8_t *temp_c);
tue_sts3215_status_t tue_sts3215_get_async_write_flag(tue_sts3215_t *dev, uint8_t id, uint8_t *flag);
tue_sts3215_status_t tue_sts3215_get_servo_status(tue_sts3215_t *dev, uint8_t id, uint8_t *status_mask);
tue_sts3215_status_t tue_sts3215_is_moving(tue_sts3215_t *dev, uint8_t id, bool *is_moving);
tue_sts3215_status_t tue_sts3215_get_motor_current_raw(tue_sts3215_t *dev, uint8_t id, int16_t *current_6_5ma);
tue_sts3215_status_t tue_sts3215_get_motor_current_ma(tue_sts3215_t *dev, uint8_t id, float *current_ma);

/**
  * @brief  Burst read full feedback telemetry block (registers 0x38..0x46, 15 bytes)
  *         in a single roundtrip bus transaction.
  * @param  dev       Driver handle.
  * @param  id        Servo ID (0-253).
  * @param  feedback  Pointer to feedback structure to populate.
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_get_feedback(tue_sts3215_t *dev, uint8_t id, tue_sts3215_feedback_t *feedback);

/* Multi-Servo Synchronized Commands -----------------------------------------*/

/**
  * @brief  Synchronously update target positions for multiple servos in one packet.
  * @param  dev      Driver handle.
  * @param  targets  Array of position/speed/accel entries.
  * @param  count    Number of servos in array.
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_sync_write_position(
    tue_sts3215_t *dev,
    const tue_sts3215_sync_pos_t *targets,
    uint8_t count
);

/**
  * @brief  Synchronously update running speeds for multiple servos in one packet.
  * @param  dev      Driver handle.
  * @param  targets  Array of speed/accel entries.
  * @param  count    Number of servos in array.
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_sync_write_speed(
    tue_sts3215_t *dev,
    const tue_sts3215_sync_speed_t *targets,
    uint8_t count
);

/**
  * @brief  Synchronously enable/disable torque for multiple servos in one packet.
  * @param  dev      Driver handle.
  * @param  ids      Array of servo IDs.
  * @param  enables  Array of enable flags (1: Enable, 0: Disable).
  * @param  count    Number of servos.
  * @retval TUE_STS3215_OK on success.
  */
tue_sts3215_status_t tue_sts3215_sync_write_torque_enable(
    tue_sts3215_t *dev,
    const uint8_t *ids,
    const uint8_t *enables,
    uint8_t count
);

#ifdef __cplusplus
}
#endif

#endif /* TUE_STS3215_H_ */
