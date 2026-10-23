/**
  ******************************************************************************
  * @file           : tue_usb.h
  * @brief          : TUE USB binary protocol header for STM32F407
  ******************************************************************************
  * @attention
  *
  * Robust, modular binary request/response protocol over USB CDC.
  * Explicit little-endian serialization, CRC-16/CCITT-FALSE integrity,
  * sequence numbers, and FreeRTOS task-based decoupled frame processing.
  *
  ******************************************************************************
  */

#ifndef TUE_USB_H_
#define TUE_USB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"

/* Protocol Frame Constants --------------------------------------------------*/
#define TUE_USB_SOF1                0xAAU
#define TUE_USB_SOF2                0x55U
#define TUE_USB_HEADER_SIZE         7U   /* SOF1(1) + SOF2(1) + SEQ(1) + CMD(1) + STATUS(1) + LEN(2) */
#define TUE_USB_CRC_SIZE            2U   /* CRC16(2) */
#define TUE_USB_MAX_PAYLOAD         256U
#define TUE_USB_MAX_FRAME_SIZE      (TUE_USB_HEADER_SIZE + TUE_USB_MAX_PAYLOAD + TUE_USB_CRC_SIZE) /* 265 bytes */

/* Status Codes --------------------------------------------------------------*/
typedef enum {
    TUE_STATUS_OK                   = 0x00U,
    TUE_STATUS_ERR_UNKNOWN_CMD      = 0x01U,
    TUE_STATUS_ERR_INVALID_LEN      = 0x02U,
    TUE_STATUS_ERR_INVALID_PARAM    = 0x03U,
    TUE_STATUS_ERR_CRC              = 0x04U,
    TUE_STATUS_ERR_BUSY             = 0x05U,
    TUE_STATUS_ERR_EXEC             = 0x06U
} tue_status_t;

/* Command IDs ---------------------------------------------------------------*/
/* 0x01–0x0F: System Commands */
#define TUE_CMD_SYS_PING            0x01U
#define TUE_CMD_SYS_GET_INFO        0x02U
#define TUE_CMD_SYS_RESET           0x03U
#define TUE_CMD_SYS_LED_CTRL        0x04U
#define TUE_CMD_SYS_GET_STATUS      0x05U

/* 0x10–0x1F: DC Motor Commands */
#define TUE_CMD_DC_SET_DUTY         0x10U
#define TUE_CMD_DC_STOP             0x11U
#define TUE_CMD_DC_GET_STATUS       0x12U
#define TUE_CMD_DC_SET_PID          0x13U

/* 0x20–0x2F: Stepper Motor Commands */
#define TUE_CMD_STEP_ENABLE         0x20U
#define TUE_CMD_STEP_MOVE           0x21U
#define TUE_CMD_STEP_STOP           0x22U
#define TUE_CMD_STEP_GET_POS        0x23U
#define TUE_CMD_STEP_SET_HOMING     0x24U

/* 0x30–0x3F: Servo Commands (STS3215) */
#define TUE_CMD_SERVO_PING          0x30U
#define TUE_CMD_SERVO_READ_REG      0x31U
#define TUE_CMD_SERVO_WRITE_REG     0x32U
#define TUE_CMD_SERVO_SET_POS       0x33U
#define TUE_CMD_SERVO_GET_FEEDBACK  0x34U
#define TUE_CMD_SERVO_TORQUE_ENABLE 0x35U

/* 0x40–0x4F: Sensor Commands */
#define TUE_CMD_SENSOR_READ_IMU     0x40U
#define TUE_CMD_SENSOR_READ_ENCODER 0x41U
#define TUE_CMD_SENSOR_READ_ADC     0x42U
#define TUE_CMD_SENSOR_READ_GPIO    0x43U

/* 0x50–0x5F: Bus Passthrough Commands */
#define TUE_CMD_BUS_CAN_TX          0x50U
#define TUE_CMD_BUS_CAN_RX          0x51U
#define TUE_CMD_BUS_I2C_TRANSFER    0x52U
#define TUE_CMD_BUS_SPI_TRANSFER    0x53U

/* 0x60–0x6F: Configuration and Control Modes */
#define TUE_CMD_CFG_SET_MODE        0x60U
#define TUE_CMD_CFG_GET_MODE        0x61U
#define TUE_CMD_CFG_SAVE_FLASH      0x62U
#define TUE_CMD_CFG_LOAD_DEFAULT    0x63U

/* LED Control Modes ---------------------------------------------------------*/
#define TUE_LED_MODE_OFF            0x00U
#define TUE_LED_MODE_ON             0x01U
#define TUE_LED_MODE_TOGGLE         0x02U
#define TUE_LED_MODE_QUERY          0xFFU

/* DC Motor Operating & Brake Modes (0x10, 0x11) -----------------------------*/
typedef enum {
    TUE_DC_DIR_COAST            = 0x00U,
    TUE_DC_DIR_CW               = 0x01U,
    TUE_DC_DIR_CCW              = 0x02U,
    TUE_DC_DIR_BRAKE            = 0x03U
} tue_dc_dir_t;

typedef enum {
    TUE_DC_BRAKE_COAST          = 0x00U,
    TUE_DC_BRAKE_ACTIVE         = 0x01U
} tue_dc_brake_t;

/* Stepper Move & Stop Modes (0x21, 0x22) ------------------------------------*/
typedef enum {
    TUE_STEP_MODE_ABSOLUTE      = 0x00U,
    TUE_STEP_MODE_RELATIVE      = 0x01U
} tue_step_mode_t;

typedef enum {
    TUE_STEP_STOP_DECEL         = 0x00U,
    TUE_STEP_STOP_EMERGENCY     = 0x01U
} tue_step_stop_t;

/* Frame Structure Definition ------------------------------------------------*/
typedef struct {
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  status;
    uint16_t len;
    uint8_t  payload[TUE_USB_MAX_PAYLOAD];
    uint16_t crc;
} tue_frame_t;

/* Command Handler Signature -------------------------------------------------*/
/**
  * @brief  Command handler function pointer type.
  * @param  req_payload  Pointer to input request payload bytes.
  * @param  req_len      Length of input request payload.
  * @param  resp_payload Buffer to populate output response payload.
  * @param  resp_len     Pointer to store output response payload length.
  * @param  max_resp_len Capacity of resp_payload buffer.
  * @retval tue_status_t Execution status.
  */
typedef tue_status_t (*tue_cmd_handler_t)(
    const uint8_t *req_payload,
    uint16_t req_len,
    uint8_t *resp_payload,
    uint16_t *resp_len,
    uint16_t max_resp_len
);

/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Initializes the TUE USB subsystem: creates FreeRTOS stream buffer,
  *         mutex, and worker task.
  */
void tue_usb_init(void);

/**
  * @brief  Non-blocking ISR callback to be invoked directly from CDC_Receive_FS.
  *         Transfers received byte stream into FreeRTOS stream buffer without parsing.
  * @param  data  Pointer to received data buffer from USB endpoint.
  * @param  len   Number of received bytes.
  */
void tue_usb_on_cdc_rx(const uint8_t *data, uint32_t len);

/**
  * @brief  Feed a single byte into the frame parser state machine.
  * @param  byte  Raw byte to process.
  */
void tue_usb_feed_byte(uint8_t byte);

/**
  * @brief  Registers a command handler into the dynamic command table.
  * @param  cmd_id       Command ID (0x01..0xFF).
  * @param  min_req_len  Minimum accepted request payload length.
  * @param  max_req_len  Maximum accepted request payload length.
  * @param  handler      Function pointer to command handler.
  * @retval tue_status_t TUE_STATUS_OK on success, error if table full.
  */
tue_status_t tue_usb_register_command(
    uint8_t cmd_id,
    uint16_t min_req_len,
    uint16_t max_req_len,
    tue_cmd_handler_t handler
);

/**
  * @brief  Thread-safe transmission of a protocol frame over USB CDC.
  * @param  seq     Sequence number to echo or send.
  * @param  cmd     Command ID.
  * @param  status  Status code.
  * @param  payload Pointer to payload bytes (or NULL if len == 0).
  * @param  len     Payload length.
  * @retval tue_status_t TUE_STATUS_OK or error code.
  */
tue_status_t tue_usb_send_response(
    uint8_t seq,
    uint8_t cmd,
    tue_status_t status,
    const uint8_t *payload,
    uint16_t len
);

/**
  * @brief  Calculates CRC-16/CCITT-FALSE over a buffer using fast table lookup.
  * @param  data     Pointer to data.
  * @param  len      Length in bytes.
  * @param  init_crc Initial CRC value (typically 0xFFFF).
  * @retval uint16_t Computed CRC-16.
  */
uint16_t tue_usb_crc16(const uint8_t *data, size_t len, uint16_t init_crc);

/* Motor Control Subsystem Functions (DC, Stepper, STS3215 Servo) ------------*/

/**
  * @brief  Initialize all motor hardware peripherals (TIM3/4 PWM, Stepper GPIOs, STS3215 UART).
  */
void tue_motor_init(void);

/**
  * @brief  Control DC motor channel PWM duty cycle and direction (0x10 CMD_DC_SET_DUTY).
  * @param  motor_id   Channel 1..4.
  * @param  direction  0: Coast, 1: CW, 2: CCW, 3: Active Brake.
  * @param  duty       0..10000 (0.01% units).
  * @retval tue_status_t Status.
  */
tue_status_t tue_motor_dc_set_duty(uint8_t motor_id, uint8_t direction, uint16_t duty);

/**
  * @brief  Stop or brake specified DC motor channel (0x11 CMD_DC_STOP).
  * @param  motor_id   Channel 1..4 (or 0 for all).
  * @param  brake_mode 0: Coast, 1: Active Brake.
  * @retval tue_status_t Status.
  */
tue_status_t tue_motor_dc_stop(uint8_t motor_id, uint8_t brake_mode);

/**
  * @brief  Query DC motor status and hardware fault input (0x12 CMD_DC_GET_STATUS).
  * @param  motor_id   Channel 1..4.
  * @param  direction  Pointer to store active direction.
  * @param  duty       Pointer to store active duty.
  * @param  fault      Pointer to store fault flag (1: fault active).
  * @retval tue_status_t Status.
  */
tue_status_t tue_motor_dc_get_status(uint8_t motor_id, uint8_t *direction, uint16_t *duty, uint8_t *fault);

/**
  * @brief  Configure velocity PID parameters for DC motor (0x13 CMD_DC_SET_PID).
  */
tue_status_t tue_motor_dc_set_pid(uint8_t motor_id, float kp, float ki, float kd);

/**
  * @brief  Enable or disable stepper motor driver outputs (0x20 CMD_STEP_ENABLE).
  * @param  stepper_id Channel 1..4 (or 0 for all).
  * @param  enable     true to enable, false to disable.
  * @retval tue_status_t Status.
  */
tue_status_t tue_motor_step_enable(uint8_t stepper_id, bool enable);

/**
  * @brief  Command stepper motor target move profile (0x21 CMD_STEP_MOVE).
  */
tue_status_t tue_motor_step_move(
    uint8_t stepper_id,
    bool is_relative,
    int32_t target_steps,
    uint32_t speed_hz,
    uint32_t accel
);

/**
  * @brief  Stop stepper motor motion (0x22 CMD_STEP_STOP).
  */
tue_status_t tue_motor_step_stop(uint8_t stepper_id, bool emergency_stop);

/**
  * @brief  Query stepper current position and moving state (0x23 CMD_STEP_GET_POS).
  */
tue_status_t tue_motor_step_get_pos(uint8_t stepper_id, int32_t *current_steps, bool *is_moving);

/**
  * @brief  Initiate stepper sensor homing sequence (0x24 CMD_STEP_SET_HOMING).
  */
tue_status_t tue_motor_step_set_homing(uint8_t stepper_id, uint8_t direction);

/**
  * @brief  Ping STS3215 servo motor (0x30 CMD_SERVO_PING).
  */
tue_status_t tue_motor_servo_ping(uint8_t servo_id);

/**
  * @brief  Set STS3215 servo goal position, speed, and acceleration (0x33 CMD_SERVO_SET_POS).
  */
tue_status_t tue_motor_servo_set_pos(uint8_t servo_id, int16_t position, int16_t speed, uint8_t accel);

/**
  * @brief  Toggle STS3215 servo torque switch or zero calibration (0x35 CMD_SERVO_TORQUE_ENABLE).
  */
tue_status_t tue_motor_servo_torque_enable(uint8_t servo_id, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* TUE_USB_H_ */
