/**
 * Based on Arduino library from Azoteq PTY Ltd
 */
#pragma once
#include <cinttypes>

namespace esphome {
namespace iqs7222c {

/* Device Firmware version select */
#define IQS7222C_v2_23 1
/* Older versions of IQS7222C, might not work as expected */
#define IQS7222C_v2_6 0
#define IQS7222C_v1_13 0

// // Public Global Definitions
// /* For use with Wire.h library. True argument with some functions closes the
//    I2C communication window.*/
// #define STOP true
// /* For use with Wire.h library. False argument with some functions keeps the
//    I2C communication window open. */
// #define RESTART false

// Device Info
#define IQS7222C_PRODUCT_NUM 0x035F

// Info Flags Byte Bits.
#define IQS7222C_ATI_ACTIVE_BIT 0
#define IQS7222C_ATI_ERROR_BIT 1
#define IQS7222C_DEVICE_RESET_BIT 3
#define IQS7222C_POWER_EVENT_BIT_0 4
#define IQS7222C_POWER_EVENT_BIT_1 5
#define IQS7222C_NORMAL_POWER_BIT 0b00
#define IQS7222C_LOW_POWER_BIT 0b01
#define IQS7222C_ULP_BIT 0b10
#define IQS7222C_NORMAL_POWER_UPDATE 6
#define IQS7222C_GLOBAL_HALT 7

/* Channel Proximity Bits */
#define IQS7222C_CH0_PROX_BIT 0
#define IQS7222C_CH1_PROX_BIT 1
#define IQS7222C_CH2_PROX_BIT 2
#define IQS7222C_CH3_PROX_BIT 3
#define IQS7222C_CH4_PROX_BIT 4
#define IQS7222C_CH5_PROX_BIT 5
#define IQS7222C_CH6_PROX_BIT 6
#define IQS7222C_CH7_PROX_BIT 7
#define IQS7222C_CH8_PROX_BIT 0
#define IQS7222C_CH9_PROX_BIT 1

/* Channel Touch Bits */
#define IQS7222C_CH0_TOUCH_BIT 0
#define IQS7222C_CH1_TOUCH_BIT 1
#define IQS7222C_CH2_TOUCH_BIT 2
#define IQS7222C_CH3_TOUCH_BIT 3
#define IQS7222C_CH4_TOUCH_BIT 4
#define IQS7222C_CH5_TOUCH_BIT 5
#define IQS7222C_CH6_TOUCH_BIT 6
#define IQS7222C_CH7_TOUCH_BIT 7
#define IQS7222C_CH8_TOUCH_BIT 0
#define IQS7222C_CH9_TOUCH_BIT 1

/* Utility Bits */
#define IQS7222C_ACK_RESET_BIT 0
#define IQS7222C_SW_RESET_BIT 1
#define IQS7222C_RE_ATI_BIT 2
#define IQS7222C_RESEED_BIT 3
#define IQS7222C_POWER_MODE_BIT_0 4
#define IQS7222C_POWER_MODE_BIT_1 5
#define IQS7222C_INTERFACE_SELECT_BIT_0 6
#define IQS7222C_INTERFACE_SELECT_BIT_1 7
#define IQS7222C_INTERFACE_I2C_STREAM_BIT 0b00
#define IQS7222C_INTERFACE_I2C_EVENT_BIT 0b01
#define IQS7222C_INTERFACE_I2C_STREAM_IN_TOUCH 0b10

/* Defines and structs for IQS7222C states */
/**
 * @brief  iqs7222c Init Enumeration.
 */
typedef enum {
  IQS7222C_INIT_NONE = (uint8_t) 0x00,
  IQS7222C_INIT_VERIFY_PRODUCT,
  IQS7222C_INIT_READ_RESET,
  IQS7222C_INIT_CHIP_RESET,
  IQS7222C_INIT_UPDATE_SETTINGS,
  IQS7222C_INIT_CHECK_RESET,
  IQS7222C_INIT_ACK_RESET,
  IQS7222C_INIT_ATI,
  IQS7222C_INIT_WAIT_FOR_ATI,
  IQS7222C_INIT_READ_DATA,
  IQS7222C_INIT_ACTIVATE_EVENT_MODE,
  IQS7222C_INIT_ACTIVATE_STREAM_IN_TOUCH_MODE,
  IQS7222C_INIT_DONE
} iqs7222c_init_e;

typedef enum {
  IQS7222C_STATE_NONE = (uint8_t) 0x00,
  IQS7222C_STATE_START,
  IQS7222C_STATE_INIT,
  IQS7222C_STATE_SW_RESET,
  IQS7222C_STATE_CHECK_RESET,
  IQS7222C_STATE_RUN,
} iqs7222c_state_e;

typedef enum {
  IQS7222C_CH0 = (uint8_t) 0x00,
  IQS7222C_CH1,
  IQS7222C_CH2,
  IQS7222C_CH3,
  IQS7222C_CH4,
  IQS7222C_CH5,
  IQS7222C_CH6,
  IQS7222C_CH7,
  IQS7222C_CH8,
  IQS7222C_CH9,
} iqs7222c_channel_e;

typedef enum {
  IQS7222C_SLIDER0 = false,
  IQS7222C_SLIDER1 = true,
} iqs7222c_slider_e;

typedef enum {
  IQS7222C_CH_NONE = (uint8_t) 0x00,
  IQS7222C_CH_PROX,
  IQS7222C_CH_TOUCH,
  IQS7222C_CH_UNKNOWN,
} iqs7222c_ch_states;
typedef enum {
  IQS7222C_NORMAL_POWER = (uint8_t) 0x00,
  IQS7222C_LOW_POWER,
  IQS7222C_ULP,
  IQS7222C_HALT,
  IQS7222C_POWER_UNKNOWN
} iqs7222c_power_modes;

/* IQS7222C Memory map data variables, only save the data that might be used
   during program runtime */
#pragma pack(1)
typedef struct {
  /* READ ONLY */                 //  I2C Addresses:
  uint8_t VERSION_DETAILS[20];    // 	0x00 -> 0x09
  uint8_t SYSTEM_STATUS[2];       // 	0x10
  uint8_t EVENTS[2];              // 	0x11
  uint8_t PROX_EVENT_STATES[2];   // 	0x12
  uint8_t TOUCH_EVENT_STATES[2];  // 	0x13
  uint8_t SLIDER_WHEEL_0[2];      // 	0x14
  uint8_t SLIDER_WHEEL_1[2];      // 	0x15
  uint8_t CH0_COUNTS_LTA[2];      // 	0x20
  uint8_t CH1_COUNTS_LTA[2];      // 	0x21
  uint8_t CH2_COUNTS_LTA[2];      // 	0x22
  uint8_t CH3_COUNTS_LTA[2];      // 	0x23
  uint8_t CH4_COUNTS_LTA[2];      // 	0x24
  uint8_t CH5_COUNTS_LTA[2];      // 	0x25
  uint8_t CH6_COUNTS_LTA[2];      // 	0x26
  uint8_t CH7_COUNTS_LTA[2];      // 	0x27
  uint8_t CH8_COUNTS_LTA[2];      // 	0x28
  uint8_t CH9_COUNTS_LTA[2];      // 	0x29

  /* READ WRITE */            //  I2C Addresses:
  uint8_t SYSTEM_CONTROL[2];  // 	0xD0
} IQS7222C_MEMORY_MAP;
#pragma pack(4)

#pragma pack(1)
typedef struct {
  iqs7222c_state_e state;
  iqs7222c_init_e init_state;
} iqs7222c_s;
#pragma pack(4)

}  // namespace iqs7222c
}  // namespace esphome