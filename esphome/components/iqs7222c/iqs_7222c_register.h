#ifndef __IQS_7222C_REGISTER_H
#define __IQS_7222C_REGISTER_H

#define IQS_7222C_VERSION_REG 0x0
#define IQS_7222C_SYSTEM_STATES_REG 0x10
#define IQS_7222C_EVENT_STATES_REG 0x11
#define IQS_7222C_PROX_EVENT_STATES_REG 0x12
#define IQS_7222C_TOUCH_EVENT_STATES_REG 0x13
#define IQS_7222C_SLIDER_WHEEL_0_OUTPUT_REG 0x14
#define IQS_7222C_SLIDER_WHEEL_1_OUTPUT_REG 0x15

#define IQS_7222C_CYCLE_SETUP_0 0x0080
#define IQS_7222C_CYCLE_SETUP_1 0x0081
#define IQS_7222C_CYCLE_SETUP_2 0x0082
#define IQS_7222C_CYCLE_SETUP_3 0x0083
#define IQS_7222C_CYCLE_SETUP_4 0x0084

#define IQS_7222C_GLOBAL_CYCLE_SETUP 0x0085 /* 0x8500-0x8503 */

#define IQS_7222C_BUTTON_SETUP_0 0x0090 /* 0x9000 - 0x9002*/
#define IQS_7222C_BUTTON_SETUP_1 0x0091
#define IQS_7222C_BUTTON_SETUP_2 0x0092
#define IQS_7222C_BUTTON_SETUP_3 0x0093
#define IQS_7222C_BUTTON_SETUP_4 0x0094
#define IQS_7222C_BUTTON_SETUP_5 0x0095
#define IQS_7222C_BUTTON_SETUP_6 0x0096
#define IQS_7222C_BUTTON_SETUP_7 0x0097
#define IQS_7222C_BUTTON_SETUP_8 0x0098
#define IQS_7222C_BUTTON_SETUP_9 0x0099

#define IQS_7222C_CHANNEL_0_SETUP 0x00A0 /* 0xA000 - 0xA005*/
#define IQS_7222C_CHANNEL_1_SETUP 0x00A1
#define IQS_7222C_CHANNEL_2_SETUP 0x00A2
#define IQS_7222C_CHANNEL_3_SETUP 0x00A3
#define IQS_7222C_CHANNEL_4_SETUP 0x00A4
#define IQS_7222C_CHANNEL_5_SETUP 0x00A5
#define IQS_7222C_CHANNEL_6_SETUP 0x00A6
#define IQS_7222C_CHANNEL_7_SETUP 0x00A7
#define IQS_7222C_CHANNEL_8_SETUP 0x00A8
#define IQS_7222C_CHANNEL_9_SETUP 0x00A9

#define IQS_7222C_FILTER_BEATS 0x00AA /* 0xAA00 - 0xAA01 */

#define IQS_7222C_SLIDER_0_SETUP 0x00B0 /* 0xB000 - 0xB009 */
#define IQS_7222C_SLIDER_1_SETUP 0x00B1

#define IQS_7222C_GPIO_0_SETTING 0x00C0 /* 0xC000 - 0xC002 */
#define IQS_7222C_GPIO_1_SETTING 0x00C1
#define IQS_7222C_GPIO_2_SETTING 0x00C2

#define IQS_7222C_PMU_SYS_SETTING 0xD0 /* 0xD0 - 0xDB */

/* Reg = 0x10*/
typedef union iqs_722c_system_status {
  /* data */
  uint16_t value;
  struct {
    uint16_t ati_active : 1;
    uint16_t ati_error : 1;
    uint16_t reserve_1 : 1;
    uint16_t reset : 1;      /* 1 = reset occur; 0 = reset not occur */
    uint16_t power_mode : 2; /* 00 = normal power mode; 01 = low power mode; 10 = ultra-low power mode */
    uint16_t np_update : 1;  /* 1 = normal power update occured; 0 = no normal power update occured */
    uint16_t global_hal : 1; /* 1 = global halt active; 0 = global halt not active */
    uint16_t reserve_2 : 8;
  } flag;

} iqs_7222c_system_status_t;

/* Regsiter 0x11*/
typedef union iqs_7222c_events {
  uint16_t value;
  struct {
    uint16_t prox_event : 1;
    uint16_t touch_event : 1;
    uint16_t reserved : 10;
    uint16_t ati_event : 1;
    uint16_t power_event : 1;
    uint16_t reserved_2 : 2;
  } flag;

} iqs_7222c_event_t;

/* Register 0x12 */
typedef union iqs_7222c_prox_event {
  uint16_t value;
  struct {
    uint16_t ch0 : 1;
    uint16_t ch1 : 1;
    uint16_t ch2 : 1;
    uint16_t ch3 : 1;
    uint16_t ch4 : 1;
    uint16_t ch5 : 1;
    uint16_t ch6 : 1;
    uint16_t ch7 : 1;
    uint16_t ch8 : 1;
    uint16_t ch9 : 1;
    uint16_t reserved : 6;
  } flag;
} iqs_7222c_prox_event_t;

/* Register 0x13 */
typedef union iqs_7222c_touch_event {
  uint16_t value;
  struct {
    uint16_t ch0 : 1;
    uint16_t ch1 : 1;
    uint16_t ch2 : 1;
    uint16_t ch3 : 1;
    uint16_t ch4 : 1;
    uint16_t ch5 : 1;
    uint16_t ch6 : 1;
    uint16_t ch7 : 1;
    uint16_t ch8 : 1;
    uint16_t ch9 : 1;
    uint16_t reserved : 6;
  } flag;
} iqs_7222c_touch_event_t;

typedef struct iqs_7222c_states {
  /* data */
  iqs_7222c_system_status_t status;
  iqs_7222c_event_t event;
  iqs_7222c_prox_event_t prox;
  iqs_7222c_touch_event_t touch;
} iqs_7222c_states_t;

#endif