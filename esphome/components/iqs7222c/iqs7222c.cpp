#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "iqs7222c.h"
#include "iqs7222c_init_azoteq_ui.h"

namespace esphome {
namespace iqs7222c {

bool testing_mode = false;

static const char *const TAG = "iqs7222c";

static const uint16_t cycle_setup_0[4] = {
    IQS_7222C_CYCLE_SETUP_0, (CYCLE_0_CONV_FREQ_PERIOD << 8 | CYCLE_0_CONV_FREQ_FRAC),
    (CYCLE_0_CTX_SELECT << 8 | CYCLE_0_SETTINGS), (CYCLE_0_IREF_1 | CYCLE_0_IREF_0)};

static const uint16_t cycle_setup_1[4] = {
    IQS_7222C_CYCLE_SETUP_1, (CYCLE_1_CONV_FREQ_PERIOD << 8 | CYCLE_1_CONV_FREQ_FRAC),
    (CYCLE_1_CTX_SELECT << 8 | CYCLE_1_SETTINGS), (CYCLE_1_IREF_1 | CYCLE_1_IREF_0)};

static const uint16_t cycle_setup_2[4] = {
    IQS_7222C_CYCLE_SETUP_2, (CYCLE_2_CONV_FREQ_PERIOD << 8 | CYCLE_2_CONV_FREQ_FRAC),
    (CYCLE_2_CTX_SELECT << 8 | CYCLE_2_SETTINGS), (CYCLE_2_IREF_1 | CYCLE_2_IREF_0)};

static const uint16_t cycle_setup_3[4] = {
    IQS_7222C_CYCLE_SETUP_3, (CYCLE_3_CONV_FREQ_PERIOD << 8 | CYCLE_3_CONV_FREQ_FRAC),
    (CYCLE_3_CTX_SELECT << 8 | CYCLE_3_SETTINGS), (CYCLE_3_IREF_1 | CYCLE_3_IREF_0)};

static const uint16_t cycle_setup_4[4] = {
    IQS_7222C_CYCLE_SETUP_4, (CYCLE_4_CONV_FREQ_PERIOD << 8 | CYCLE_4_CONV_FREQ_FRAC),
    (CYCLE_4_CTX_SELECT << 8 | CYCLE_4_SETTINGS), (CYCLE_4_IREF_1 | CYCLE_4_IREF_0)};

static const uint16_t global_cycle_setup[4] = {
    IQS_7222C_GLOBAL_CYCLE_SETUP, (GLOBAL_CYCLE_SETUP_1 << 8 | GLOBAL_CYCLE_SETUP_0),
    (FINE_DIVIDER_PRELOAD | COARSE_DIVIDER_PRELOAD), (COMPENSATION_PRELOAD_1 | COMPENSATION_PRELOAD_0)};

static const uint16_t button_setup_0[4] = {(IQS_7222C_BUTTON_SETUP_0),
                                           (BUTTON_0_ENTER_EXIT << 8 | BUTTON_0_PROX_THRESHOLD),
                                           (BUTTON_0_TOUCH_HYSTERESIS << 8 | BUTTON_0_TOUCH_THRESHOLD),
                                           (BUTTON_0_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_0_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_1[4] = {(IQS_7222C_BUTTON_SETUP_1),
                                           (BUTTON_1_ENTER_EXIT << 8 | BUTTON_1_PROX_THRESHOLD),
                                           (BUTTON_1_TOUCH_HYSTERESIS << 8 | BUTTON_1_TOUCH_THRESHOLD),
                                           (BUTTON_1_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_1_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_2[4] = {(IQS_7222C_BUTTON_SETUP_2),
                                           (BUTTON_2_ENTER_EXIT << 8 | BUTTON_2_PROX_THRESHOLD),
                                           (BUTTON_2_TOUCH_HYSTERESIS << 8 | BUTTON_2_TOUCH_THRESHOLD),
                                           (BUTTON_2_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_2_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_3[4] = {(IQS_7222C_BUTTON_SETUP_3),
                                           (BUTTON_3_ENTER_EXIT << 8 | BUTTON_3_PROX_THRESHOLD),
                                           (BUTTON_3_TOUCH_HYSTERESIS << 8 | BUTTON_3_TOUCH_THRESHOLD),
                                           (BUTTON_3_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_3_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_4[4] = {(IQS_7222C_BUTTON_SETUP_4),
                                           (BUTTON_4_ENTER_EXIT << 8 | BUTTON_4_PROX_THRESHOLD),
                                           (BUTTON_4_TOUCH_HYSTERESIS << 8 | BUTTON_4_TOUCH_THRESHOLD),
                                           (BUTTON_4_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_4_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_5[4] = {(IQS_7222C_BUTTON_SETUP_5),
                                           (BUTTON_5_ENTER_EXIT << 8 | BUTTON_5_PROX_THRESHOLD),
                                           (BUTTON_5_TOUCH_HYSTERESIS << 8 | BUTTON_5_TOUCH_THRESHOLD),
                                           (BUTTON_5_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_5_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_6[4] = {(IQS_7222C_BUTTON_SETUP_6),
                                           (BUTTON_6_ENTER_EXIT << 8 | BUTTON_6_PROX_THRESHOLD),
                                           (BUTTON_6_TOUCH_HYSTERESIS << 8 | BUTTON_6_TOUCH_THRESHOLD),
                                           (BUTTON_6_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_6_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_7[4] = {(IQS_7222C_BUTTON_SETUP_7),
                                           (BUTTON_7_ENTER_EXIT << 8 | BUTTON_7_PROX_THRESHOLD),
                                           (BUTTON_7_TOUCH_HYSTERESIS << 8 | BUTTON_7_TOUCH_THRESHOLD),
                                           (BUTTON_7_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_7_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_8[4] = {(IQS_7222C_BUTTON_SETUP_8),
                                           (BUTTON_8_ENTER_EXIT << 8 | BUTTON_8_PROX_THRESHOLD),
                                           (BUTTON_8_TOUCH_HYSTERESIS << 8 | BUTTON_8_TOUCH_THRESHOLD),
                                           (BUTTON_8_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_8_PROX_EVENT_TIMEOUT)};

static const uint16_t button_setup_9[4] = {(IQS_7222C_BUTTON_SETUP_9),
                                           (BUTTON_9_ENTER_EXIT << 8 | BUTTON_9_PROX_THRESHOLD),
                                           (BUTTON_9_TOUCH_HYSTERESIS << 8 | BUTTON_9_TOUCH_THRESHOLD),
                                           (BUTTON_9_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_9_PROX_EVENT_TIMEOUT)};

static const uint16_t ch_setup_0[7] = {
    (IQS_7222C_CHANNEL_0_SETUP),
    (CH0_SETUP_1 << 8 | CH0_SETUP_0),
    (CH0_ATI_SETTINGS_1 << 8 | CH0_ATI_SETTINGS_0),
    (CH0_MULTIPLIERS_1 << 8 | CH0_MULTIPLIERS_0),
    (CH0_ATI_COMPENSATION_1 << 8 | CH0_ATI_COMPENSATION_0),
    (CH0_REF_PTR_1 << 8 | CH0_REF_PTR_0),
    (CH0_REFMASK_1 << 8 | CH0_REFMASK_0),
};

static const uint16_t ch_setup_1[7] = {
    (IQS_7222C_CHANNEL_1_SETUP),
    (CH1_SETUP_1 << 8 | CH1_SETUP_0),
    (CH1_ATI_SETTINGS_1 << 8 | CH1_ATI_SETTINGS_0),
    (CH1_MULTIPLIERS_1 << 8 | CH1_MULTIPLIERS_0),
    (CH1_ATI_COMPENSATION_1 << 8 | CH1_ATI_COMPENSATION_0),
    (CH1_REF_PTR_1 << 8 | CH1_REF_PTR_0),
    (CH1_REFMASK_1 << 8 | CH1_REFMASK_0),
};

static const uint16_t ch_setup_2[7] = {
    (IQS_7222C_CHANNEL_2_SETUP),
    (CH2_SETUP_1 << 8 | CH2_SETUP_0),
    (CH2_ATI_SETTINGS_1 << 8 | CH2_ATI_SETTINGS_0),
    (CH2_MULTIPLIERS_1 << 8 | CH2_MULTIPLIERS_0),
    (CH2_ATI_COMPENSATION_1 << 8 | CH2_ATI_COMPENSATION_0),
    (CH2_REF_PTR_1 << 8 | CH2_REF_PTR_0),
    (CH2_REFMASK_1 << 8 | CH2_REFMASK_0),
};

static const uint16_t ch_setup_3[7] = {
    (IQS_7222C_CHANNEL_3_SETUP),
    (CH3_SETUP_1 << 8 | CH3_SETUP_0),
    (CH3_ATI_SETTINGS_1 << 8 | CH3_ATI_SETTINGS_0),
    (CH3_MULTIPLIERS_1 << 8 | CH3_MULTIPLIERS_0),
    (CH3_ATI_COMPENSATION_1 << 8 | CH3_ATI_COMPENSATION_0),
    (CH3_REF_PTR_1 << 8 | CH3_REF_PTR_0),
    (CH3_REFMASK_1 << 8 | CH3_REFMASK_0),
};

static const uint16_t ch_setup_4[7] = {
    (IQS_7222C_CHANNEL_4_SETUP),
    (CH4_SETUP_1 << 8 | CH4_SETUP_0),
    (CH4_ATI_SETTINGS_1 << 8 | CH4_ATI_SETTINGS_0),
    (CH4_MULTIPLIERS_1 << 8 | CH4_MULTIPLIERS_0),
    (CH4_ATI_COMPENSATION_1 << 8 | CH4_ATI_COMPENSATION_0),
    (CH4_REF_PTR_1 << 8 | CH4_REF_PTR_0),
    (CH4_REFMASK_1 << 8 | CH4_REFMASK_0),
};

static const uint16_t ch_setup_5[7] = {
    (IQS_7222C_CHANNEL_5_SETUP),
    (CH5_SETUP_1 << 8 | CH5_SETUP_0),
    (CH5_ATI_SETTINGS_1 << 8 | CH5_ATI_SETTINGS_0),
    (CH5_MULTIPLIERS_1 << 8 | CH5_MULTIPLIERS_0),
    (CH5_ATI_COMPENSATION_1 << 8 | CH5_ATI_COMPENSATION_0),
    (CH5_REF_PTR_1 << 8 | CH5_REF_PTR_0),
    (CH5_REFMASK_1 << 8 | CH5_REFMASK_0),
};

static const uint16_t ch_setup_6[7] = {
    (IQS_7222C_CHANNEL_6_SETUP),
    (CH6_SETUP_1 << 8 | CH6_SETUP_0),
    (CH6_ATI_SETTINGS_1 << 8 | CH6_ATI_SETTINGS_0),
    (CH6_MULTIPLIERS_1 << 8 | CH6_MULTIPLIERS_0),
    (CH6_ATI_COMPENSATION_1 << 8 | CH6_ATI_COMPENSATION_0),
    (CH6_REF_PTR_1 << 8 | CH6_REF_PTR_0),
    (CH6_REFMASK_1 << 8 | CH6_REFMASK_0),
};

static const uint16_t ch_setup_7[7] = {
    (IQS_7222C_CHANNEL_7_SETUP),
    (CH7_SETUP_1 << 8 | CH7_SETUP_0),
    (CH7_ATI_SETTINGS_1 << 8 | CH7_ATI_SETTINGS_0),
    (CH7_MULTIPLIERS_1 << 8 | CH7_MULTIPLIERS_0),
    (CH7_ATI_COMPENSATION_1 << 8 | CH7_ATI_COMPENSATION_0),
    (CH7_REF_PTR_1 << 8 | CH7_REF_PTR_0),
    (CH7_REFMASK_1 << 8 | CH7_REFMASK_0),
};

static const uint16_t ch_setup_8[7] = {
    (IQS_7222C_CHANNEL_8_SETUP),
    (CH8_SETUP_1 << 8 | CH8_SETUP_0),
    (CH8_ATI_SETTINGS_1 << 8 | CH8_ATI_SETTINGS_0),
    (CH8_MULTIPLIERS_1 << 8 | CH8_MULTIPLIERS_0),
    (CH8_ATI_COMPENSATION_1 << 8 | CH8_ATI_COMPENSATION_0),
    (CH8_REF_PTR_1 << 8 | CH8_REF_PTR_0),
    (CH8_REFMASK_1 << 8 | CH8_REFMASK_0),
};

static const uint16_t ch_setup_9[7] = {
    (IQS_7222C_CHANNEL_9_SETUP),
    (CH9_SETUP_1 << 8 | CH9_SETUP_0),
    (CH9_ATI_SETTINGS_1 << 8 | CH9_ATI_SETTINGS_0),
    (CH9_MULTIPLIERS_1 << 8 | CH9_MULTIPLIERS_0),
    (CH9_ATI_COMPENSATION_1 << 8 | CH9_ATI_COMPENSATION_0),
    (CH9_REF_PTR_1 << 8 | CH9_REF_PTR_0),
    (CH9_REFMASK_1 << 8 | CH9_REFMASK_0),
};

static const uint16_t filter_beats[3] = {(IQS_7222C_FILTER_BEATS), (LTA_BETA_FILTER << 8 | COUNTS_BETA_FILTER),
                                         (RESERVED_FILTER_0 << 8 | LTA_FAST_BETA_FILTER)};

static const uint16_t slider_wheel_setup_0[11] = {(IQS_7222C_SLIDER_0_SETUP),
                                                  (SLIDER0_LOWER_CAL << 8 | SLIDER0SETUP_GENERAL),
                                                  (SLIDER0_BOTTOM_SPEED << 8 | SLIDER0_UPPER_CAL),
                                                  (SLIDER0_TOPSPEED_1 << 8 | SLIDER0_TOPSPEED_0),
                                                  (SLIDER0_RESOLUTION_1 << 8 | SLIDER0_RESOLUTION_0 << 8),
                                                  (SLIDER0_ENABLE_MASK_8_9 << 8 | SLIDER0_ENABLE_MASK_0_7),
                                                  (SLIDER0_ENABLESTATUSLINK_1 << 8 | SLIDER0_ENABLESTATUSLINK_0),
                                                  (SLIDER0_DELTA0_1 << 8 | SLIDER0_DELTA0_0),
                                                  (SLIDER0_DELTA1_1 << 8 | SLIDER0_DELTA1_0),
                                                  (SLIDER0_DELTA2_1 << 8 | SLIDER0_DELTA2_0),
                                                  (SLIDER0_DELTA3_1 << 8 | SLIDER0_DELTA3_0)};

static const uint16_t slider_wheel_setup_1[11] = {
    (IQS_7222C_SLIDER_1_SETUP),
    (SLIDER1_LOWER_CAL << 8 | SLIDER1SETUP_GENERAL),
    (SLIDER1_BOTTOM_SPEED << 8 | SLIDER1_UPPER_CAL),
    (SLIDER1_TOPSPEED_1 << 8 | SLIDER1_TOPSPEED_0),
    (SLIDER1_RESOLUTION_1 << 8 | SLIDER1_RESOLUTION_0 << 8),
    (SLIDER1_ENABLE_MASK_8_9 << 8 | SLIDER1_ENABLE_MASK_0_7),
    (SLIDER1_ENABLESTATUSLINK_1 << 8 | SLIDER1_ENABLESTATUSLINK_0),
    (SLIDER1_DELTA0_1 << 8 | SLIDER1_DELTA0_0),
    (SLIDER1_DELTA1_1 << 8 | SLIDER1_DELTA1_0),
    (SLIDER1_DELTA2_1 << 8 | SLIDER1_DELTA2_0),
    (SLIDER1_DELTA3_1 << 8 | SLIDER1_DELTA3_0),
};
// // static const uint16_t slider_wheel_setup_0_to_1[21] =
// //     {
// //         (IQS_7222C_SLIDER_0_SETUP),
// //         (SLIDER0_LOWER_CAL << 8 | SLIDER0SETUP_GENERAL),
// //         (SLIDER0_BOTTOM_SPEED << 8 | SLIDER0_UPPER_CAL),
// //         (SLIDER0_TOPSPEED_1 << 8 | SLIDER0_TOPSPEED_0),
// //         (SLIDER0_RESOLUTION_1 << 8 | SLIDER0_RESOLUTION_0 << 8),
// //         (SLIDER0_ENABLE_MASK_8_9 << 8 | SLIDER0_ENABLE_MASK_0_7),
// //         (SLIDER0_ENABLESTATUSLINK_1 << 8 | SLIDER0_ENABLESTATUSLINK_0),
// //         (SLIDER0_DELTA0_1 << 8 | SLIDER0_DELTA0_0),
// //         (SLIDER0_DELTA1_1 << 8 | SLIDER0_DELTA1_0),
// //         (SLIDER0_DELTA2_1 << 8 | SLIDER0_DELTA2_0),
// //         (SLIDER0_DELTA3_1 << 8 | SLIDER0_DELTA3_0),

// //         (SLIDER1_LOWER_CAL << 8 | SLIDER1SETUP_GENERAL),
// //         (SLIDER1_BOTTOM_SPEED << 8 | SLIDER1_UPPER_CAL),
// //         (SLIDER1_TOPSPEED_1 << 8 | SLIDER1_TOPSPEED_0),
// //         (SLIDER1_RESOLUTION_1 << 8 | SLIDER1_RESOLUTION_0 << 8),
// //         (SLIDER1_ENABLE_MASK_8_9 << 8 | SLIDER1_ENABLE_MASK_0_7),
// //         (SLIDER1_ENABLESTATUSLINK_1 << 8 | SLIDER1_ENABLESTATUSLINK_0),
// //         (SLIDER1_DELTA0_1 << 8 | SLIDER1_DELTA0_0),
// //         (SLIDER1_DELTA1_1 << 8 | SLIDER1_DELTA1_0),
// //         (SLIDER1_DELTA2_1 << 8 | SLIDER1_DELTA2_0),
// //         (SLIDER1_DELTA3_1 << 8 | SLIDER1_DELTA3_0),

// // };

static uint16_t gpio_setting_0[4] = {
    (IQS_7222C_GPIO_0_SETTING),
    (GPIO0_SETUP_1 << 8 | GPIO0_SETUP_0),
    (GPIO0_ENABLE_MASK_8_9 << 8 | GPIO0_ENABLE_MASK_0_7),
    (GPIO0_ENABLESTATUSLINK_1 << 8 | GPIO0_ENABLESTATUSLINK_0),

};

static uint16_t gpio_setting_1[4] = {
    (IQS_7222C_GPIO_1_SETTING),
    (GPIO1_SETUP_1 << 8 | GPIO1_SETUP_0),
    (GPIO1_ENABLE_MASK_8_9 << 8 | GPIO1_ENABLE_MASK_0_7),
    (GPIO1_ENABLESTATUSLINK_1 << 8 | GPIO1_ENABLESTATUSLINK_0),

};

static uint16_t gpio_setting_2[4] = {
    (IQS_7222C_GPIO_2_SETTING),
    (GPIO2_SETUP_1 << 8 | GPIO2_SETUP_0),
    (GPIO2_ENABLE_MASK_8_9 << 8 | GPIO2_ENABLE_MASK_0_7),
    (GPIO2_ENABLESTATUSLINK_1 << 8 | GPIO2_ENABLESTATUSLINK_0),

};

static uint8_t pmu_sys_setting[22] = {(IQS_7222C_PMU_SYS_SETTING),
                                      SYSTEM_CONTROL_0,
                                      SYSTEM_CONTROL_1,
                                      ATI_ERROR_TIMEOUT_0,
                                      ATI_ERROR_TIMEOUT_1,
                                      ATI_REPORT_RATE_0,
                                      ATI_REPORT_RATE_1,
                                      NORMAL_MODE_TIMEOUT_0,
                                      NORMAL_MODE_TIMEOUT_1,
                                      NORMAL_MODE_REPORT_RATE_0,
                                      NORMAL_MODE_REPORT_RATE_1,
                                      LP_MODE_TIMEOUT_0,
                                      LP_MODE_TIMEOUT_1,
                                      LP_MODE_REPORT_RATE_0,
                                      LP_MODE_REPORT_RATE_1,
                                      ULP_MODE_TIMEOUT_0,
                                      ULP_MODE_TIMEOUT_1,
                                      ULP_MODE_REPORT_RATE_0,
                                      ULP_MODE_REPORT_RATE_1,
                                      TOUCH_PROX_EVENT_MASK,
                                      POWER_ATI_EVENT_MASK,
                                      I2CCOMMS_0};

static uint8_t soft_reset[3] = {IQS_7222C_PMU_SYS_SETTING, 0x02, 0x00};

void IRAM_ATTR IQS7222CStore::gpio_intr(IQS7222CStore *store) {
  store->deviceRDY = !store->rdy_pin.digital_read();  // RDY active low
}

void IQS7222CComponent::iqs_7222c_init(void) {
  ESP_LOGD(TAG, "iqs_7222c_init");

  iqs_7222c_touch_evt.value = 0;
  if (testing_mode)
    return;

  // iqs_7222c_hal_init();

  iqs_7222c_mclr_set();
  iqs_7222c_delay(100);
  iqs_7222c_mclr_clear();
  iqs_7222c_delay(100);

  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  iqs_7222c_i2c_write((uint8_t *) &soft_reset, sizeof(soft_reset));
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);

  while (iqs_7222c_rdy_read() != 0)
    ;
  iqs_7222c_i2c_write_cont((uint8_t *) &cycle_setup_0, sizeof(cycle_setup_0));
  iqs_7222c_i2c_write_cont((uint8_t *) &cycle_setup_1, sizeof(cycle_setup_1));
  iqs_7222c_i2c_write_cont((uint8_t *) &cycle_setup_2, sizeof(cycle_setup_2));
  iqs_7222c_i2c_write_cont((uint8_t *) &cycle_setup_3, sizeof(cycle_setup_3));
  iqs_7222c_i2c_write_cont((uint8_t *) &cycle_setup_4, sizeof(cycle_setup_4));
  iqs_7222c_i2c_write_cont((uint8_t *) &global_cycle_setup, sizeof(global_cycle_setup));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_0, sizeof(button_setup_0));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_1, sizeof(button_setup_1));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_2, sizeof(button_setup_2));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_3, sizeof(button_setup_3));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_4, sizeof(button_setup_4));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_5, sizeof(button_setup_5));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_6, sizeof(button_setup_6));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_7, sizeof(button_setup_7));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_8, sizeof(button_setup_8));
  iqs_7222c_i2c_write_cont((uint8_t *) &button_setup_9, sizeof(button_setup_9));
  // // iqs_7222c_i2c_write_cont((uint8_t *)&button_stup_0_to_4, sizeof(button_stup_0_to_4));
  // // iqs_7222c_i2c_write_cont((uint8_t *)&button_stup_5_to_9, sizeof(button_stup_5_to_9));
  // // iqs_7222c_i2c_write_cont((uint8_t *)&ch_setup_0_9, sizeof(ch_setup_0_9));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_0, sizeof(ch_setup_0));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_1, sizeof(ch_setup_1));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_2, sizeof(ch_setup_2));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_3, sizeof(ch_setup_3));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_4, sizeof(ch_setup_4));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_5, sizeof(ch_setup_5));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_6, sizeof(ch_setup_6));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_7, sizeof(ch_setup_7));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_8, sizeof(ch_setup_8));
  iqs_7222c_i2c_write_cont((uint8_t *) &ch_setup_9, sizeof(ch_setup_9));

  iqs_7222c_i2c_write_cont((uint8_t *) &filter_beats, sizeof(filter_beats));
  iqs_7222c_i2c_write_cont((uint8_t *) &slider_wheel_setup_0, sizeof(slider_wheel_setup_0));
  iqs_7222c_i2c_write_cont((uint8_t *) &slider_wheel_setup_1, sizeof(slider_wheel_setup_1));
  // // iqs_7222c_i2c_write_cont((uint8_t *)&slider_wheel_setup_0_to_1, sizeof(slider_wheel_setup_0_to_1));
  // // iqs_7222c_i2c_write_cont((uint8_t *)&gpio_setting_0_to_2, sizeof(gpio_setting_0_to_2));
  iqs_7222c_i2c_write_cont((uint8_t *) &gpio_setting_0, sizeof(gpio_setting_0));
  iqs_7222c_i2c_write_cont((uint8_t *) &gpio_setting_1, sizeof(gpio_setting_1));
  iqs_7222c_i2c_write_cont((uint8_t *) &gpio_setting_2, sizeof(gpio_setting_2));
  iqs_7222c_i2c_write((uint8_t *) &pmu_sys_setting, sizeof(pmu_sys_setting));
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);

  uint16_t ch_setup_reg = 0x00A0;
  uint16_t data = 0;
  while (iqs_7222c_rdy_read() != 0)
    ;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch0 setup :0x%04X", data);

  ch_setup_reg = 0x00A1;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch1 setup :0x%04X", data);

  ch_setup_reg = 0x00A2;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch2 setup :0x%04X", data);

  ch_setup_reg = 0x00A3;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch3 setup :0x%04X", data);

  ch_setup_reg = 0x00A4;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch4 setup :0x%04X", data);

  ch_setup_reg = 0x00A5;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch5 setup :0x%04X", data);

  ch_setup_reg = 0x00A6;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch6 setup :0x%04X", data);

  ch_setup_reg = 0x00A7;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch7 setup :0x%04X", data);

  ch_setup_reg = 0x00A8;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch8 setup :0x%04X", data);

  ch_setup_reg = 0x00A9;
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0)
    ;
  data = 0;
  iqs_7222c_i2c_read_registers((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
  ESP_LOGD(TAG, "722c ch9 setup :0x%04X", data);

  iqs_7222c_i2c_stop();
  iqs_7222c_delay(20);
  while (iqs_7222c_rdy_read() != 0) {
    iqs_7222c_read_state(&iqs_7222c_states);
  }
  iqs_7222c_i2c_stop();
  iqs_7222c_delay(100);
  iqs_7222c_initialized = true;
}

void IQS7222CComponent::iqs_7222c_read_version(void) {
  // uint16_t stop_byte = 0xFF;
  uint8_t reg_addr = IQS_7222C_VERSION_REG;
  uint8_t data[6] = {0};

  iqs_7222c_i2c_read_registers(&reg_addr, 1, data, sizeof(data));
  // iqs_7222c_i2c_wite((uint8_t *)&stop_byte, 2);
}
/**
 * @brief Read system status, and event states. (Address 0x10 - 0x13)
 * @note Incuding Proxmity events, touch events
 */
void IQS7222CComponent::iqs_7222c_read_state(iqs_7222c_states_t *state) {
  uint8_t reg_addr = IQS_7222C_SYSTEM_STATES_REG;

  iqs_7222c_i2c_read_registers(&reg_addr, 1, (uint8_t *) state, sizeof(iqs_7222c_states_t));

  if (state->touch.value == 0xEEEE || state->touch.value == 0xFFEE || state->touch.value == 0xEEFF ||
      state->touch.value == 0xFFFF) {
    state->touch.value = 0;
  }
}

void IQS7222CComponent::iqs_7222c_read_touch_event(void) {
  uint8_t reg_addr = IQS_7222C_TOUCH_EVENT_STATES_REG;

  iqs_7222c_i2c_read_registers(&reg_addr, 1, (uint8_t *) &iqs_7222c_touch_evt, sizeof(iqs_7222c_touch_evt));

  /* If the sensor isn ot ready to ready data, it returns 0xEEEE*/
  if (iqs_7222c_touch_evt.value == 0xEEEE) {
    iqs_7222c_i2c_stop();
    iqs_7222c_touch_evt.value = 0;
  }
}

uint8_t IQS7222CComponent::iqs_7222c_rdy_read(void) { return this->interrupt_pin_->digital_read(); }
/**
 * @brief Pull low to trigger MCLR reset
 *
 */
void IQS7222CComponent::iqs_7222c_mclr_set(void) { this->mclr_pin_->digital_write(false); }

void IQS7222CComponent::iqs_7222c_mclr_clear(void) { this->mclr_pin_->digital_write(true); }

void IQS7222CComponent::iqs_7222c_delay(uint32_t ms) { delay(ms); }

void IQS7222CComponent::iqs_7222c_i2c_read_registers(uint8_t *addr, uint16_t addr_size, uint8_t *data,
                                                     uint16_t data_len) {
  i2c::ErrorCode err;
  if (addr_size == 2) {
    err = this->read_register16(*((uint16_t *) addr), data, data_len, false);
  } else {
    err = this->read_register(*addr, data, data_len, false);
  }

  /* If the sensor is not ready to read data, it returns 0xEEEE */
  if (err != i2c::ErrorCode::ERROR_OK) {
    ESP_LOGI(TAG, "err %d %s", err, esp_err_to_name(err));
    ESP_LOGD(TAG, "iqs_7222c_i2c_read_registers. err=  %d", err);
    memset(data, 0, data_len);
    iqs_7222c_i2c_stop();
    delay(20);
    if (addr_size == 2) {
      err = this->read_register16(*((uint16_t *) addr), data, data_len, false);
    } else {
      err = this->read_register(*addr, data, data_len, false);
    }
  }
}

void IQS7222CComponent::iqs_7222c_i2c_stop(void) {
  uint16_t stop_byte = 0x00FF;
  this->bus_->write(address_, (uint8_t *) &stop_byte, 2, true);
}

void IQS7222CComponent::iqs_7222c_i2c_write(uint8_t *data, uint16_t data_len) {
  this->bus_->write(address_, data, data_len, true);
}

/**
 * @brief Do not send stop after writing
 *
 * @param data
 * @param data_len
 */
void IQS7222CComponent::iqs_7222c_i2c_write_cont(uint8_t *data, uint16_t data_len) {
  this->bus_->write(address_, data, data_len, false);
}

void IQS7222CComponent::iqs_7222c_i2c_read(uint8_t *data, uint16_t data_len) {
  this->bus_->read(address_, data, data_len);
}

/********************************************************************************** */

void IQS7222CChannel::publish(bool state) {
  ESP_LOGD(TAG, "Channel %d state: %d", channel_, state);
  this->publish_state(state);
}

void IQS7222CComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IQS7222C...");

  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->pin_mode(gpio::FLAG_INPUT);
    this->interrupt_pin_->setup();
    this->attach_interrupt_(this->interrupt_pin_, gpio::INTERRUPT_ANY_EDGE);
  }

  this->mclr_pin_->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLUP);
  this->mclr_pin_->setup();

  this->set_timeout(1 * 1000, [this]() {
    ESP_LOGD(TAG, "First run execution. publishing buttons");
    for (auto btn = 0; btn < IQS7222C_MAX_BUTTONS; btn++) {
      for (auto *channel : this->channels[btn]) {
        channel->publish_state(false);
      }
    }
  });

  this->set_timeout(5 * 1000, [this]() {
    ESP_LOGD(TAG, "Delayed init");
    iqs_7222c_init();
  });
}

void IQS7222CComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "IQS7222C:");
  LOG_I2C_DEVICE(this);
  LOG_PIN("  MCLR Pin: ", this->mclr_pin_);
  LOG_PIN("  RDY Pin: ", this->interrupt_pin_);

  // ESP_LOGCONFIG(TAG, "  Product ID: 0x%x", this->iqs7222c_product_id_);
  // ESP_LOGCONFIG(TAG, "  Manufacture ID: 0x%x", this->iqs7222c_manufacture_id_);
  // ESP_LOGCONFIG(TAG, "  Revision ID: 0x%x", this->iqs7222c_revision_);

  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "Product ID or Manufacture ID of the connected device does not match a known IQS7222C.");
      break;
    case NONE:
    default:
      break;
  }
}

void IQS7222CComponent::emulate_touch(uint8_t btn) {
  if (btn >= IQS7222C_MAX_BUTTONS)
    return;
  for (auto *channel : this->channels[btn]) {
    ESP_LOGD(TAG, "Emulate Button %d", btn);
    channel->publish_state(true);
  }

  this->set_timeout(50, [this, btn]() {
    for (auto *channel : this->channels[btn]) {
      ESP_LOGD(TAG, "Emulate Button %d", btn);
      channel->publish_state(false);
    }
  });
}

void IQS7222CComponent::loop() {
  static uint16_t old_status{0xffff};
  static uint16_t old_touch_status{0xffff};

  if (testing_mode || iqs_7222c_initialized) {
    // while (iqs_7222c_rdy_read() != 0)
    if (testing_mode || iqs_7222c_rdy_read() == 0) {  // RDY low
      if (!testing_mode) {
        iqs_7222c_read_state(&iqs_7222c_states);
      }

      if (old_touch_status != iqs_7222c_states.touch.value) {
        ESP_LOGD(TAG, "Touch event: %d", iqs_7222c_states.touch.value);
        new_data_available = true;
      }
      if (old_status != iqs_7222c_states.status.value) {
        ESP_LOGD(TAG, "Status event: %d", iqs_7222c_states.status.value);
      }
    }

    if (new_data_available) {
      ESP_LOGD(TAG, "New touch data available");

      new_data_available = false;

      for (auto btn = 0; btn < IQS7222C_MAX_BUTTONS; btn++) {
        uint16_t touch_bit = (uint16_t) 1 << btn;
        bool touched_now = (iqs_7222c_states.touch.value & touch_bit) != 0;
        bool touched_before = (old_touch_status & touch_bit) != 0;

        if (touched_now && !touched_before) {
          ESP_LOGD(TAG, "Button %d touched", btn);
        } else if (touched_before && !touched_now) {
          ESP_LOGD(TAG, "Button %d released", btn);
        }

        if (touched_now != touched_before) {
          for (auto *channel : this->channels[btn]) {
            channel->publish_state(touched_now);
          }
        }
      }
    }
    old_touch_status = iqs_7222c_states.touch.value;
    old_status = iqs_7222c_states.status.value;
  }
}

void IQS7222CComponent::attach_interrupt_(InternalGPIOPin *irq_pin, esphome::gpio::InterruptType type) {
  ESP_LOGD(TAG, "attach_interrupt_(RDY)");
  this->store_.rdy_pin = irq_pin->to_isr();
  this->clear_rdy_interrupt_();
  irq_pin->attach_interrupt(IQS7222CStore::gpio_intr, &this->store_, type);
}

void IQS7222CComponent::clear_rdy_interrupt_(void) { store_.deviceRDY = false; }

bool IQS7222CComponent::get_rdy_interrupt_(void) { return store_.deviceRDY; }

}  // namespace iqs7222c
}  // namespace esphome
