#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "iqs7222c.h"
#include "iqs7222c_init_default.h"

namespace esphome {
namespace iqs7222c {

static const char *const TAG = "iqs7222c";

static uint16_t cycle_setup[5][4] = {
    {IQS_7222C_CYCLE_SETUP_0, (CYCLE_0_CONV_FREQ_PERIOD << 8 | CYCLE_0_CONV_FREQ_FRAC),
     (CYCLE_0_CTX_SELECT << 8 | CYCLE_0_SETTINGS), (CYCLE_0_IREF_1 << 8 | CYCLE_0_IREF_0)},
    {IQS_7222C_CYCLE_SETUP_1, (CYCLE_1_CONV_FREQ_PERIOD << 8 | CYCLE_1_CONV_FREQ_FRAC),
     (CYCLE_1_CTX_SELECT << 8 | CYCLE_1_SETTINGS), (CYCLE_1_IREF_1 << 8 | CYCLE_1_IREF_0)},
    {IQS_7222C_CYCLE_SETUP_2, (CYCLE_2_CONV_FREQ_PERIOD << 8 | CYCLE_2_CONV_FREQ_FRAC),
     (CYCLE_2_CTX_SELECT << 8 | CYCLE_2_SETTINGS), (CYCLE_2_IREF_1 << 8 | CYCLE_2_IREF_0)},
    {IQS_7222C_CYCLE_SETUP_3, (CYCLE_3_CONV_FREQ_PERIOD << 8 | CYCLE_3_CONV_FREQ_FRAC),
     (CYCLE_3_CTX_SELECT << 8 | CYCLE_3_SETTINGS), (CYCLE_3_IREF_1 << 8 | CYCLE_3_IREF_0)},
    {IQS_7222C_CYCLE_SETUP_4, (CYCLE_4_CONV_FREQ_PERIOD << 8 | CYCLE_4_CONV_FREQ_FRAC),
     (CYCLE_4_CTX_SELECT << 8 | CYCLE_4_SETTINGS), (CYCLE_4_IREF_1 << 8 | CYCLE_4_IREF_0)}};

static uint16_t global_cycle_setup[4] = {
    IQS_7222C_GLOBAL_CYCLE_SETUP, (GLOBAL_CYCLE_SETUP_1 << 8 | GLOBAL_CYCLE_SETUP_0),
    (FINE_DIVIDER_PRELOAD << 8 | COARSE_DIVIDER_PRELOAD), (COMPENSATION_PRELOAD_1 << 8 | COMPENSATION_PRELOAD_0)};

static uint16_t button_setup[IQS7222C_MAX_BUTTONS][4] = {
    {(IQS_7222C_BUTTON_SETUP_0), (BUTTON_0_ENTER_EXIT << 8 | BUTTON_0_PROX_THRESHOLD),
     (BUTTON_0_TOUCH_HYSTERESIS << 8 | BUTTON_0_TOUCH_THRESHOLD),
     (BUTTON_0_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_0_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_1), (BUTTON_1_ENTER_EXIT << 8 | BUTTON_1_PROX_THRESHOLD),
     (BUTTON_1_TOUCH_HYSTERESIS << 8 | BUTTON_1_TOUCH_THRESHOLD),
     (BUTTON_1_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_1_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_2), (BUTTON_2_ENTER_EXIT << 8 | BUTTON_2_PROX_THRESHOLD),
     (BUTTON_2_TOUCH_HYSTERESIS << 8 | BUTTON_2_TOUCH_THRESHOLD),
     (BUTTON_2_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_2_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_3), (BUTTON_3_ENTER_EXIT << 8 | BUTTON_3_PROX_THRESHOLD),
     (BUTTON_3_TOUCH_HYSTERESIS << 8 | BUTTON_3_TOUCH_THRESHOLD),
     (BUTTON_3_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_3_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_4), (BUTTON_4_ENTER_EXIT << 8 | BUTTON_4_PROX_THRESHOLD),
     (BUTTON_4_TOUCH_HYSTERESIS << 8 | BUTTON_4_TOUCH_THRESHOLD),
     (BUTTON_4_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_4_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_5), (BUTTON_5_ENTER_EXIT << 8 | BUTTON_5_PROX_THRESHOLD),
     (BUTTON_5_TOUCH_HYSTERESIS << 8 | BUTTON_5_TOUCH_THRESHOLD),
     (BUTTON_5_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_5_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_6), (BUTTON_6_ENTER_EXIT << 8 | BUTTON_6_PROX_THRESHOLD),
     (BUTTON_6_TOUCH_HYSTERESIS << 8 | BUTTON_6_TOUCH_THRESHOLD),
     (BUTTON_6_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_6_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_7), (BUTTON_7_ENTER_EXIT << 8 | BUTTON_7_PROX_THRESHOLD),
     (BUTTON_7_TOUCH_HYSTERESIS << 8 | BUTTON_7_TOUCH_THRESHOLD),
     (BUTTON_7_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_7_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_8), (BUTTON_8_ENTER_EXIT << 8 | BUTTON_8_PROX_THRESHOLD),
     (BUTTON_8_TOUCH_HYSTERESIS << 8 | BUTTON_8_TOUCH_THRESHOLD),
     (BUTTON_8_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_8_PROX_EVENT_TIMEOUT)},

    {(IQS_7222C_BUTTON_SETUP_9), (BUTTON_9_ENTER_EXIT << 8 | BUTTON_9_PROX_THRESHOLD),
     (BUTTON_9_TOUCH_HYSTERESIS << 8 | BUTTON_9_TOUCH_THRESHOLD),
     (BUTTON_9_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_9_PROX_EVENT_TIMEOUT)},
};

static uint16_t ch_setup[IQS7222C_MAX_CHANNELS][7] = {{
                                                          (IQS_7222C_CHANNEL_0_SETUP),
                                                          (CH0_SETUP_1 << 8 | CH0_SETUP_0),
                                                          (CH0_ATI_SETTINGS_1 << 8 | CH0_ATI_SETTINGS_0),
                                                          (CH0_MULTIPLIERS_1 << 8 | CH0_MULTIPLIERS_0),
                                                          (CH0_ATI_COMPENSATION_1 << 8 | CH0_ATI_COMPENSATION_0),
                                                          (CH0_REF_PTR_1 << 8 | CH0_REF_PTR_0),
                                                          (CH0_REFMASK_1 << 8 | CH0_REFMASK_0),
                                                      },

                                                      {
                                                          (IQS_7222C_CHANNEL_1_SETUP),
                                                          (CH1_SETUP_1 << 8 | CH1_SETUP_0),
                                                          (CH1_ATI_SETTINGS_1 << 8 | CH1_ATI_SETTINGS_0),
                                                          (CH1_MULTIPLIERS_1 << 8 | CH1_MULTIPLIERS_0),
                                                          (CH1_ATI_COMPENSATION_1 << 8 | CH1_ATI_COMPENSATION_0),
                                                          (CH1_REF_PTR_1 << 8 | CH1_REF_PTR_0),
                                                          (CH1_REFMASK_1 << 8 | CH1_REFMASK_0),
                                                      },
                                                      {
                                                          (IQS_7222C_CHANNEL_2_SETUP),
                                                          (CH2_SETUP_1 << 8 | CH2_SETUP_0),
                                                          (CH2_ATI_SETTINGS_1 << 8 | CH2_ATI_SETTINGS_0),
                                                          (CH2_MULTIPLIERS_1 << 8 | CH2_MULTIPLIERS_0),
                                                          (CH2_ATI_COMPENSATION_1 << 8 | CH2_ATI_COMPENSATION_0),
                                                          (CH2_REF_PTR_1 << 8 | CH2_REF_PTR_0),
                                                          (CH2_REFMASK_1 << 8 | CH2_REFMASK_0),
                                                      },
                                                      {
                                                          (IQS_7222C_CHANNEL_3_SETUP),
                                                          (CH3_SETUP_1 << 8 | CH3_SETUP_0),
                                                          (CH3_ATI_SETTINGS_1 << 8 | CH3_ATI_SETTINGS_0),
                                                          (CH3_MULTIPLIERS_1 << 8 | CH3_MULTIPLIERS_0),
                                                          (CH3_ATI_COMPENSATION_1 << 8 | CH3_ATI_COMPENSATION_0),
                                                          (CH3_REF_PTR_1 << 8 | CH3_REF_PTR_0),
                                                          (CH3_REFMASK_1 << 8 | CH3_REFMASK_0),
                                                      },
                                                      {
                                                          (IQS_7222C_CHANNEL_4_SETUP),
                                                          (CH4_SETUP_1 << 8 | CH4_SETUP_0),
                                                          (CH4_ATI_SETTINGS_1 << 8 | CH4_ATI_SETTINGS_0),
                                                          (CH4_MULTIPLIERS_1 << 8 | CH4_MULTIPLIERS_0),
                                                          (CH4_ATI_COMPENSATION_1 << 8 | CH4_ATI_COMPENSATION_0),
                                                          (CH4_REF_PTR_1 << 8 | CH4_REF_PTR_0),
                                                          (CH4_REFMASK_1 << 8 | CH4_REFMASK_0),
                                                      },
                                                      {
                                                          (IQS_7222C_CHANNEL_5_SETUP),
                                                          (CH5_SETUP_1 << 8 | CH5_SETUP_0),
                                                          (CH5_ATI_SETTINGS_1 << 8 | CH5_ATI_SETTINGS_0),
                                                          (CH5_MULTIPLIERS_1 << 8 | CH5_MULTIPLIERS_0),
                                                          (CH5_ATI_COMPENSATION_1 << 8 | CH5_ATI_COMPENSATION_0),
                                                          (CH5_REF_PTR_1 << 8 | CH5_REF_PTR_0),
                                                          (CH5_REFMASK_1 << 8 | CH5_REFMASK_0),
                                                      },
                                                      {
                                                          (IQS_7222C_CHANNEL_6_SETUP),
                                                          (CH6_SETUP_1 << 8 | CH6_SETUP_0),
                                                          (CH6_ATI_SETTINGS_1 << 8 | CH6_ATI_SETTINGS_0),
                                                          (CH6_MULTIPLIERS_1 << 8 | CH6_MULTIPLIERS_0),
                                                          (CH6_ATI_COMPENSATION_1 << 8 | CH6_ATI_COMPENSATION_0),
                                                          (CH6_REF_PTR_1 << 8 | CH6_REF_PTR_0),
                                                          (CH6_REFMASK_1 << 8 | CH6_REFMASK_0),
                                                      },
                                                      {
                                                          (IQS_7222C_CHANNEL_7_SETUP),
                                                          (CH7_SETUP_1 << 8 | CH7_SETUP_0),
                                                          (CH7_ATI_SETTINGS_1 << 8 | CH7_ATI_SETTINGS_0),
                                                          (CH7_MULTIPLIERS_1 << 8 | CH7_MULTIPLIERS_0),
                                                          (CH7_ATI_COMPENSATION_1 << 8 | CH7_ATI_COMPENSATION_0),
                                                          (CH7_REF_PTR_1 << 8 | CH7_REF_PTR_0),
                                                          (CH7_REFMASK_1 << 8 | CH7_REFMASK_0),
                                                      },
                                                      {
                                                          (IQS_7222C_CHANNEL_8_SETUP),
                                                          (CH8_SETUP_1 << 8 | CH8_SETUP_0),
                                                          (CH8_ATI_SETTINGS_1 << 8 | CH8_ATI_SETTINGS_0),
                                                          (CH8_MULTIPLIERS_1 << 8 | CH8_MULTIPLIERS_0),
                                                          (CH8_ATI_COMPENSATION_1 << 8 | CH8_ATI_COMPENSATION_0),
                                                          (CH8_REF_PTR_1 << 8 | CH8_REF_PTR_0),
                                                          (CH8_REFMASK_1 << 8 | CH8_REFMASK_0),
                                                      },
                                                      {
                                                          (IQS_7222C_CHANNEL_9_SETUP),
                                                          (CH9_SETUP_1 << 8 | CH9_SETUP_0),
                                                          (CH9_ATI_SETTINGS_1 << 8 | CH9_ATI_SETTINGS_0),
                                                          (CH9_MULTIPLIERS_1 << 8 | CH9_MULTIPLIERS_0),
                                                          (CH9_ATI_COMPENSATION_1 << 8 | CH9_ATI_COMPENSATION_0),
                                                          (CH9_REF_PTR_1 << 8 | CH9_REF_PTR_0),
                                                          (CH9_REFMASK_1 << 8 | CH9_REFMASK_0),
                                                      }};

static const uint16_t filter_betas[3] = {(IQS_7222C_FILTER_BETAS), (LTA_BETA_FILTER << 8 | COUNTS_BETA_FILTER),
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

static const uint16_t slider_wheel_setup_1[11] = {(IQS_7222C_SLIDER_1_SETUP),
                                                  (SLIDER1_LOWER_CAL << 8 | SLIDER1SETUP_GENERAL),
                                                  (SLIDER1_BOTTOM_SPEED << 8 | SLIDER1_UPPER_CAL),
                                                  (SLIDER1_TOPSPEED_1 << 8 | SLIDER1_TOPSPEED_0),
                                                  (SLIDER1_RESOLUTION_1 << 8 | SLIDER1_RESOLUTION_0 << 8),
                                                  (SLIDER1_ENABLE_MASK_8_9 << 8 | SLIDER1_ENABLE_MASK_0_7),
                                                  (SLIDER1_ENABLESTATUSLINK_1 << 8 | SLIDER1_ENABLESTATUSLINK_0),
                                                  (SLIDER1_DELTA0_1 << 8 | SLIDER1_DELTA0_0),
                                                  (SLIDER1_DELTA1_1 << 8 | SLIDER1_DELTA1_0),
                                                  (SLIDER1_DELTA2_1 << 8 | SLIDER1_DELTA2_0),
                                                  (SLIDER1_DELTA3_1 << 8 | SLIDER1_DELTA3_0)};

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

static const uint16_t stop_byte = 0x00FF;

#define WAIT_FOR_RDY_WINDOW() \
  { \
    while (this->rdy_read_() != 0) \
      delay(1); \
  }

#define REPORT_RDY_WINDOW() \
  { \
    if (this->rdy_read_() == 0) { \
      ESP_LOGD(TAG, "RDY opened"); \
    } else { \
      ESP_LOGD(TAG, "RDY closed...."); \
    } \
  }

/***********************************************************************************/

void IQS7222CButton::publish(bool state) {
  ESP_LOGD(TAG, "IQS7222CButton %d state: %d", channel_, state);
  this->publish_state(state);
}

/***********************************************************************************/
float IQS7222CComponent::get_setup_priority() const { return setup_priority::DATA; }

void IQS7222CComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IQS7222C...");

  this->rdy_pin_->pin_mode(gpio::FLAG_INPUT);
  this->rdy_pin_->setup();

  this->mclr_pin_->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLUP);
  this->mclr_pin_->setup();

  if (!this->no_init_) {
    State first_state = this->test_mode_ ? State::RUNTIME : State::INIT_HARD_RESET;

    if (this->init_delay_ms_ > 0) {
      this->set_next_state_delayed_(first_state, this->init_delay_ms_);
    } else {
      this->set_next_state_(first_state);
    }
  }
}

void IQS7222CComponent::start_init() {
  if (!(this->state_ == State::NOT_INITIALIZED || this->state_ == State::RUNTIME)) {
    ESP_LOGW(TAG, "IQS7222C can be re-initialized only in non-init or runtime state.");
    return;
  }
  ESP_LOGW(TAG, "IQS7222C manual (re)init");
  State first_state = this->test_mode_ ? State::RUNTIME : State::INIT_HARD_RESET;
  this->set_next_state_(first_state);
}

void IQS7222CComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "IQS7222C:");

  LOG_I2C_DEVICE(this);
  LOG_PIN("  MCLR Pin: ", this->mclr_pin_);
  LOG_PIN("  RDY Pin: ", this->rdy_pin_);

  ESP_LOGCONFIG(TAG, "  Product Nr: %d", this->product_version_.product_nr);
  ESP_LOGCONFIG(TAG, "  Version: %d.%d", this->product_version_.ver_major, this->product_version_.ver_minor);

  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "Product Nr of the connected device does not match a known IQS7222C.");
      break;
    case NONE:
    default:
      break;
  }
}

void IQS7222CComponent::loop() {
  this->report_state_();
  switch (this->state_) {
    case State::NOT_INITIALIZED:
    case State::WAIT:
      break;
    case State::INIT_HARD_RESET:
      this->mclr_set_();  // Pull MCLR low to enter reset state
      this->set_next_state_delayed_(State::INIT_WAIT_MCLR, 100);
      break;
    case State::INIT_WAIT_MCLR:
      this->mclr_clear_();
      this->set_next_state_delayed_(State::INIT_READ_PRODUCT_VERSION, 100);
      break;
    case State::INIT_READ_PRODUCT_VERSION: {
      WAIT_FOR_RDY_WINDOW();
      this->read_product_version_();
      this->i2c_stop_();
      if (this->product_version_.product_nr == IQS7222C_PRODUCT_NR) {
        this->set_next_state_(State::INIT_SOFT_RESET);
        break;
      }
      // try one more time
      delay(20);
      WAIT_FOR_RDY_WINDOW();
      this->read_product_version_();
      this->i2c_stop_();
      if (this->product_version_.product_nr == IQS7222C_PRODUCT_NR) {
        this->set_next_state_(State::INIT_SOFT_RESET);
        break;
      }
      ESP_LOGE(TAG, "Product Nr of the connected device does not match a known IQS7222C. Got: %d",
               this->product_version_.product_nr);
      this->mark_failed();
      this->error_code_ = COMMUNICATION_FAILED;
      this->set_next_state_(State::NOT_INITIALIZED);
    } break;

    case State::INIT_SOFT_RESET: {
      this->i2c_stop_and_delay_();
      WAIT_FOR_RDY_WINDOW();
      this->i2c_write_((uint8_t *) &soft_reset, sizeof(soft_reset));
      this->i2c_stop_();  // and_delay_();
      this->set_next_state_(State::INIT_WRITE_SETTINGS);
    } break;

    case State::INIT_WRITE_SETTINGS: {
      this->write_settings_();
      this->set_next_state_(State::INIT_READ_STATE);
    } break;

    case State::INIT_READ_STATE: {
      WAIT_FOR_RDY_WINDOW();
      this->read_device_states_();
      this->i2c_stop_();
      if (this->device_states_.status.flag.reset) {
        ESP_LOGD(TAG, "Device reset detected");
        this->set_next_state_(State::ACK_RESET);
        break;
      }
    } break;

    case State::ACK_RESET: {
      uint8_t ack_reset[3];
      ack_reset[0] = pmu_sys_setting[0];
      ack_reset[1] = pmu_sys_setting[1] | 1;
      ack_reset[2] = pmu_sys_setting[2];
      WAIT_FOR_RDY_WINDOW();
      this->i2c_write_((uint8_t *) &ack_reset, sizeof(ack_reset));
      this->i2c_stop_();
      if (this->ati_enabled_) {
        this->set_next_state_(State::INIT_ATI);
      } else {
        this->set_next_state_delayed_(State::RUNTIME, 100);
      }
    } break;

    case State::INIT_ATI: {
      uint8_t init_ati[3];
      init_ati[0] = pmu_sys_setting[0];
      init_ati[1] = pmu_sys_setting[1] | 4;
      init_ati[2] = pmu_sys_setting[2];
      WAIT_FOR_RDY_WINDOW();
      this->i2c_write_((uint8_t *) &init_ati, sizeof(init_ati));
      this->i2c_stop_();
      this->set_next_state_(State::INIT_WAIT_ATI);
    } break;

    case State::INIT_WAIT_ATI: {
      if (this->rdy_read_() != 0)
        break;
      this->read_device_states_();
      this->i2c_stop_();
      if (this->device_states_.status.flag.ati_active) {
        ESP_LOGD(TAG, "ATI activated");
        this->set_next_state_delayed_(State::RUNTIME, 100);
        break;
      }
    } break;

    case State::RUNTIME: {
      bool new_touch_data_available{false};

      // RDY low = communication window opened
      if (this->test_mode_ || this->rdy_read_() == 0) {
        bool report_required{false};

        static uint32_t last_report{0};
        uint32_t now = millis();
        if (now - last_report > 1000) {
          last_report = now;
          report_required = true;
        }

        if (!this->test_mode_) {
          this->read_device_states_();
          this->i2c_stop_();

          if (this->device_states_.status.flag.reset) {
            ESP_LOGW(TAG, "Device reset detected in runtime. What to do?");
            // this->set_next_state_(State::ACK_RESET);
            // break;
          }
        }

        if (this->device_states_prev_.touch.value != this->device_states_.touch.value) {
          ESP_LOGD(TAG, "Touch event: %d", this->device_states_.touch.value);
          new_touch_data_available = true;
        }

        // if (this->device_states_prev_.status.value != this->device_states_.status.value) {
        //   ESP_LOGD(TAG, "Status event: %d (ati_active=%d, ati_err=%d, reset=%d, power=%d, np=%d, halt=%d",
        //            this->device_states_.status.value, this->device_states_.status.flag.ati_active,
        //            this->device_states_.status.flag.ati_error, this->device_states_.status.flag.reset,
        //            this->device_states_.status.flag.power_mode, this->device_states_.status.flag.np_update,
        //            this->device_states_.status.flag.global_hal);
        // }

        if (new_touch_data_available || report_required) {
          //  ESP_LOGD(TAG, "New touch data available");
          //  this->i2c_stop_and_delay_();
          WAIT_FOR_RDY_WINDOW();
          this->read_touch_counts_();
          this->i2c_stop_();

          new_touch_data_available = false;
          this->process_touch_data_();
        }
        this->device_states_prev_.touch.value = this->device_states_.touch.value;
        this->device_states_prev_.status.value = this->device_states_.status.value;
      }
    } break;
    default:
      this->mark_failed();
  }
}

/*
  (BUTTON_0_ENTER_EXIT << 8 | BUTTON_0_PROX_THRESHOLD),
  (BUTTON_0_TOUCH_HYSTERESIS << 8 | BUTTON_0_TOUCH_THRESHOLD),
  (BUTTON_0_TOUCH_EVENT_TIMEOUT << 8 | BUTTON_0_PROX_EVENT_TIMEOUT)},
*/
void IQS7222CComponent::set_button_config(uint8_t nr, uint8_t prox_thr, uint8_t enter_exit, uint8_t touch_thr,
                                          uint8_t touch_hy, uint8_t prox_ev_tm, uint8_t touch_ev_tm) {
  if (nr < IQS7222C_MAX_BUTTONS) {
    button_setup[nr][1] = encode_uint16(enter_exit, prox_thr);
    button_setup[nr][2] = encode_uint16(touch_hy, touch_thr);
    button_setup[nr][3] = encode_uint16(touch_ev_tm, prox_ev_tm);
  }
};

void IQS7222CComponent::set_channel_config(uint8_t nr, uint16_t setup, uint16_t ati_settings, uint16_t multipliers,
                                           uint16_t ati_compensation, uint16_t ref_ptr, uint16_t refmask) {
  if (nr < IQS7222C_MAX_CHANNELS) {
    ch_setup[nr][1] = setup;
    ch_setup[nr][2] = ati_settings;
    ch_setup[nr][3] = multipliers;
    ch_setup[nr][4] = ati_compensation;
    ch_setup[nr][5] = ref_ptr;
    ch_setup[nr][6] = refmask;
  }
}

void IQS7222CComponent::set_cycle_config(uint8_t nr, uint8_t freq_frac, uint8_t freq_period, uint8_t settings,
                                         uint8_t ctx_select, uint8_t iref_0, uint8_t iref_1) {
  /*
   (CYCLE_0_CONV_FREQ_PERIOD << 8 | CYCLE_0_CONV_FREQ_FRAC),
   (CYCLE_0_CTX_SELECT << 8 | CYCLE_0_SETTINGS),
   (CYCLE_0_IREF_1 | CYCLE_0_IREF_0)
                                        */
  if (nr < 5) {
    cycle_setup[nr][1] = encode_uint16(freq_period, freq_frac);
    cycle_setup[nr][2] = encode_uint16(ctx_select, settings);
    cycle_setup[nr][3] = encode_uint16(iref_1, iref_0);
  }
}

void IQS7222CComponent::set_global_cycle_config(uint8_t setup_0, uint8_t setup_1, uint8_t coarse_divider_preload,
                                                uint8_t fine_divider_preload, uint8_t compensation_preload_0,
                                                uint8_t compensation_preload_1) {
  global_cycle_setup[1] = encode_uint16(setup_1, setup_0);
  global_cycle_setup[2] = encode_uint16(fine_divider_preload, coarse_divider_preload);
  global_cycle_setup[3] = encode_uint16(compensation_preload_1, compensation_preload_0);
}

void IQS7222CComponent::register_button(IQS7222CButton *button) {
  if (button->get_channel() < IQS7222C_MAX_BUTTONS) {
    this->buttons[button->get_channel()].push_back(button);
  }
}

void IQS7222CComponent::emulate_touch(uint8_t btn) {
  if (btn >= IQS7222C_MAX_BUTTONS)
    return;

  for (auto *button : this->buttons[btn]) {
    ESP_LOGD(TAG, "Emulate: Button %d touched", btn);
    button->publish_state(true);
  }

  this->set_timeout(50, [this, btn]() {
    for (auto *button : this->buttons[btn]) {
      ESP_LOGD(TAG, "Emulate: Button %d released", btn);
      button->publish(false);
    }
  });
}

/***********************************************************************************/

void IQS7222CComponent::report_state_() {
  if (this->last_reported_state_ != this->state_) {
    this->last_reported_state_ = this->state_;
    switch (this->state_) {
      case State::NOT_INITIALIZED:
        ESP_LOGD(TAG, "State::NOT_INITIALIZED");
        break;
      case State::WAIT:
        ESP_LOGD(TAG, "State::WAIT");
        break;
      case State::INIT_HARD_RESET:
        ESP_LOGD(TAG, "State::INIT_HARD_RESET");
        break;
      case State::INIT_WAIT_MCLR:
        ESP_LOGD(TAG, "State::INIT_WAIT_MCLR");
        break;
      case State::INIT_READ_PRODUCT_VERSION:
        ESP_LOGD(TAG, "State::INIT_READ_PRODUCT_VERSION");
        break;
      case State::INIT_SOFT_RESET:
        ESP_LOGD(TAG, "State::INIT_SOFT_RESET");
        break;
      case State::INIT_WRITE_SETTINGS:
        ESP_LOGD(TAG, "State::INIT_WRITE_SETTINGS");
        break;
      case State::INIT_READ_STATE:
        ESP_LOGD(TAG, "State::INIT_READ_STATE");
        break;
      case State::ACK_RESET:
        ESP_LOGD(TAG, "State::ACK_RESET");
        break;
      case State::INIT_ATI:
        ESP_LOGD(TAG, "State::INIT_ATI");
        break;
      case State::INIT_WAIT_ATI:
        ESP_LOGD(TAG, "State::INIT_WAIT_ATI");
        break;
      case State::RUNTIME:
        ESP_LOGD(TAG, "State::RUNTIME");
        break;
      default:
        ESP_LOGE(TAG, "Unknown state: %u", (uint8_t) this->state_);
    }
  }
}

inline void IQS7222CComponent::set_next_state_(State state) { this->state_ = state; }

void IQS7222CComponent::set_next_state_delayed_(State state, uint32_t delay_ms) {
  this->state_ = State::WAIT;
  this->set_timeout(delay_ms, [this, state]() { this->state_ = state; });
}

void IQS7222CComponent::write_settings_() {
  ESP_LOGV(TAG, "write_settings_()");

  if (1) {
    const bool RESTART = false;

    uint8_t transferBytes[30];

    WAIT_FOR_RDY_WINDOW();

    /* Change the Cycle Setup */
    /* Memory Map Position 0x8000 - 0x8403 */
    transferBytes[0] = CYCLE_0_CONV_FREQ_FRAC;
    transferBytes[1] = CYCLE_0_CONV_FREQ_PERIOD;
    transferBytes[2] = CYCLE_0_SETTINGS;
    transferBytes[3] = CYCLE_0_CTX_SELECT;
    transferBytes[4] = CYCLE_0_IREF_0;
    transferBytes[5] = CYCLE_0_IREF_1;
    transferBytes[6] = CYCLE_1_CONV_FREQ_FRAC;
    transferBytes[7] = CYCLE_1_CONV_FREQ_PERIOD;
    transferBytes[8] = CYCLE_1_SETTINGS;
    transferBytes[9] = CYCLE_1_CTX_SELECT;
    transferBytes[10] = CYCLE_1_IREF_0;
    transferBytes[11] = CYCLE_1_IREF_1;
    transferBytes[12] = CYCLE_2_CONV_FREQ_FRAC;
    transferBytes[13] = CYCLE_2_CONV_FREQ_PERIOD;
    transferBytes[14] = CYCLE_2_SETTINGS;
    transferBytes[15] = CYCLE_2_CTX_SELECT;
    transferBytes[16] = CYCLE_2_IREF_0;
    transferBytes[17] = CYCLE_2_IREF_1;
    transferBytes[18] = CYCLE_3_CONV_FREQ_FRAC;
    transferBytes[19] = CYCLE_3_CONV_FREQ_PERIOD;
    transferBytes[20] = CYCLE_3_SETTINGS;
    transferBytes[21] = CYCLE_3_CTX_SELECT;
    transferBytes[22] = CYCLE_3_IREF_0;
    transferBytes[23] = CYCLE_3_IREF_1;
    transferBytes[24] = CYCLE_4_CONV_FREQ_FRAC;
    transferBytes[25] = CYCLE_4_CONV_FREQ_PERIOD;
    transferBytes[26] = CYCLE_4_SETTINGS;
    transferBytes[27] = CYCLE_4_CTX_SELECT;
    transferBytes[28] = CYCLE_4_IREF_0;
    transferBytes[29] = CYCLE_4_IREF_1;
    this->write_register16(0x8000, transferBytes, 30);
    // Serial.println("\t\t1. Write Cycle Settings");

    WAIT_FOR_RDY_WINDOW();

    /* Change the Global Cycle Setup */
    /* Memory Map Position 0x8500 - 0x8502 */
    transferBytes[0] = GLOBAL_CYCLE_SETUP_0;
    transferBytes[1] = GLOBAL_CYCLE_SETUP_1;
    transferBytes[2] = COARSE_DIVIDER_PRELOAD;
    transferBytes[3] = FINE_DIVIDER_PRELOAD;
    transferBytes[4] = COMPENSATION_PRELOAD_0;
    transferBytes[5] = COMPENSATION_PRELOAD_1;
    this->write_register16(0x8500, transferBytes, 6);
    // Serial.println("\t\t2. Write Global Cycle Settings");
    WAIT_FOR_RDY_WINDOW();

    /* Change the Button Setup 0 - 4 */
    /* Memory Map Position 0x9000 - 0x9502 */
    transferBytes[0] = BUTTON_0_PROX_THRESHOLD;
    transferBytes[1] = BUTTON_0_ENTER_EXIT;
    transferBytes[2] = BUTTON_0_TOUCH_THRESHOLD;
    transferBytes[3] = BUTTON_0_TOUCH_HYSTERESIS;
    transferBytes[4] = BUTTON_0_PROX_EVENT_TIMEOUT;
    transferBytes[5] = BUTTON_0_TOUCH_EVENT_TIMEOUT;
    transferBytes[6] = BUTTON_1_PROX_THRESHOLD;
    transferBytes[7] = BUTTON_1_ENTER_EXIT;
    transferBytes[8] = BUTTON_1_TOUCH_THRESHOLD;
    transferBytes[9] = BUTTON_1_TOUCH_HYSTERESIS;
    transferBytes[10] = BUTTON_1_PROX_EVENT_TIMEOUT;
    transferBytes[11] = BUTTON_1_TOUCH_EVENT_TIMEOUT;
    transferBytes[12] = BUTTON_2_PROX_THRESHOLD;
    transferBytes[13] = BUTTON_2_ENTER_EXIT;
    transferBytes[14] = BUTTON_2_TOUCH_THRESHOLD;
    transferBytes[15] = BUTTON_2_TOUCH_HYSTERESIS;
    transferBytes[16] = BUTTON_2_PROX_EVENT_TIMEOUT;
    transferBytes[17] = BUTTON_2_TOUCH_EVENT_TIMEOUT;
    transferBytes[18] = BUTTON_3_PROX_THRESHOLD;
    transferBytes[19] = BUTTON_3_ENTER_EXIT;
    transferBytes[20] = BUTTON_3_TOUCH_THRESHOLD;
    transferBytes[21] = BUTTON_3_TOUCH_HYSTERESIS;
    transferBytes[22] = BUTTON_3_PROX_EVENT_TIMEOUT;
    transferBytes[23] = BUTTON_3_TOUCH_EVENT_TIMEOUT;
    transferBytes[24] = BUTTON_4_PROX_THRESHOLD;
    transferBytes[25] = BUTTON_4_ENTER_EXIT;
    transferBytes[26] = BUTTON_4_TOUCH_THRESHOLD;
    transferBytes[27] = BUTTON_4_TOUCH_HYSTERESIS;
    transferBytes[28] = BUTTON_4_PROX_EVENT_TIMEOUT;
    transferBytes[29] = BUTTON_4_TOUCH_EVENT_TIMEOUT;
    this->write_register16(0x9000, transferBytes, 30);
    WAIT_FOR_RDY_WINDOW();
    // Serial.println("\t\t3. Write Button Settings 0 - 4");

    /* Change the Button Setup 5 - 9 */
    /* Memory Map Position 0x9500 - 0x9902 */
    transferBytes[0] = BUTTON_5_PROX_THRESHOLD;
    transferBytes[1] = BUTTON_5_ENTER_EXIT;
    transferBytes[2] = BUTTON_5_TOUCH_THRESHOLD;
    transferBytes[3] = BUTTON_5_TOUCH_HYSTERESIS;
    transferBytes[4] = BUTTON_5_PROX_EVENT_TIMEOUT;
    transferBytes[5] = BUTTON_5_TOUCH_EVENT_TIMEOUT;
    transferBytes[6] = BUTTON_6_PROX_THRESHOLD;
    transferBytes[7] = BUTTON_6_ENTER_EXIT;
    transferBytes[8] = BUTTON_6_TOUCH_THRESHOLD;
    transferBytes[9] = BUTTON_6_TOUCH_HYSTERESIS;
    transferBytes[10] = BUTTON_6_PROX_EVENT_TIMEOUT;
    transferBytes[11] = BUTTON_6_TOUCH_EVENT_TIMEOUT;
    transferBytes[12] = BUTTON_7_PROX_THRESHOLD;
    transferBytes[13] = BUTTON_7_ENTER_EXIT;
    transferBytes[14] = BUTTON_7_TOUCH_THRESHOLD;
    transferBytes[15] = BUTTON_7_TOUCH_HYSTERESIS;
    transferBytes[16] = BUTTON_7_PROX_EVENT_TIMEOUT;
    transferBytes[17] = BUTTON_7_TOUCH_EVENT_TIMEOUT;
    transferBytes[18] = BUTTON_8_PROX_THRESHOLD;
    transferBytes[19] = BUTTON_8_ENTER_EXIT;
    transferBytes[20] = BUTTON_8_TOUCH_THRESHOLD;
    transferBytes[21] = BUTTON_8_TOUCH_HYSTERESIS;
    transferBytes[22] = BUTTON_8_PROX_EVENT_TIMEOUT;
    transferBytes[23] = BUTTON_8_TOUCH_EVENT_TIMEOUT;
    transferBytes[24] = BUTTON_9_PROX_THRESHOLD;
    transferBytes[25] = BUTTON_9_ENTER_EXIT;
    transferBytes[26] = BUTTON_9_TOUCH_THRESHOLD;
    transferBytes[27] = BUTTON_9_TOUCH_HYSTERESIS;
    transferBytes[28] = BUTTON_9_PROX_EVENT_TIMEOUT;
    transferBytes[29] = BUTTON_9_TOUCH_EVENT_TIMEOUT;
    this->write_register16(0x9500, transferBytes, 30);
    WAIT_FOR_RDY_WINDOW();
    // Serial.println("\t\t4. Write Button Settings 5 - 9");

    /* Change the CH0 Setup */
    /* Memory Map Position 0xA000 - 0xA005 */
    transferBytes[0] = CH0_SETUP_0;
    transferBytes[1] = CH0_SETUP_1;
    transferBytes[2] = CH0_ATI_SETTINGS_0;
    transferBytes[3] = CH0_ATI_SETTINGS_1;
    transferBytes[4] = CH0_MULTIPLIERS_0;
    transferBytes[5] = CH0_MULTIPLIERS_1;
    transferBytes[6] = CH0_ATI_COMPENSATION_0;
    transferBytes[7] = CH0_ATI_COMPENSATION_1;
    transferBytes[8] = CH0_REF_PTR_0;
    transferBytes[9] = CH0_REF_PTR_1;
    transferBytes[10] = CH0_REFMASK_0;
    transferBytes[11] = CH0_REFMASK_1;
    this->write_register16(0xA000, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    // Serial.println("\t\t5. Write Channel 0 Settings");

    /* Change the CH1 Setup */
    /* Memory Map Position 0xA100 - 0xA105 */
    transferBytes[0] = CH1_SETUP_0;
    transferBytes[1] = CH1_SETUP_1;
    transferBytes[2] = CH1_ATI_SETTINGS_0;
    transferBytes[3] = CH1_ATI_SETTINGS_1;
    transferBytes[4] = CH1_MULTIPLIERS_0;
    transferBytes[5] = CH1_MULTIPLIERS_1;
    transferBytes[6] = CH1_ATI_COMPENSATION_0;
    transferBytes[7] = CH1_ATI_COMPENSATION_1;
    transferBytes[8] = CH1_REF_PTR_0;
    transferBytes[9] = CH1_REF_PTR_1;
    transferBytes[10] = CH1_REFMASK_0;
    transferBytes[11] = CH1_REFMASK_1;
    this->write_register16(0xA100, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    // Serial.println("\t\t6. Write Channel 1 Settings");

    /* Change the CH2 Setup */
    /* Memory Map Position 0xA200 - 0xA205 */
    transferBytes[0] = CH2_SETUP_0;
    transferBytes[1] = CH2_SETUP_1;
    transferBytes[2] = CH2_ATI_SETTINGS_0;
    transferBytes[3] = CH2_ATI_SETTINGS_1;
    transferBytes[4] = CH2_MULTIPLIERS_0;
    transferBytes[5] = CH2_MULTIPLIERS_1;
    transferBytes[6] = CH2_ATI_COMPENSATION_0;
    transferBytes[7] = CH2_ATI_COMPENSATION_1;
    transferBytes[8] = CH2_REF_PTR_0;
    transferBytes[9] = CH2_REF_PTR_1;
    transferBytes[10] = CH2_REFMASK_0;
    transferBytes[11] = CH2_REFMASK_1;
    this->write_register16(0xA200, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    // Serial.println("\t\t7. Write Channel 2 Settings");

    /* Change the CH3 Setup */
    /* Memory Map Position 0xA300 - 0xA305 */
    transferBytes[0] = CH3_SETUP_0;
    transferBytes[1] = CH3_SETUP_1;
    transferBytes[2] = CH3_ATI_SETTINGS_0;
    transferBytes[3] = CH3_ATI_SETTINGS_1;
    transferBytes[4] = CH3_MULTIPLIERS_0;
    transferBytes[5] = CH3_MULTIPLIERS_1;
    transferBytes[6] = CH3_ATI_COMPENSATION_0;
    transferBytes[7] = CH3_ATI_COMPENSATION_1;
    transferBytes[8] = CH3_REF_PTR_0;
    transferBytes[9] = CH3_REF_PTR_1;
    transferBytes[10] = CH3_REFMASK_0;
    transferBytes[11] = CH3_REFMASK_1;
    this->write_register16(0xA300, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    // Serial.println("\t\t8. Write Channel 3 Settings");

    /* Change the CH4 Setup */
    /* Memory Map Position 0xA400 - 0xA405 */
    transferBytes[0] = CH4_SETUP_0;
    transferBytes[1] = CH4_SETUP_1;
    transferBytes[2] = CH4_ATI_SETTINGS_0;
    transferBytes[3] = CH4_ATI_SETTINGS_1;
    transferBytes[4] = CH4_MULTIPLIERS_0;
    transferBytes[5] = CH4_MULTIPLIERS_1;
    transferBytes[6] = CH4_ATI_COMPENSATION_0;
    transferBytes[7] = CH4_ATI_COMPENSATION_1;
    transferBytes[8] = CH4_REF_PTR_0;
    transferBytes[9] = CH4_REF_PTR_1;
    transferBytes[10] = CH4_REFMASK_0;
    transferBytes[11] = CH4_REFMASK_1;
    this->write_register16(0xa400, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    // Serial.println("\t\t9. Write Channel 4 Settings");

    /* Change the CH5 Setup */
    /* Memory Map Position 0xA500 - 0xA505 */
    transferBytes[0] = CH5_SETUP_0;
    transferBytes[1] = CH5_SETUP_1;
    transferBytes[2] = CH5_ATI_SETTINGS_0;
    transferBytes[3] = CH5_ATI_SETTINGS_1;
    transferBytes[4] = CH5_MULTIPLIERS_0;
    transferBytes[5] = CH5_MULTIPLIERS_1;
    transferBytes[6] = CH5_ATI_COMPENSATION_0;
    transferBytes[7] = CH5_ATI_COMPENSATION_1;
    transferBytes[8] = CH5_REF_PTR_0;
    transferBytes[9] = CH5_REF_PTR_1;
    transferBytes[10] = CH5_REFMASK_0;
    transferBytes[11] = CH5_REFMASK_1;
    this->write_register16(0xa500, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    // Serial.println("\t\t10. Write Channel 5 Settings");

    /* Change the CH6 Setup */
    /* Memory Map Position 0xA600 - 0xA605 */
    transferBytes[0] = CH6_SETUP_0;
    transferBytes[1] = CH6_SETUP_1;
    transferBytes[2] = CH6_ATI_SETTINGS_0;
    transferBytes[3] = CH6_ATI_SETTINGS_1;
    transferBytes[4] = CH6_MULTIPLIERS_0;
    transferBytes[5] = CH6_MULTIPLIERS_1;
    transferBytes[6] = CH6_ATI_COMPENSATION_0;
    transferBytes[7] = CH6_ATI_COMPENSATION_1;
    transferBytes[8] = CH6_REF_PTR_0;
    transferBytes[9] = CH6_REF_PTR_1;
    transferBytes[10] = CH6_REFMASK_0;
    transferBytes[11] = CH6_REFMASK_1;
    this->write_register16(0xA600, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t11. Write Channel 6 Settings");

    /* Change the CH7 Setup */
    /* Memory Map Position 0xA700 - 0xA705 */
    transferBytes[0] = CH7_SETUP_0;
    transferBytes[1] = CH7_SETUP_1;
    transferBytes[2] = CH7_ATI_SETTINGS_0;
    transferBytes[3] = CH7_ATI_SETTINGS_1;
    transferBytes[4] = CH7_MULTIPLIERS_0;
    transferBytes[5] = CH7_MULTIPLIERS_1;
    transferBytes[6] = CH7_ATI_COMPENSATION_0;
    transferBytes[7] = CH7_ATI_COMPENSATION_1;
    transferBytes[8] = CH7_REF_PTR_0;
    transferBytes[9] = CH7_REF_PTR_1;
    transferBytes[10] = CH7_REFMASK_0;
    transferBytes[11] = CH7_REFMASK_1;
    this->write_register16(0xA700, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t12. Write Channel 7 Settings");

    /* Change the CH8 Setup */
    /* Memory Map Position 0xA800 - 0xA805 */
    transferBytes[0] = CH8_SETUP_0;
    transferBytes[1] = CH8_SETUP_1;
    transferBytes[2] = CH8_ATI_SETTINGS_0;
    transferBytes[3] = CH8_ATI_SETTINGS_1;
    transferBytes[4] = CH8_MULTIPLIERS_0;
    transferBytes[5] = CH8_MULTIPLIERS_1;
    transferBytes[6] = CH8_ATI_COMPENSATION_0;
    transferBytes[7] = CH8_ATI_COMPENSATION_1;
    transferBytes[8] = CH8_REF_PTR_0;
    transferBytes[9] = CH8_REF_PTR_1;
    transferBytes[10] = CH8_REFMASK_0;
    transferBytes[11] = CH8_REFMASK_1;
    this->write_register16(0xA800, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t13. Write Channel 8 Settings");

    /* Change the CH9 Setup */
    /* Memory Map Position 0xA900 - 0xA905 */
    transferBytes[0] = CH9_SETUP_0;
    transferBytes[1] = CH9_SETUP_1;
    transferBytes[2] = CH9_ATI_SETTINGS_0;
    transferBytes[3] = CH9_ATI_SETTINGS_1;
    transferBytes[4] = CH9_MULTIPLIERS_0;
    transferBytes[5] = CH9_MULTIPLIERS_1;
    transferBytes[6] = CH9_ATI_COMPENSATION_0;
    transferBytes[7] = CH9_ATI_COMPENSATION_1;
    transferBytes[8] = CH9_REF_PTR_0;
    transferBytes[9] = CH9_REF_PTR_1;
    transferBytes[10] = CH9_REFMASK_0;
    transferBytes[11] = CH9_REFMASK_1;
    this->write_register16(0xA900, transferBytes, 12);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t14. Write Channel 9 Settings");

    /* Change the Filter Betas */
    /* Memory Map Position 0xAA00 - 0xAA01 */
    transferBytes[0] = COUNTS_BETA_FILTER;
    transferBytes[1] = LTA_BETA_FILTER;
    transferBytes[2] = LTA_FAST_BETA_FILTER;
    transferBytes[3] = RESERVED_FILTER_0;
    this->write_register16(0xAA00, transferBytes, 4);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t15. Write Filter Betas");

    /* Change the Slider/Wheel 0 Setup 0 & Delta Link */
    /* Memory Map Position 0xB000 - 0xB009 */
    transferBytes[0] = SLIDER0SETUP_GENERAL;
    transferBytes[1] = SLIDER0_LOWER_CAL;
    transferBytes[2] = SLIDER0_UPPER_CAL;
    transferBytes[3] = SLIDER0_BOTTOM_SPEED;
    transferBytes[4] = SLIDER0_TOPSPEED_0;
    transferBytes[5] = SLIDER0_TOPSPEED_1;
    transferBytes[6] = SLIDER0_RESOLUTION_0;
    transferBytes[7] = SLIDER0_RESOLUTION_1;
    transferBytes[8] = SLIDER0_ENABLE_MASK_0_7;
    transferBytes[9] = SLIDER0_ENABLE_MASK_8_9;
    transferBytes[10] = SLIDER0_ENABLESTATUSLINK_0;
    transferBytes[11] = SLIDER0_ENABLESTATUSLINK_1;
    transferBytes[12] = SLIDER0_DELTA0_0;
    transferBytes[13] = SLIDER0_DELTA0_1;
    transferBytes[14] = SLIDER0_DELTA1_0;
    transferBytes[15] = SLIDER0_DELTA1_1;
    transferBytes[16] = SLIDER0_DELTA2_0;
    transferBytes[17] = SLIDER0_DELTA2_1;
    transferBytes[18] = SLIDER0_DELTA3_0;
    transferBytes[19] = SLIDER0_DELTA3_1;
    this->write_register16(0xB000, transferBytes, 20);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t16. Slider/Wheel 0 Settings");

    /* Change the Slider/Wheel 1 Setup 0 */
    /* Memory Map Position 0xB100 - 0xB105 */
    transferBytes[0] = SLIDER1SETUP_GENERAL;
    transferBytes[1] = SLIDER1_LOWER_CAL;
    transferBytes[2] = SLIDER1_UPPER_CAL;
    transferBytes[3] = SLIDER1_BOTTOM_SPEED;
    transferBytes[4] = SLIDER1_TOPSPEED_0;
    transferBytes[5] = SLIDER1_TOPSPEED_1;
    transferBytes[6] = SLIDER1_RESOLUTION_0;
    transferBytes[7] = SLIDER1_RESOLUTION_1;
    transferBytes[8] = SLIDER1_ENABLE_MASK_0_7;
    transferBytes[9] = SLIDER1_ENABLE_MASK_8_9;
    transferBytes[10] = SLIDER1_ENABLESTATUSLINK_0;
    transferBytes[11] = SLIDER1_ENABLESTATUSLINK_1;
    transferBytes[12] = SLIDER1_DELTA0_0;
    transferBytes[13] = SLIDER1_DELTA0_1;
    transferBytes[14] = SLIDER1_DELTA1_0;
    transferBytes[15] = SLIDER1_DELTA1_1;
    transferBytes[16] = SLIDER1_DELTA2_0;
    transferBytes[17] = SLIDER1_DELTA2_1;
    transferBytes[18] = SLIDER1_DELTA3_0;
    transferBytes[19] = SLIDER1_DELTA3_1;
    this->write_register16(0xB100, transferBytes, 20);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t17. Slider/Wheel 1 Settings");

    /* Change the GPIO Settings */
    /* Memory Map Position 0xC000 - 0xC202 */
    transferBytes[0] = GPIO0_SETUP_0;
    transferBytes[1] = GPIO0_SETUP_1;
    transferBytes[2] = GPIO0_ENABLE_MASK_0_7;
    transferBytes[3] = GPIO0_ENABLE_MASK_8_9;
    transferBytes[4] = GPIO0_ENABLESTATUSLINK_0;
    transferBytes[5] = GPIO0_ENABLESTATUSLINK_1;
    transferBytes[6] = GPIO1_SETUP_0;
    transferBytes[7] = GPIO1_SETUP_1;
    transferBytes[8] = GPIO1_ENABLE_MASK_0_7;
    transferBytes[9] = GPIO1_ENABLE_MASK_8_9;
    transferBytes[10] = GPIO1_ENABLESTATUSLINK_0;
    transferBytes[11] = GPIO1_ENABLESTATUSLINK_1;
    transferBytes[12] = GPIO2_SETUP_0;
    transferBytes[13] = GPIO2_SETUP_1;
    transferBytes[14] = GPIO2_ENABLE_MASK_0_7;
    transferBytes[15] = GPIO2_ENABLE_MASK_8_9;
    transferBytes[16] = GPIO2_ENABLESTATUSLINK_0;
    transferBytes[17] = GPIO2_ENABLESTATUSLINK_1;
    this->write_register16(0xC000, transferBytes, 18);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t18. GPIO 0 Settings");

    /* Change the System Settings */
    /* Memory Map Position 0xD0 - 0xD9 */
    transferBytes[0] = SYSTEM_CONTROL_0;
    transferBytes[1] = SYSTEM_CONTROL_1;
    transferBytes[2] = ATI_ERROR_TIMEOUT_0;
    transferBytes[3] = ATI_ERROR_TIMEOUT_1;
    transferBytes[4] = ATI_REPORT_RATE_0;
    transferBytes[5] = ATI_REPORT_RATE_1;
    transferBytes[6] = NORMAL_MODE_TIMEOUT_0;
    transferBytes[7] = NORMAL_MODE_TIMEOUT_1;
    transferBytes[8] = NORMAL_MODE_REPORT_RATE_0;
    transferBytes[9] = NORMAL_MODE_REPORT_RATE_1;
    transferBytes[10] = LP_MODE_TIMEOUT_0;
    transferBytes[11] = LP_MODE_TIMEOUT_1;
    transferBytes[12] = LP_MODE_REPORT_RATE_0;
    transferBytes[13] = LP_MODE_REPORT_RATE_1;
    transferBytes[14] = ULP_MODE_TIMEOUT_0;
    transferBytes[15] = ULP_MODE_TIMEOUT_1;
    transferBytes[16] = ULP_MODE_REPORT_RATE_0;
    transferBytes[17] = ULP_MODE_REPORT_RATE_1;
    transferBytes[18] = TOUCH_PROX_EVENT_MASK;
    transferBytes[19] = POWER_ATI_EVENT_MASK;
    transferBytes[20] = I2CCOMMS_0;
    this->write_register(0xd0, transferBytes, 21);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t19. System Settings");

    /* Change the GPIO Override */
    /* Memory Map Position 0xDB - 0xDB */
    transferBytes[0] = GPIO_OVERRIDE;
    this->write_register(0xdb, transferBytes, 1);
    WAIT_FOR_RDY_WINDOW();
    //  Serial.println("\t\t20. GPIO Override");

    /* Change the Comms timeout setting */
    /* Memory Map Position 0xDC - 0xDC */
    transferBytes[0] = COMMS_TIMEOUT_0;
    transferBytes[1] = COMMS_TIMEOUT_1;
    this->write_register(0xdc, transferBytes, 2);
    //  Serial.println("\t\t21. Communication Timeout");
  } else {
    uint8_t i{0};

    WAIT_FOR_RDY_WINDOW();
    this->write_register16(0x8000, (uint8_t *) &cycle_setup[0][1], 6, false);
    this->write_register16(0x8100, (uint8_t *) &cycle_setup[1][1], 6, false);
    this->write_register16(0x8200, (uint8_t *) &cycle_setup[2][1], 6, false);
    this->write_register16(0x8300, (uint8_t *) &cycle_setup[3][1], 6, false);
    this->write_register16(0x8400, (uint8_t *) &cycle_setup[4][1], 6, false);
    this->write_register16(0x8500, (uint8_t *) &global_cycle_setup[1], 6, true);
    this->i2c_stop_and_delay_();

    // this->write_register16(0x8000, (uint8_t *) &cycle_setup[0][1], 6, false);
    // this->write_register16(0x8100, (uint8_t *) &cycle_setup[1][1], 6, false);
    // this->write_register16(0x8200, (uint8_t *) &cycle_setup[2][1], 6, false);
    // this->write_register16(0x8300, (uint8_t *) &cycle_setup[3][1], 6, false);
    // this->write_register16(0x8400, (uint8_t *) &cycle_setup[4][1], 6, false);
    // this->write_register16(0x8500, (uint8_t *) &global_cycle_setup[1], 6, false);

    // this->i2c_write_cont_((uint8_t *) &cycle_setup_0, sizeof(cycle_setup_0));
    // this->i2c_write_cont_((uint8_t *) &cycle_setup_1, sizeof(cycle_setup_1));
    // this->i2c_write_cont_((uint8_t *) &cycle_setup_2, sizeof(cycle_setup_2));
    // this->i2c_write_cont_((uint8_t *) &cycle_setup_3, sizeof(cycle_setup_3));
    // this->i2c_write_cont_((uint8_t *) &cycle_setup_4, sizeof(cycle_setup_4));
    // // this->i2c_write_cont_((uint8_t *) &global_cycle_setup, sizeof(global_cycle_setup));

    // for (i = 0; i < IQS7222C_MAX_BUTTONS; i++) {
    //   //      WAIT_FOR_RDY_WINDOW();
    //   this->write_register16(0x9000 + i * 0x0100, (uint8_t *) &button_setup[i][1], 6, false);
    //   // this->i2c_write_cont_((uint8_t *) &button_setup[i], sizeof(button_setup[i]));
    // }
    WAIT_FOR_RDY_WINDOW();
    this->write_register16(0x9000, (uint8_t *) &button_setup[0][1], 6, false);
    this->write_register16(0x9100, (uint8_t *) &button_setup[1][1], 6, false);
    this->write_register16(0x9200, (uint8_t *) &button_setup[2][1], 6, false);
    this->write_register16(0x9300, (uint8_t *) &button_setup[3][1], 6, false);
    this->write_register16(0x9400, (uint8_t *) &button_setup[4][1], 6, false);
    this->write_register16(0x9500, (uint8_t *) &button_setup[5][1], 6, false);
    this->write_register16(0x9600, (uint8_t *) &button_setup[6][1], 6, false);
    this->write_register16(0x9700, (uint8_t *) &button_setup[7][1], 6, false);
    this->write_register16(0x9800, (uint8_t *) &button_setup[8][1], 6, false);
    this->write_register16(0x9900, (uint8_t *) &button_setup[9][1], 6, true);

    this->i2c_stop_();

    WAIT_FOR_RDY_WINDOW();
    this->write_register16(0xA000, (uint8_t *) &ch_setup[0][1], 6, false);
    this->write_register16(0xA100, (uint8_t *) &ch_setup[1][1], 6, false);
    this->write_register16(0xA200, (uint8_t *) &ch_setup[2][1], 6, false);
    this->write_register16(0xA300, (uint8_t *) &ch_setup[3][1], 6, false);
    this->write_register16(0xA400, (uint8_t *) &ch_setup[4][1], 6, false);
    this->write_register16(0xA500, (uint8_t *) &ch_setup[5][1], 6, false);
    this->write_register16(0xA600, (uint8_t *) &ch_setup[6][1], 6, false);
    this->write_register16(0xA700, (uint8_t *) &ch_setup[7][1], 6, false);
    this->write_register16(0xA800, (uint8_t *) &ch_setup[8][1], 6, false);
    this->write_register16(0xA900, (uint8_t *) &ch_setup[9][1], 6, true);

    this->i2c_stop_();

    for (i = 0; i < IQS7222C_MAX_CHANNELS; i++) {
      ESP_LOGD(TAG, "write_settings_() channel %d (0x%04X, 0=%02x,1=%02x) [1]=0x%04X, %d bytes", i, ch_setup[i][0],
               ((uint8_t *) ch_setup[i])[0], ((uint8_t *) ch_setup[i])[1], ch_setup[i][1], sizeof(ch_setup[i]));
    }

    ESP_LOGD(TAG, "before betas");
    WAIT_FOR_RDY_WINDOW();
    this->write_register16(0xAA00, (uint8_t *) &filter_betas[1], 4, false);
    this->write_register16(0xB000, (uint8_t *) &slider_wheel_setup_0[1], 20, false);
    this->write_register16(0xB100, (uint8_t *) &slider_wheel_setup_1[1], 20, false);
    this->write_register16(0xC000, (uint8_t *) &gpio_setting_0[1], 6, false);
    this->write_register16(0xC100, (uint8_t *) &gpio_setting_1[1], 6, false);
    this->write_register16(0xC200, (uint8_t *) &gpio_setting_2[1], 6, false);
    this->write_register(0xD0, (uint8_t *) &pmu_sys_setting[1], sizeof(pmu_sys_setting) - 1, true);

    this->i2c_stop_and_delay_();
  }
  // uint16_t ch_setup_reg = 0x00A0;
  uint16_t ch_setup_reg2 = 0xA000;
  // uint16_t data = 0;
  uint16_t data2 = 0;

  for (uint16_t ch_num = 0; ch_num < 10; ch_num++) {
    //   ch_setup_reg = 0x00A0 + ch_num;
    //   data = 0;

    ch_setup_reg2 = 0xA000 + ch_num * 0x0100;
    //   data2 = 0;

    //   WAIT_FOR_RDY_WINDOW();
    //   // this->i2c_read_registers_((uint8_t *) &ch_setup_reg, 2, (uint8_t *) &data, 2);
    //   // this->read_register16(ch_setup_reg2, (uint8_t *) &data2, 2, false);
    //   this->read_register((uint8_t) ch_setup_reg, (uint8_t *) &data2, 2);
    //   ESP_LOGD(TAG, "iqs7222c channel config [%d] 0x%04X", ch_num, data2);
    //   //    this->i2c_stop_and_delay_();
    // }
    WAIT_FOR_RDY_WINDOW();
    this->read_register16(ch_setup_reg2, (uint8_t *) &data2, 2, false);
    ESP_LOGD(TAG, "iqs7222c channel config [%u] 0x%04X", ch_num, data2);
  }
}

void IQS7222CComponent::read_product_version_() {
  uint8_t reg_addr = IQS_7222C_VERSION_REG;

  this->i2c_read_registers_(&reg_addr, 1, product_version_.data, sizeof(product_version_.data));
  ESP_LOGD(TAG, "read_product_version_. Product Num = %d, Major = %d, Minor = %d", product_version_.product_nr,
           product_version_.ver_major, product_version_.ver_minor);
}

/**
 * @brief Read system status, and event states. (Address 0x10 - 0x13)
 * @note Incuding Proxmity events, touch events
 */
void IQS7222CComponent::read_device_states_() {
  uint8_t reg_addr = IQS_7222C_SYSTEM_STATES_REG;

  this->i2c_read_registers_(&reg_addr, 1, (uint8_t *) &this->device_states_, sizeof(iqs_7222c_states_t));

  if (this->device_states_.touch.value == 0xEEEE || this->device_states_.touch.value == 0xFFEE ||
      this->device_states_.touch.value == 0xEEFF || this->device_states_.touch.value == 0xFFFF) {
    this->device_states_.touch.value = 0;
  }
}

void IQS7222CComponent::read_touch_event_() {
  uint8_t reg_addr = IQS_7222C_TOUCH_EVENT_STATES_REG;

  this->i2c_read_registers_(&reg_addr, 1, (uint8_t *) &device_states_.touch, sizeof(device_states_.touch));

  /* If the sensor isn't ready to read data, it returns 0xEEEE */
  if (device_states_.touch.value == 0xEEEE) {
    this->i2c_stop_();
    device_states_.touch.value = 0;
  }
}

void IQS7222CComponent::read_touch_counts_() {
  uint16_t counts[IQS7222C_MAX_CHANNELS] = {0};
  uint16_t lta[IQS7222C_MAX_CHANNELS] = {0};

  uint8_t reg_addr = IQS_7222C_CHANNEL_0_COUNTS;
  this->i2c_read_registers_(&reg_addr, 1, (uint8_t *) &counts, IQS7222C_MAX_CHANNELS * 2);
  this->i2c_stop_();

  WAIT_FOR_RDY_WINDOW();
  reg_addr = IQS_7222C_CHANNEL_0_LTA;
  this->i2c_read_registers_(&reg_addr, 1, (uint8_t *) &lta, IQS7222C_MAX_CHANNELS * 2);

  // ESP_LOGD(TAG, "counts ch0-ch9 %5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d", counts[0], counts[1], counts[2], counts[3],
  //          counts[4], counts[5], counts[6], counts[7], counts[8], counts[9]);
  // ESP_LOGD(TAG, "lta    ch0-ch9 %5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d", lta[0], lta[1], lta[2], lta[3], lta[4],
  //          lta[5], lta[6], lta[7], lta[8], lta[9]);
}

void IQS7222CComponent::process_touch_data_() {
  for (auto btn = 0; btn < IQS7222C_MAX_BUTTONS; btn++) {
    uint16_t touch_bit = (uint16_t) 1 << btn;
    bool touched_now = (this->device_states_.touch.value & touch_bit) != 0;
    bool touched_before = (this->device_states_prev_.touch.value & touch_bit) != 0;

    if (touched_now && !touched_before) {
      ESP_LOGD(TAG, "Button %d touched", btn);
    } else if (touched_before && !touched_now) {
      ESP_LOGD(TAG, "Button %d released", btn);
    }

    if (touched_now != touched_before) {
      for (auto *button : this->buttons[btn]) {
        button->publish(touched_now);
      }
    }
  }
}

inline uint8_t IQS7222CComponent::rdy_read_() { return this->rdy_pin_->digital_read(); }

inline void IQS7222CComponent::mclr_set_() { this->mclr_pin_->digital_write(false); }

inline void IQS7222CComponent::mclr_clear_() { this->mclr_pin_->digital_write(true); }

inline void IQS7222CComponent::i2c_stop_() {
  // uint16_t stop_byte = 0x00FF;
  // system_i2c_master_write_slave(IQS_7222C_I2C_NUM, IQS_SLAVE_ADDRESS, (uint8_t*) &stop_byte, 2, true);

  this->bus_->write(address_, (uint8_t *) &stop_byte, 2, true);
}

inline void IQS7222CComponent::i2c_stop_and_delay_() {
  this->i2c_stop_();
  delay(20);
}

inline void IQS7222CComponent::i2c_write_(uint8_t *data, uint16_t data_len) {
  //    system_i2c_master_write_slave(IQS_7222C_I2C_NUM, IQS_SLAVE_ADDRESS, data, data_len, true);
  this->bus_->write(address_, data, data_len, true);
}

inline void IQS7222CComponent::i2c_write_cont_(uint8_t *data, uint16_t data_len) {
  //    system_i2c_master_write_slave(IQS_7222C_I2C_NUM, IQS_SLAVE_ADDRESS, data, data_len, false);

  this->bus_->write(address_, data, data_len, false);
}

inline void IQS7222CComponent::i2c_read_(uint8_t *data, uint16_t data_len) {
  //     system_i2c_master_read_slave(IQS_7222C_I2C_NUM, IQS_SLAVE_ADDRESS, data, data_len);
  this->bus_->read(address_, data, data_len);
}

void IQS7222CComponent::i2c_read_registers_(uint8_t *addr, uint16_t addr_size, uint8_t *data, uint16_t data_len) {
  // esp_err_t err;

  // err = system_i2c_master_write_to_read_slave(IQS_7222C_I2C_NUM, IQS_SLAVE_ADDRESS, addr, addr_size, data, data_len);

  // /* If the sensor is not ready to read data, it returns 0xEEEE */
  // if (err != ESP_OK)
  // {
  //     ESP_LOGI(TAG, "err %d %s", err, esp_err_to_name(err));
  //     memset(data,0,data_len);
  //     iqs_7222c_i2c_stop();
  //     vTaskDelay(20/portTICK_PERIOD_MS);
  //     /* Try to ready again */
  //     err = system_i2c_master_write_to_read_slave(IQS_7222C_I2C_NUM, IQS_SLAVE_ADDRESS, addr, addr_size, data,
  //     data_len);
  // }

  i2c::ErrorCode err;
  if (addr_size == 2) {
    err = this->read_register16(*((uint16_t *) addr), data, data_len, false);
  } else {
    err = this->read_register(*addr, data, data_len, false);
  }

  /* If the sensor is not ready to read data, it returns 0xEEEE */
  if (err != i2c::ErrorCode::ERROR_OK) {
    ESP_LOGI(TAG, "err %d %s", err, esp_err_to_name(err));
    ESP_LOGD(TAG, "i2c_read_registers_. err=  %d", err);
    memset(data, 0, data_len);
    this->i2c_stop_and_delay_();
    if (addr_size == 2) {
      err = this->read_register16(*((uint16_t *) addr), data, data_len, false);
    } else {
      err = this->read_register(*addr, data, data_len, false);
    }
  }
}
}  // namespace iqs7222c
}  // namespace esphome
