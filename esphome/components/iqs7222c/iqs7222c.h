#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include "iqs7222c_registers.h"

#include <vector>

namespace esphome {
namespace iqs7222c {

const uint8_t IQS7222C_MAX_BUTTONS = 10;
const uint8_t IQS7222C_MAX_CHANNELS = 10;
const uint16_t IQS7222C_PRODUCT_NR = 863;

class IQS7222CButton : public binary_sensor::BinarySensor {
 public:
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  uint8_t get_channel() const { return this->channel_; }
  void publish(bool state);

 protected:
  uint8_t channel_{0};
};

class IQS7222CComponent : public Component, public i2c::I2CDevice {
 public:
  //
  // Standard EspHome functions
  //
  float get_setup_priority() const override;
  void setup() override;
  void dump_config() override;
  void loop() override;

  //
  // Setters for the yaml
  //
  void set_rdy_pin(InternalGPIOPin *rdy_pin) { this->rdy_pin_ = rdy_pin; }
  void set_mclr_pin(GPIOPin *mclr_pin) { this->mclr_pin_ = mclr_pin; }
  void set_enable_test_mode(bool enable_test_mode) { this->test_mode_ = enable_test_mode; }
  void set_init_delay_ms(uint16_t delay_ms) { this->init_delay_ms_ = delay_ms; }

  void set_button_config(uint8_t nr, uint8_t prox_thr, uint8_t enter_exit, uint8_t touch_thr, uint8_t touch_hy,
                         uint8_t prox_ev_tm, uint8_t touch_ev_tm);
  /*
#define CH0_SETUP_0 0xF3
#define CH0_SETUP_1 0x11
#define CH0_ATI_SETTINGS_0 0x3D
#define CH0_ATI_SETTINGS_1 0x64
#define CH0_MULTIPLIERS_0 0x4A
#define CH0_MULTIPLIERS_1 0x31
#define CH0_ATI_COMPENSATION_0 0xFC
#define CH0_ATI_COMPENSATION_1 0x61
#define CH0_REF_PTR_0 0x00
#define CH0_REF_PTR_1 0x00
#define CH0_REFMASK_0 0x00
#define CH0_REFMASK_1 0x00
*/
  void set_channel_config(uint8_t nr, uint8_t setup_0, uint8_t setup_1, uint8_t ati_settings_0, uint8_t ati_settings_1,
                          uint8_t multipliers_0, uint8_t multipliers_1, uint8_t ati_compensation_0,
                          uint8_t ati_compensation_1, uint8_t ref_ptr_0, uint8_t ref_ptr_1, uint8_t refmask_0,
                          uint8_t refmask_1);

  void register_button(IQS7222CButton *btn);

  void emulate_touch(uint8_t btn);

 protected:
  GPIOPin *mclr_pin_{nullptr};         // Master Clear pin - for resetting the device. Active low.
  InternalGPIOPin *rdy_pin_{nullptr};  // Ready pin - device is ready to communicate. Active low.

  bool test_mode_{false};
  uint32_t init_delay_ms_{0};

  union {
    uint8_t data[6];
    struct {
      uint16_t product_nr;
      uint16_t ver_major;
      uint16_t ver_minor;
    } __attribute__((packed));
  } product_version_;

  iqs_7222c_states_t device_states_;
  iqs_7222c_states_t device_states_prev_{0xffff, 0xffff, 0xffff, 0xffff};

  std::vector<IQS7222CButton *> buttons[IQS7222C_MAX_BUTTONS];

  //
  // Main functions
  //
  void init_device_();
  void write_settings_();
  void read_product_version_();
  void read_device_states_();
  void read_touch_event_();
  void read_touch_counts_();

  void process_touch_data_();

  //
  // Pin-level functions
  //
  inline uint8_t rdy_read_();
  inline void mclr_set_();
  inline void mclr_clear_();

  //
  // I2C direct access functions
  //
  inline void i2c_stop_();
  inline void i2c_stop_and_delay_();
  inline void i2c_write_(uint8_t *data, uint16_t data_len);
  inline void i2c_write_cont_(uint8_t *data, uint16_t data_len);
  inline void i2c_read_(uint8_t *data, uint16_t data_len);
  void i2c_read_registers_(uint8_t *addr, uint16_t addr_size, uint8_t *data, uint16_t data_len);

  //
  // FSM
  //
  enum class State : uint8_t {
    NOT_INITIALIZED = 0,
    WAIT,
    INIT_HARD_RESET,
    INIT_WAIT_MCLR,
    INIT_READ_PRODUCT_VERSION,
    INIT_SOFT_RESET,
    INIT_WRITE_SETTINGS,
    INIT_READ_STATE,
    RUNTIME,
  } state_{State::NOT_INITIALIZED}, last_reported_state_{State::NOT_INITIALIZED};

  void report_state_();

  inline void set_next_state_(State state);
  void set_next_state_delayed_(State state, uint32_t delay_ms);

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
  } error_code_{NONE};
};

}  // namespace iqs7222c
}  // namespace esphome
