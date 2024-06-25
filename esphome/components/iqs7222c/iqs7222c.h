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

struct IQS7222CStore {
  volatile bool deviceRDY{false};
  ISRInternalGPIOPin rdy_pin;
  static void gpio_intr(IQS7222CStore *store);
};

class IQS7222CChannel : public binary_sensor::BinarySensor {
 public:
  void set_channel(uint8_t channel) { channel_ = channel; }
  const uint8_t get_channel() const { return channel_; }
  void publish(bool state);

 protected:
  uint8_t channel_{0};
};

class IQS7222CComponent : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void loop() override;

  void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin_ = pin; }
  void set_mclr_pin(GPIOPin *mclr_pin) { this->mclr_pin_ = mclr_pin; }

  void register_channel(IQS7222CChannel *channel) {
    if (channel->get_channel() < IQS7222C_MAX_BUTTONS) {
      this->channels[channel->get_channel()].push_back(channel);
    };
  }
  void emulate_touch(uint8_t btn);

 protected:
  GPIOPin *mclr_pin_{nullptr};
  InternalGPIOPin *interrupt_pin_{nullptr};
  void attach_interrupt_(InternalGPIOPin *irq_pin, esphome::gpio::InterruptType type);
  void clear_rdy_interrupt_(void);
  bool get_rdy_interrupt_(void);
  IQS7222CStore store_;

  bool new_data_available{false};
  bool iqs_7222c_initialized{false};

  iqs_7222c_touch_event_t iqs_7222c_touch_evt;
  iqs_7222c_states_t iqs_7222c_states;

  std::vector<IQS7222CChannel *> channels[IQS7222C_MAX_BUTTONS];

  //////////////////
  // Main functions
  //////////////////

  void iqs_7222c_init(void);

  void iqs_7222c_set_rdy_state(bool active);
  bool iqs_7222c_get_rdy_pin_active(void);

  void iqs_7222c_read_version(void);
  void iqs_7222c_read_state(iqs_7222c_states_t *state);
  void iqs_7222c_read_touch_event(void);

  /////////////////
  // HAL functions
  /////////////////

  void iqs_7222c_delay(uint32_t ms);
  void iqs_7222c_hal_init(void);

  uint8_t iqs_7222c_rdy_read(void);

  void iqs_7222c_mclr_set(void);
  void iqs_7222c_mclr_clear(void);

  void iqs_7222c_i2c_stop(void);

  void iqs_7222c_i2c_write(uint8_t *data, uint16_t data_len);
  void iqs_7222c_i2c_write_cont(uint8_t *data, uint16_t data_len);
  void iqs_7222c_i2c_read(uint8_t *data, uint16_t data_len);
  void iqs_7222c_i2c_read_registers(uint8_t *addr, uint16_t addr_size, uint8_t *data, uint16_t data_len);

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
  } error_code_{NONE};
};

}  // namespace iqs7222c
}  // namespace esphome
