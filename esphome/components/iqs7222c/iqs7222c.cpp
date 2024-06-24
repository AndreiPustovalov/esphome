#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "iqs7222c.h"
#include "iqs7222c_init_azoteq_ui.h"

namespace esphome {
namespace iqs7222c {

// Public Global Definitions
/* For use with Wire.h library. True argument with some functions closes the
   I2C communication window.*/
#define STOP true
/* For use with Wire.h library. False argument with some functions keeps the
   I2C communication window open. */
#define RESTART false

static const char *const TAG = "iqs7222c";

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

  this->mclr_pin_->setup();
  this->hard_reset_();
  // store_.iqs7222c_deviceRDY = false;//true;  // interrupts are not working yet
  // force_I2C_communication();

  iqs7222c_state.state = IQS7222C_STATE_NONE;            // IQS7222C_STATE_START;
  iqs7222c_state.init_state = IQS7222C_INIT_READ_RESET;  //  IQS7222C_INIT_VERIFY_PRODUCT;

  //   // Check if IQS7222C is actually connected
  //   this->read_byte(IQS7222C_PRODUCT_ID, &this->iqs7222c_product_id_);
  //   this->read_byte(IQS7222C_MANUFACTURE_ID, &this->iqs7222c_manufacture_id_);
  //   this->read_byte(IQS7222C_REVISION, &this->iqs7222c_revision_);

  //   if ((this->iqs7222c_product_id_ != 0x50) || (this->iqs7222c_manufacture_id_ != 0x5D)) {
  //     this->error_code_ = COMMUNICATION_FAILED;
  //     this->mark_failed();
  //     return;
  //   }

  //   // Set sensitivity
  //   uint8_t sensitivity = 0;
  //   this->read_byte(IQS7222C_SENSITVITY, &sensitivity);
  //   sensitivity = sensitivity & 0x0f;
  //   this->write_byte(IQS7222C_SENSITVITY, sensitivity | this->touch_threshold_);

  //   // Allow multiple touches
  //   this->write_byte(IQS7222C_MULTI_TOUCH, this->allow_multiple_touches_);

  //   // Have LEDs follow touches
  //   this->write_byte(IQS7222C_LED_LINK, 0xFF);

  //   // Speed up a bit
  //   this->write_byte(IQS7222C_STAND_BY_CONFIGURATION, 0x30);
}

void IQS7222CComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "IQS7222C:");
  LOG_I2C_DEVICE(this);
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

enum class State : uint8_t {
  NONE,
  POR,
  ACK_R,
  R_R,
  WRITE_SET,
  V0,
  V1,
  V2,

  RUN
};
static uint32_t last_rdy_window_number = 0;
static State state = State::NONE;

void IQS7222CComponent::loop() {
  if (first_run) {
    first_run = false;
    this->set_timeout(1 * 1000, [this]() {
      iqs7222c_state.state = IQS7222C_STATE_START;
      ESP_LOGD(TAG, "First run execution. publishing buttons");
      for (auto btn = 0; btn < IQS7222C_MAX_BUTTONS; btn++) {
        for (auto *channel : this->channels[btn]) {
          ESP_LOGD(TAG, "Button %d", btn);
          channel->publish_state(false);
        }
      }
    });
    this->set_timeout(12 * 1000, [this]() {
      iqs7222c_state.state = IQS7222C_STATE_START;
      ESP_LOGD(TAG, "First run execution. touch hard reset");
      this->hard_reset_();
      state = State::POR;
    });
    return;
  }

  //  this->run();
  this->run2();
  // force_I2C_communication();  // prompt the IQS7222C
  //   force_comms_and_reset(); // function to initialize a force communication window.
  /* Process data read from IQS7222C when new data is available (RDY Line Low) */
  if (new_data_available) {
    ESP_LOGD(TAG, "run() New data available");
    //    check_power_mode();       // Verify if a power mode change occurred
    //    read_slider_coordinates();// Read the latest slider coordinates
    publish_channel_states_();  // Check if a channel state change has occurred
                                //    show_iqs7222c_data();     // Show data if a forced communication window was
                                //    requested

    new_data_available = false;
  }
}

void IQS7222CComponent::publish_channel_states_() {
  // check button changes
  // button can be in 3 states: NONE, PROX, TOUCH,
  // PROX is not interesting for us, we only use TOUCH
  for (auto btn = 0; btn < IQS7222C_MAX_BUTTONS; btn++) {
    bool touched = channel_touchState((iqs7222c_channel_e) btn);
    if (touched) {
      ESP_LOGD(TAG, "Button %d touched", btn);
    }

    if (touched != (button_states[btn] == IQS7222C_CH_TOUCH)) {
      button_states[btn] = touched ? IQS7222C_CH_TOUCH : IQS7222C_CH_NONE;
      for (auto *channel : this->channels[btn]) {
        channel->publish_state(touched);
      }
    }
  }
}

void IQS7222CComponent::run2() {
  static uint16_t prod_num{0};
  static uint8_t ver_maj{0}, ver_min{0};
  if (!this->getRDYStatus())
    return;

  auto same_window = last_rdy_window_number == store_.rdy_window_number;

  if (!same_window) {
    last_rdy_window_number = store_.rdy_window_number;
    ESP_LOGD(TAG, "RDY %d, State %u", store_.rdy_window_number, (uint8_t) state);
  }
  switch (state) {
    case State::NONE:
      break;
    case State::POR: {
      updateInfoFlags(RESTART);
      if (checkReset()) {
        ESP_LOGD(TAG, "Reset event occurred.");
        state = State::ACK_R;
      } else {
        ESP_LOGD(TAG, " No Reset Event Detected - Request SW Reset");
        state = State::R_R;
      }
      break;

      case State::R_R: {
        SW_Reset(STOP);
        ESP_LOGD(TAG, "Software Reset requested.");
        delay(100);
        state = State::V0;
      } break;

      case State::ACK_R: {
        ESP_LOGD(TAG, "ACK_RESET.");
        acknowledgeReset(STOP);
        state = State::WRITE_SET;
      } break;

      case State::WRITE_SET: {
        ESP_LOGD(TAG, "Write MM.");
        writeMM(STOP);
        state = State::V0;
      } break;

      case State::V0: {
        uint8_t transferBytes[6];    // A temporary array to hold the byte to be transferred.
        uint8_t prodNumLow = 0;      // Temporary storage for the counts low byte.
        uint8_t prodNumHigh = 0;     // Temporary storage for the counts high byte.
        uint16_t prodNumReturn = 0;  // The 16-bit return value.

        /* Read the Device info from the IQS7222C. */
        readRandomBytes(IQS7222C_MM_PROD_NUM, 6, transferBytes, STOP);

        ESP_LOGD(TAG, "Product info data: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x", transferBytes[0],
                 transferBytes[1], transferBytes[2], transferBytes[3], transferBytes[4], transferBytes[5]);

        /* Construct the 16-bit return value. */
        prodNumLow = transferBytes[0];
        prodNumHigh = transferBytes[1];
        prodNumReturn = (uint16_t) (prodNumLow);
        prodNumReturn |= (uint16_t) (prodNumHigh << 8);

        state = State::RUN;
      } break;

        //        prod_num = getProductNum(RESTART);
        // ver_maj = getmajorVersion(RESTART);
        // ver_min = getminorVersion(STOP);
        //        ESP_LOGD(TAG, "1Product number is: %d v %d.%d", prod_num, ver_maj, ver_min);

        //        state = State::V0;

        // ver_maj = getmajorVersion(STOP);
        // // ver_min = getminorVersion(STOP);
        // ESP_LOGD(TAG, "2Product number is: %d v %d.%d", prod_num, ver_maj, ver_min);

        //      state = State::V1;
        //      break;
      case State::V1:
        ver_min = getminorVersion(STOP);
        ESP_LOGD(TAG, "3Product number is: %d v %d.%d", prod_num, ver_maj, ver_min);

        state = State::RUN;
        break;

      case State::RUN:
        if (same_window) {
          break;
        }
        break;
    }
  }
}

/* Private Global Variables */

// uint8_t iqs7222c_ready_pin;

/* Private Functions */
// void iqs7222c_ready_interrupt(void);

// /*****************************************************************************/
// /*                             PUBLIC METHODS                                */
// /*****************************************************************************/

// /**
//  * @name   begin
//  * @brief  A method to initialize the IQS7222C device with the device address
//  *         and ready pin specified by the user.
//  * @param  deviceAddress -> The address of the IQS7222C device.
//  * @param  readyPin      -> The Arduino pin which is connected to the ready
//  *                          pin of the IQS7222C device.
//  * @retval None.
//  * @note   - Receiving a true return value does not mean that initialization
//  *           was successful.
//  *         - Receiving a true return value only means that the IQS device
//  *           responded to the request for communication.
//  *         - Receiving a false return value means that initialization did not
//  *           take place at all.
//  *         - If communication is successfully established then it is unlikely
//  *           that initialization will fail.
//  */
// void IQS7222C::begin(uint8_t deviceAddressIn, uint8_t readyPinIn) {
//   // Initialize I2C communication here, since this library can't function without it.
//   Wire.begin();
//   Wire.setClock(400000);

//   _deviceAddress = deviceAddressIn;
//   iqs7222c_ready_pin = readyPinIn;
//   attachInterrupt(digitalPinToInterrupt(iqs7222c_ready_pin), iqs7222c_ready_interrupt, CHANGE);

//   /* Initialize "running" and "init" state machine variables. */
//   iqs7222c_state.state = IQS7222C_STATE_START;
//   iqs7222c_state.init_state = IQS7222C_INIT_VERIFY_PRODUCT;
// }

/**
 * @name   init
 * @brief  A method that runs through a normal start-up routine to set up the
 *         IQS7222C with the desired settings from the IQS7222C_init.h file.
 * @retval Returns true if the full start-up routine has been completed,
 *         returns false if not.
 * @note   No false return will be given, the program will thus be stuck when
 *         one of the cases is not able to finish. See serial communication to
 *         find the ERROR case
 */
bool IQS7222CComponent::init(void) {
  uint16_t prod_num;
  uint8_t ver_maj, ver_min;

  switch (iqs7222c_state.init_state) {
    /* Verifies product number to determine if the correct device is connected for
       this example */
    case IQS7222C_INIT_VERIFY_PRODUCT:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_VERIFY_PRODUCT");
        prod_num = getProductNum(RESTART);
        ver_maj = getmajorVersion(RESTART);
        ver_min = getminorVersion(STOP);
        ESP_LOGD(TAG, "Product number is: %d v %d.%d", prod_num, ver_maj, ver_min);
        if (prod_num == IQS7222C_PRODUCT_NUM) {
          ESP_LOGD(TAG, "IQS7222C Release UI Confirmed!");
          iqs7222c_state.init_state = IQS7222C_INIT_UPDATE_SETTINGS;
        } else {
          ESP_LOGD(TAG, "Device is not a IQS7222C!");
          iqs7222c_state.init_state = IQS7222C_INIT_NONE;
          this->error_code_ = COMMUNICATION_FAILED;
          this->mark_failed();
        }
      }
      break;

    /* Verify if a reset has occurred */
    case IQS7222C_INIT_READ_RESET:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_READ_RESET");
        updateInfoFlags(RESTART);
        if (checkReset()) {
          ESP_LOGD(TAG, "Reset event occurred.");
          iqs7222c_state.init_state = IQS7222C_INIT_VERIFY_PRODUCT;
        } else {
          ESP_LOGD(TAG, " No Reset Event Detected - Request SW Reset");
          iqs7222c_state.init_state = IQS7222C_INIT_CHIP_RESET;
        }
      }
      break;

    /* Perform SW Reset */
    case IQS7222C_INIT_CHIP_RESET:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_CHIP_RESET");

        // Perform SW Reset
        SW_Reset(STOP);
        ESP_LOGD(TAG, "Software Reset Bit Set.");
        delay(100);
        iqs7222c_state.init_state = IQS7222C_INIT_VERIFY_PRODUCT;
      }
      break;

    /* Write all settings to IQS7222C from .h file */
    case IQS7222C_INIT_UPDATE_SETTINGS:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_UPDATE_SETTINGS");
        writeMM(RESTART);
        iqs7222c_state.init_state = IQS7222C_INIT_ACK_RESET;
      }
      break;

    /* Acknowledge that the device went through a reset */
    case IQS7222C_INIT_ACK_RESET:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_ACK_RESET");
        acknowledgeReset(STOP);
        iqs7222c_state.init_state = IQS7222C_INIT_ATI;
      }
      break;

    /* Run the ATI algorithm to recalibrate the device with newly added settings */
    case IQS7222C_INIT_ATI:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_ATI");
        ReATI(STOP);
        iqs7222c_state.init_state = IQS7222C_INIT_WAIT_FOR_ATI;
        ESP_LOGD(TAG, "IQS7222C_INIT_WAIT_FOR_ATI");
      }
      break;

    /* Read the ATI Active bit to see if the rest of the program can continue */
    case IQS7222C_INIT_WAIT_FOR_ATI:
      if (this->getRDYStatus()) {
        if (!readATIactive()) {
          ESP_LOGD(TAG, "DONE");
          iqs7222c_state.init_state = IQS7222C_INIT_READ_DATA;
        }
      }
      break;

    /* Read the latest data from the iqs7222c */
    case IQS7222C_INIT_READ_DATA:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_READ_DATA");
        queueValueUpdates();
        iqs7222c_state.init_state = IQS7222C_INIT_ACTIVATE_EVENT_MODE;
        // IQS7222C_INIT_ACTIVATE_STREAM_IN_TOUCH_MODE;
      }
      break;

    /* Turn on I2C event mode */
    case IQS7222C_INIT_ACTIVATE_EVENT_MODE:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_ACTIVATE_EVENT_MODE");
        setEventMode(STOP);
        iqs7222c_state.init_state = IQS7222C_INIT_DONE;
      }
      break;

    /* Turn on I2C Stream-In-Touch mode */
    case IQS7222C_INIT_ACTIVATE_STREAM_IN_TOUCH_MODE:
      if (this->getRDYStatus()) {
        ESP_LOGD(TAG, "IQS7222C_INIT_ACTIVATE_STREAM_IN_TOUCH_MODE");
        setStreamInTouchMode(STOP);
        iqs7222c_state.init_state = IQS7222C_INIT_DONE;
      }
      break;

    /* If all operations have been completed correctly, the RDY pin can be set
     * up as an interrupt to indicate when new data is available */
    case IQS7222C_INIT_DONE:
      ESP_LOGD(TAG, "IQS7222C_INIT_DONE");
      new_data_available = false;
      this->clearRDY();
      // store_.iqs7222c_deviceRDY = false;

      // if (this->interrupt_pin_ != nullptr) {
      //   this->interrupt_pin_->pin_mode(gpio::FLAG_INPUT);
      //   this->interrupt_pin_->setup();
      //   this->attach_interrupt_(this->interrupt_pin_, gpio::INTERRUPT_ANY_EDGE);
      // }

      return true;
      break;

    default:
      break;
  }
  return false;
}

/**
 * @name   run
 * @brief  This method is called continuously during runtime, and serves as the
 *         main state machine
 * @param  None.
 * @retval None.
 * @note   The state machine continuously checks for specific events and
 *         updates the state machine accordingly. A reset event will cause
 *         the state machine to re-initialize the device.
 *
 *         queueValueUpdates can be edited by the user if other data should be
 *         read every time a RDY window is received.
 */
void IQS7222CComponent::run(void) {
  {
    static iqs7222c_s old = {IQS7222C_STATE_NONE, IQS7222C_INIT_ACTIVATE_STREAM_IN_TOUCH_MODE};
    if (old.state != iqs7222c_state.state || old.init_state != iqs7222c_state.init_state) {
      old = iqs7222c_state;
      ESP_LOGD(TAG, "run() %d, %d", iqs7222c_state.state, iqs7222c_state.init_state);
    }
  }
  switch (iqs7222c_state.state) {
    /* After a hardware reset, this is the starting position of the main state
       machine */
    case IQS7222C_STATE_START:
      ESP_LOGD(TAG, "IQS7222C Initialization:");
      iqs7222c_state.state = IQS7222C_STATE_INIT;
      break;

    /* Perform the initialization routine on the IQS7222C */
    case IQS7222C_STATE_INIT:
      if (init()) {
        ESP_LOGD(TAG, "IQS7222C Initialization complete!\n");
        iqs7222c_state.state = IQS7222C_STATE_RUN;
      }
      break;

    /* Send an 12C software reset in the next rdy window */
    case IQS7222C_STATE_SW_RESET:
      if (store_.iqs7222c_deviceRDY) {
        SW_Reset(STOP);
        iqs7222c_state.state = IQS7222C_STATE_RUN;
      }
      break;

    /* Continuous reset monitoring state, ensure no reset event has occurred
       for data to be valid */
    case IQS7222C_STATE_CHECK_RESET:
      if (checkReset()) {
        ESP_LOGD(TAG, "Reset Occurred!\n");
        new_data_available = false;
        iqs7222c_state.state = IQS7222C_STATE_START;
        iqs7222c_state.init_state = IQS7222C_INIT_VERIFY_PRODUCT;
      }

      /* A reset did not occur, move to the run state and wait for new ready window */
      else {
        ESP_LOGD(TAG, "New data available!\n");
        new_data_available = true; /* No reset, thus data is valid */
        iqs7222c_state.state = IQS7222C_STATE_RUN;
      }
      break;

    /* If a RDY Window is open, read the latest values from the IQS7222C */
    case IQS7222C_STATE_RUN:
      if (store_.iqs7222c_deviceRDY) {
        queueValueUpdates();
        this->clearRDY();
        new_data_available = false;
        iqs7222c_state.state = IQS7222C_STATE_CHECK_RESET;
      }
      break;
  }
}

void IRAM_ATTR IQS7222CStore::gpio_intr(IQS7222CStore *store) {
  bool old = store->iqs7222c_deviceRDY;
  bool current = !store->irq_pin.digital_read();  // RDY active low
  store->iqs7222c_deviceRDY = current;
  if (old != current) {
    store->rdy_window_number++;
  }
}

void IQS7222CComponent::attach_interrupt_(InternalGPIOPin *irq_pin, esphome::gpio::InterruptType type) {
  ESP_LOGD(TAG, "attach_interrupt_(RDY)");
  this->store_.irq_pin = irq_pin->to_isr();
  this->store_.init = true;
  this->store_.rdy_window_number = 0;
  this->clearRDY();
  irq_pin->attach_interrupt(IQS7222CStore::gpio_intr, &this->store_, type);
}

/**
 * @name   iqs7222c_ready_interrupt
 * @brief  A method used as an interrupt function. Activated when a High to Low or
 *         Low to High interrupt is seen on the correct Arduino interrupt pin.
 * @param  None.
 * @retval None.
 * @note   Keep this function as simple as possible to prevent stuck states
 *         and slow operations.
 */
// void iqs7222c_ready_interrupt(void) {
//   if (digitalRead(iqs7222c_ready_pin)) {
//     store_.iqs7222c_deviceRDY = false;
//   } else {
//     store_.iqs7222c_deviceRDY = true;
//   }
// }

/**
 * @name   clearRDY
 * @brief  A method used to clear the ready interrupt bit.
 * @param  None.
 * @retval None.
 */
void IQS7222CComponent::clearRDY(void) {
  ESP_LOGD(TAG, "ClearRDY()");
  store_.iqs7222c_deviceRDY = false;
}

/**
 * @name   getRDYStatus
 * @brief  A method used to retrieve the device RDY status.
 * @param  None.
 * @retval Returns the boolean IQS323 RDY state.
 *         - True when RDY line is LOW
 *         - False when RDY line is HIGH
 */
bool IQS7222CComponent::getRDYStatus(void) {
  return store_.iqs7222c_deviceRDY;
  // || !this->store_.init;
}

/**
 * @name   queueValueUpdates
 * @brief  All I2C read operations in the queueValueUpdates method will be
 *         performed each time the IQS7222C opens a RDY window.
 * @param  None.
 * @retval None.
 * @note   Any Address in the memory map can be read from here.
 */
void IQS7222CComponent::queueValueUpdates(void) {
  uint8_t transferBytes[10];  // The array which will hold the bytes to be transferred.

  // Read the info flags.
  readRandomBytes(IQS7222C_MM_INFOFLAGS, 10, transferBytes, STOP);

  // Assign the info flags to the local memory map.
  IQSMemoryMap.SYSTEM_STATUS[0] = transferBytes[0];
  IQSMemoryMap.SYSTEM_STATUS[1] = transferBytes[1];

  IQSMemoryMap.EVENTS[0] = transferBytes[2];
  IQSMemoryMap.EVENTS[1] = transferBytes[3];

  IQSMemoryMap.PROX_EVENT_STATES[0] = transferBytes[4];
  IQSMemoryMap.PROX_EVENT_STATES[1] = transferBytes[5];

  IQSMemoryMap.TOUCH_EVENT_STATES[0] = transferBytes[6];
  IQSMemoryMap.TOUCH_EVENT_STATES[1] = transferBytes[7];

  IQSMemoryMap.SLIDER_WHEEL_0[0] = transferBytes[8];
  IQSMemoryMap.SLIDER_WHEEL_0[1] = transferBytes[9];
}

/**
 * @name	  readATIactive
 * @brief  A method that checks if the ATI routine is still active
 * @param  None.
 * @retval Returns true if the ATI_ACTIVE_BIT is cleared, false if the
 *         ATI_ACTIVE_BIT is set.
 * @note   If the ATI routine is active the channel states (NONE, PROX, TOUCH)
 *         might exhibit unwanted behaviour. Thus it is advised to wait for
 *         the routine to complete before continuing.
 */
bool IQS7222CComponent::readATIactive(void) {
  /* Read the Info flags */
  updateInfoFlags(STOP);

  /* Return the ATI Active status */
  return getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS7222C_ATI_ACTIVE_BIT);
}

/**
 * @name	  checkReset
 * @brief  A method that checks if the device has reset and returns the reset
 *         status.
 * @param  None.
 * @retval Returns true if a reset has occurred, false if no reset has occurred.
 * @note   If a reset has occurred the device settings should be reloaded using
 *         the begin function.
 * 		    After new device settings have been reloaded the acknowledge reset
 *         function can be used to clear the reset flag.
 */
bool IQS7222CComponent::checkReset(void) {
  /* Perform a bitwise AND operation inside getBIT with the DEVICE_RESET_BIT
  to return the reset status */
  return getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS7222C_DEVICE_RESET_BIT);
}

/**
 * @name	  getProductNum
 * @brief  A method that checks the device product number
 * @param  stopOrRestart -> Specifies whether the communications window must
 *                          be kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval Returns the product number as a 16-bit unsigned integer value.
 * @note   If the product is not correctly identified an appropriate messages
 *         should be displayed.
 */
uint16_t IQS7222CComponent::getProductNum(bool stopOrRestart) {
  uint8_t transferBytes[2];    // A temporary array to hold the byte to be transferred.
  uint8_t prodNumLow = 0;      // Temporary storage for the counts low byte.
  uint8_t prodNumHigh = 0;     // Temporary storage for the counts high byte.
  uint16_t prodNumReturn = 0;  // The 16-bit return value.

  /* Read the Device info from the IQS7222C. */
  readRandomBytes(IQS7222C_MM_PROD_NUM, 2, transferBytes, stopOrRestart);

  /* Construct the 16-bit return value. */
  prodNumLow = transferBytes[0];
  prodNumHigh = transferBytes[1];
  prodNumReturn = (uint16_t) (prodNumLow);
  prodNumReturn |= (uint16_t) (prodNumHigh << 8);
  /* Return the counts value. */
  return prodNumReturn;
}

/**
 * @name	getmajorVersion
 * @brief  A method that checks the device firmware version's major value.
 * @param  stopOrRestart -> Specifies whether the communications window must
 *                          be kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval Returns the major version number as an 8-bit unsigned integer value.
 */
uint8_t IQS7222CComponent::getmajorVersion(bool stopOrRestart) {
  uint8_t transferBytes[2];  // A temporary array to hold the byte to be transferred.
  uint8_t ver_maj = 0;       // Temporary storage for the firmware version major number.

  /* Read the Device info from the IQS7222C. */
  readRandomBytes(IQS7222C_MM_MAJOR_VERSION_NUM, 2, transferBytes, stopOrRestart);

  /* Get major value from correct byte */
  ver_maj = transferBytes[0];
  /* Return the major firmware version number value. */
  return ver_maj;
}

/**
 * @name	getminorVersion
 * @brief  A method that checks the device firmware version's minor value.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval Returns the minor version number as an 8-bit unsigned integer value.
 */
uint8_t IQS7222CComponent::getminorVersion(bool stopOrRestart) {
  uint8_t transferBytes[2];  // A temporary array to hold the byte to be transferred.
  uint8_t ver_min = 0;       // Temporary storage for the firmware version minor number.

  /* Read the Device info from the IQS7222C. */
  readRandomBytes(IQS7222C_MM_MINOR_VERSION_NUM, 2, transferBytes, stopOrRestart);
  /* get major value from correct byte */
  ver_min = transferBytes[0];
  /* Return the minor firmware version number value. */
  return ver_min;
}

/**
 * @name	acknowledgeReset
 * @brief  A method that clears the Reset Event bit by writing it to a 0.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   If a reset has occurred the device settings should be reloaded using
 *         the begin function. After new device settings have been reloaded
 *         this method should be used to clear the reset bit.
 */
void IQS7222CComponent::acknowledgeReset(bool stopOrRestart) {
  uint8_t transferByte[2];  // A temporary array to hold the bytes to be transferred.
  /* Read the System Flags from the IQS7222C, these must be read first in order
  not to change any settings. */
  readRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferByte, RESTART);
  /* Set the ACK_RESET_BIT in the CONTROL_SETTINGS register */
  transferByte[0] = setBit(transferByte[0], IQS7222C_ACK_RESET_BIT);
  /* Write the new byte to the System Flags address. */
  writeRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferByte, stopOrRestart);
}

/**
 * @name   ReATI
 * @brief  A method which sets the REDO_ATI_BIT in order to force the IQS7222C
 *         device to run the Automatic Tuning Implementation (ATI) routine.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   I2C communications are disabled for the duration of the ATI process.
 */
void IQS7222CComponent::ReATI(bool stopOrRestart) {
  uint8_t transferByte[2];  // Array to store the bytes transferred.

  readRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferByte, RESTART);
  /* Set the RE_ATI_BIT in the CONTROL_SETTINGS register */
  transferByte[0] = setBit(transferByte[0], IQS7222C_RE_ATI_BIT);
  /* Write the new byte to the required device. */
  writeRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferByte, stopOrRestart);
}

/**
 * @name   SW_Reset
 * @brief  A method that sets the SW RESET bit to force the IQS7222C
 *         device to do a SW reset.
 * @param  stopOrRestart -> Specifies whether the communications window must
 *                          be kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   To perform SW Reset, bit IQS7222C_SW_RESET_BIT in SYSTEM_CONTROL is
 *         set.
 */
void IQS7222CComponent::SW_Reset(bool stopOrRestart) {
  uint8_t transferByte[2];  // Array to store the bytes transferred.

  readRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferByte, RESTART);
  /* Set the SW_RESET_BIT in the CONTROL_SETTINGS register */
  transferByte[0] = setBit(transferByte[0], IQS7222C_SW_RESET_BIT);
  /* Write the new byte to the required device. */
  writeRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferByte, stopOrRestart);
}

/**
 * @name   setStreamMode
 * @brief  A method to set the IQS7222C device into streaming mode.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   All other bits at the register address are preserved.
 */
void IQS7222CComponent::setStreamMode(bool stopOrRestart) {
  uint8_t transferBytes[2];  // The array which will hold the bytes which are transferred.

  // First read the bytes at the memory address so that they can be preserved.
  readRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferBytes, RESTART);
  // Set/Clear the INTERFACE_SELECTION_BITS in CONTROL_SETTINGS
  transferBytes[0] = clearBit(transferBytes[0], IQS7222C_INTERFACE_SELECT_BIT_0);
  transferBytes[0] = clearBit(transferBytes[0], IQS7222C_INTERFACE_SELECT_BIT_1);
  // Write the bytes back to the device
  writeRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferBytes, stopOrRestart);
}

/**
 * @name   setEventMode
 * @brief  A method to set the IQS7222C device into event mode.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   All other bits at the register address are preserved.
 */
void IQS7222CComponent::setEventMode(bool stopOrRestart) {
  uint8_t transferByte[2];  // The array which will hold the bytes which are transferred.

  /* First read the bytes at the memory address so that they can be preserved */
  readRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferByte, RESTART);
  /* Set/Clear the INTERFACE_SELECTION_BITS in CONTROL_SETTINGS */
  transferByte[0] = setBit(transferByte[0], IQS7222C_INTERFACE_SELECT_BIT_0);
  transferByte[0] = clearBit(transferByte[0], IQS7222C_INTERFACE_SELECT_BIT_1);
  /* Write the bytes back to the device */
  writeRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferByte, stopOrRestart);
}

/**
 * @name   setStreamInTouchMode
 * @brief  A method to set the IQS7222C device into streaming when in touch mode.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   All other bits at the register address are preserved.
 */
void IQS7222CComponent::setStreamInTouchMode(bool stopOrRestart) {
  uint8_t transferBytes[2];  // The array which will hold the bytes which are transferred.

  // First read the bytes at the memory address so that they can be preserved.
  readRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferBytes, RESTART);
  // Set/Clear the INTERFACE_SELECTION_BITS in CONTROL_SETTINGS
  transferBytes[0] = clearBit(transferBytes[0], IQS7222C_INTERFACE_SELECT_BIT_0);
  transferBytes[0] = setBit(transferBytes[0], IQS7222C_INTERFACE_SELECT_BIT_1);
  // Write the bytes back to the device
  writeRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 2, transferBytes, stopOrRestart);
}

/**
 * @name   updateInfoFlags
 * @brief  A method that reads the info flags from the IQS7222C
 * @param  stopOrRestart -> Specifies whether the communications window must
 *                          be kept open or must be closed after this action.
 *              			       Use the STOP and RESTART definitions.
 * @retval None.
 */
void IQS7222CComponent::updateInfoFlags(bool stopOrRestart) {
  uint8_t transferBytes[2];  // The array which will hold the bytes to be transferred.

  // Read the info flags.
  readRandomBytes(IQS7222C_MM_INFOFLAGS, 2, transferBytes, stopOrRestart);
  // Assign the info flags to the local memory map.
  IQSMemoryMap.SYSTEM_STATUS[0] = transferBytes[0];
  IQSMemoryMap.SYSTEM_STATUS[1] = transferBytes[1];
  ESP_LOGD(TAG, "updateInfoFlags: 0x%02X, 0x%02X", transferBytes[0], transferBytes[1]);
}

/**
 * @name   get_PowerMode
 * @brief  A method that reads the system flags and returns the current
 *         power mode.
 * @param  void
 * @retval Returns the current iqs7222c_power_modes state the device is in.
 * @note   See Datasheet on power mode options and timeouts.
 *         Normal Power, Low Power, and Ultra Low Power (ULP).
 */
iqs7222c_power_modes IQS7222CComponent::get_PowerMode(void) {
  uint8_t buffer = getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS7222C_POWER_EVENT_BIT_0);
  buffer += getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS7222C_POWER_EVENT_BIT_1) << 1;

  if (buffer == IQS7222C_NORMAL_POWER_BIT) {
    return IQS7222C_NORMAL_POWER;
  } else if (buffer == IQS7222C_LOW_POWER_BIT) {
    return IQS7222C_LOW_POWER;
  } else if (buffer == IQS7222C_ULP_BIT) {
    return IQS7222C_ULP;
  } else {
    return IQS7222C_POWER_UNKNOWN;
  }
}

/**
 * @name   channel_touchState
 * @brief  A method which reads the TOUCH_EVENT_STATES register for a given
 *         channel and determines if the channel is in a touch state.
 * @param  channel -> The channel name on the IQS7222C (CH0-CH9) for which a
 *         touch state needs to be determined.
 * @retval Returns true if a touch is active and false if there is no touch.
 * @note   See the iqs7222c_channel_e typedef for all possible channel names.
 */
bool IQS7222CComponent::channel_touchState(iqs7222c_channel_e channel) {
  switch (channel) {
    case IQS7222C_CH0:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[0], IQS7222C_CH0_TOUCH_BIT);
      break;

    case IQS7222C_CH1:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[0], IQS7222C_CH1_TOUCH_BIT);
      break;

    case IQS7222C_CH2:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[0], IQS7222C_CH2_TOUCH_BIT);
      break;

    case IQS7222C_CH3:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[0], IQS7222C_CH3_TOUCH_BIT);
      break;

    case IQS7222C_CH4:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[0], IQS7222C_CH4_TOUCH_BIT);
      break;

    case IQS7222C_CH5:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[0], IQS7222C_CH5_TOUCH_BIT);
      break;

    case IQS7222C_CH6:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[0], IQS7222C_CH6_TOUCH_BIT);
      break;

    case IQS7222C_CH7:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[0], IQS7222C_CH7_TOUCH_BIT);
      break;

    case IQS7222C_CH8:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[1], IQS7222C_CH8_TOUCH_BIT);
      break;

    case IQS7222C_CH9:
      return getBit(IQSMemoryMap.TOUCH_EVENT_STATES[1], IQS7222C_CH9_TOUCH_BIT);
      break;

    default:
      return false;
      break;
  }
}

/**
 * @name   channel_proxState
 * @brief  A method which reads the PROX_EVENT_STATES register for a given
 *         channel and determines if the channel is in a proximity state.
 * @param  channel -> The channel name on the IQS7222C (CH0-CH9) for which
 *                    a proximity state needs to be determined.
 * @retval Returns true if proximity is active and false if there is no
 *         proximity.
 * @note   See the iqs7222c_channel_e typedef for all possible channel names.
 */
bool IQS7222CComponent::channel_proxState(iqs7222c_channel_e channel) {
  switch (channel) {
    case IQS7222C_CH0:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[0], IQS7222C_CH0_PROX_BIT);
      break;

    case IQS7222C_CH1:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[0], IQS7222C_CH1_PROX_BIT);
      break;

    case IQS7222C_CH2:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[0], IQS7222C_CH2_PROX_BIT);
      break;

    case IQS7222C_CH3:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[0], IQS7222C_CH3_PROX_BIT);
      break;

    case IQS7222C_CH4:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[0], IQS7222C_CH4_PROX_BIT);
      break;

    case IQS7222C_CH5:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[0], IQS7222C_CH5_PROX_BIT);
      break;

    case IQS7222C_CH6:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[0], IQS7222C_CH6_PROX_BIT);
      break;

    case IQS7222C_CH7:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[0], IQS7222C_CH7_PROX_BIT);
      break;

    case IQS7222C_CH8:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[1], IQS7222C_CH8_PROX_BIT);
      break;

    case IQS7222C_CH9:
      return getBit(IQSMemoryMap.PROX_EVENT_STATES[1], IQS7222C_CH9_PROX_BIT);
      break;

    default:
      return false;
      break;
  }
}

/**
 * @name   sliderCoordinate
 * @brief  A method that reads the coordinates slider output from the SLIDER_WHEEL
 *         register where the calculated current slider position is stored.
 * @param  slider -> The slider name on the IQS7222C (Slider 0 or Slider 1).
 * @retval Returns a 16-bit value that contains the slider coordinates from
 *         0 to the resolution maximum.
 * @note   See the IQS7222C_slider_e typedef for all possible slider names.
 */
uint16_t IQS7222CComponent::sliderCoordinate(iqs7222c_slider_e slider) {
  uint16_t buffer;
  if (slider == IQS7222C_SLIDER0) {
    buffer = IQSMemoryMap.SLIDER_WHEEL_0[0];
    buffer += IQSMemoryMap.SLIDER_WHEEL_0[1] << 8;
    return buffer;
  } else {
    buffer = IQSMemoryMap.SLIDER_WHEEL_1[0];
    buffer += IQSMemoryMap.SLIDER_WHEEL_1[1] << 8;
    return buffer;
  }
}
/*****************************************************************************/
/*								       			ADVANCED PUBLIC METHODS										     */
/*****************************************************************************/

/**
 * @name   writeMM
 * @brief  Function to write the whole memory map to the device (writable)
 *         registers
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   IQS7222C_init.h -> exported GUI init.h file
 */
void IQS7222CComponent::writeMM(bool stopOrRestart) {
  uint8_t transferBytes[30];  // Temporary array which holds the bytes to be transferred.

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
  writeRandomBytes16(IQS7222C_MM_CYCLE_SETUP_0, 30, transferBytes, RESTART);
  // ESP_LOGD(TAG, "1. Write Cycle Settings");

  /* Change the Global Cycle Setup */
  /* Memory Map Position 0x8500 - 0x8502 */
  transferBytes[0] = GLOBAL_CYCLE_SETUP_0;
  transferBytes[1] = GLOBAL_CYCLE_SETUP_1;
  transferBytes[2] = COARSE_DIVIDER_PRELOAD;
  transferBytes[3] = FINE_DIVIDER_PRELOAD;
  transferBytes[4] = COMPENSATION_PRELOAD_0;
  transferBytes[5] = COMPENSATION_PRELOAD_1;
  writeRandomBytes16(IQS7222C_MM_GLOBAL_CYCLE_SETUP, 6, transferBytes, RESTART);
  // ESP_LOGD(TAG, "2. Write Global Cycle Settings");

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
  writeRandomBytes16(IQS7222C_MM_BUTTON_SETUP_0, 30, transferBytes, RESTART);
  // ESP_LOGD(TAG, "3. Write Button Settings 0 - 4");

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
  writeRandomBytes16(IQS7222C_MM_BUTTON_SETUP_5, 30, transferBytes, RESTART);
  // ESP_LOGD(TAG, "4. Write Button Settings 5 - 9");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_0, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "5. Write Channel 0 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_1, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "6. Write Channel 1 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_2, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "7. Write Channel 2 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_3, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "8. Write Channel 3 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_4, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "9. Write Channel 4 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_5, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "10. Write Channel 5 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_6, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "11. Write Channel 6 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_7, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "12. Write Channel 7 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_8, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "13. Write Channel 8 Settings");

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
  writeRandomBytes16(IQS7222C_MM_CHANNEL_SETUP_9, 12, transferBytes, RESTART);
  // ESP_LOGD(TAG, "14. Write Channel 9 Settings");

  /* Change the Filter Betas */
  /* Memory Map Position 0xAA00 - 0xAA01 */
  transferBytes[0] = COUNTS_BETA_FILTER;
  transferBytes[1] = LTA_BETA_FILTER;
  transferBytes[2] = LTA_FAST_BETA_FILTER;
  transferBytes[3] = RESERVED_FILTER_0;
  writeRandomBytes16(IQS7222C_MM_FILTER_BETAS, 4, transferBytes, RESTART);
  // ESP_LOGD(TAG, "15. Write Filter Betas");

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
  writeRandomBytes16(IQS7222C_MM_SLIDER_SETUP_0, 20, transferBytes, RESTART);
  // ESP_LOGD(TAG, "16. Slider/Wheel 0 Settings");

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
  writeRandomBytes16(IQS7222C_MM_SLIDER_SETUP_1, 20, transferBytes, RESTART);
  // ESP_LOGD(TAG, "17. Slider/Wheel 1 Settings");

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
  writeRandomBytes16(IQS7222C_MM_GPIO_0_SETTINGS, 18, transferBytes, RESTART);
  // ESP_LOGD(TAG, "18. GPIO 0 Settings");

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
  writeRandomBytes(IQS7222C_MM_CONTROL_SETTINGS, 21, transferBytes, stopOrRestart);
  // ESP_LOGD(TAG, "19. System Settings");

  /* Change the GPIO Override */
  /* Memory Map Position 0xDB - 0xDB */
  transferBytes[0] = GPIO_OVERRIDE;
  writeRandomBytes(IQS7222C_MM_GPIO_OVERRIDE, 1, transferBytes, stopOrRestart);
  // ESP_LOGD(TAG, "20. GPIO Override");

  /* Change the Comms timeout setting */
  /* Memory Map Position 0xDC - 0xDC */
  transferBytes[0] = COMMS_TIMEOUT_0;
  transferBytes[1] = COMMS_TIMEOUT_1;
  writeRandomBytes(IQS7222C_MM_COMMS_TIMEOUT, 2, transferBytes, stopOrRestart);
  // ESP_LOGD(TAG, "21. Communication Timeout");
}

/*****************************************************************************/
/*                           PRIVATE METHODS                                 */
/*****************************************************************************/

/**
 * @name    readRandomBytes
 * @brief   A method that reads a specified number of bytes from a specified
 *          address and saves it into a user-supplied array.
 *          This method is used by all other methods in this class which read
 *          data from the IQS7222C device.
 * @param   memoryAddress -> The memory address at which to start reading bytes
 *                           from.  See the "iqs7222c_addresses.h" file.
 * @param   numBytes      -> The number of bytes that must be read.
 * @param   bytesArray    -> The array which will store the bytes to be read,
 *                           this array will be overwritten.
 * @param   stopOrRestart -> A boolean that specifies whether the communication
 *                           window should remain open or be closed after transfer.
 *                           False keeps it open, true closes it. Use the STOP
 *                           and RESTART definitions.
 * @retval  No value is returned, however, the user-supplied array is overwritten.
 * @note    Uses standard Arduino "Wire" library which is for I2C communication.
 *          Take note that C++ cannot return an array, therefore, the array which
 *          is passed as an argument is overwritten with the required values.
 *          Pass an array to the method by using only its name, e.g. "bytesArray",
 *          without the brackets, this passes a pointer to the array.
 */
void IQS7222CComponent::readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[],
                                        bool stopOrRestart) {
  i2c::ErrorCode ret = this->read_register(memoryAddress, bytesArray, numBytes, stopOrRestart);
  if (ret != i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Error reading bytes from memory address %02X, error %d", memoryAddress, (int) ret);
  }

  // uint8_t i = 0;  // A simple counter to assist with loading bytes into the user-supplied array.

  // // Select the device with the address of "_deviceAddress" and start communication.
  // Wire.beginTransmission(_deviceAddress);
  // // Send a bit asking for the "memoryAddress" register.
  // Wire.write(memoryAddress);
  // // Complete the selection and communication initialization.
  // Wire.endTransmission(RESTART);  // Restart transmission for reading that follows.
  // // The required device has now been selected and it has been told which register to send information from.

  // // Request "numBytes" bytes from the device which has address "_deviceAddress"
  // do {
  //   Wire.requestFrom((int) _deviceAddress, (int) numBytes, (int) stopOrRestart);
  // } while (Wire.available() == 0);  // Wait for response, this sometimes takes a few attempts

  // // Load the received bytes into the array until there are no more
  // while (Wire.available()) {
  //   // Load the received bytes into the user-supplied array
  //   bytesArray[i] = Wire.read();
  //   i++;
  // }

  /* Always manually close the RDY window after a STOP is sent to prevent
  writing while ready closes */
  if (stopOrRestart == STOP) {
    ESP_LOGD(TAG, "Closing RDY window after read bytes with STOP. reg %02X", memoryAddress);
    this->clearRDY();
  }
}

/**
 * @name   writeRandomBytes
 * @brief  A method that writes a specified number of bytes to a specified
 *         address, the bytes to write are supplied using an array pointer.
 *         This method is used by all other methods of this class that write
 *         data to the IQS7222C device.
 * @param  memoryAddress -> The memory address at which to start writing the bytes
 *                          to. See the "iqs7222c_addresses.h" file.
 * @param  numBytes      -> The number of bytes that must be written.
 * @param  bytesArray    -> The array which stores the bytes which will be
 *                          written to the memory location.
 * @param  stopOrRestart -> A boolean that specifies whether the communication
 *                          window should remain open or be closed of transfer.
 *                          False keeps it open, true closes it. Use the STOP
 *                          and RESTART definitions.
 * @retval No value is returned, only the IQS device registers are altered.
 * @note   Uses standard Arduino "Wire" library which is for I2C communication.
 *         Take note that a full array cannot be passed to a function in C++.
 *         Pass an array to the function by using only its name, e.g. "bytesArray",
 *         without the square brackets, this passes a pointer to the
 *         array. The values to be written must be loaded into the array prior
 *         to passing it to the function.
 */
void IQS7222CComponent::writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[],
                                         bool stopOrRestart) {
  this->write_register(memoryAddress, bytesArray, numBytes, stopOrRestart);
  // // Select the device with the address of "_deviceAddress" and start communication.
  // Wire.beginTransmission(_deviceAddress);
  // // Specify the memory address where the IQS7222C must start saving the data, as designated by the "memoryAddress"
  // // variable.
  // Wire.write(memoryAddress);
  // // Write the bytes as specified in the array which "arrayAddress" pointer points to.
  // for (int i = 0; i < numBytes; i++) {
  //   Wire.write(bytesArray[i]);
  // }
  // // End the transmission, user decides to STOP or RESTART.
  // Wire.endTransmission(stopOrRestart);

  /* Always manually close the RDY window after a STOP is sent to prevent writing
  while ready closes */
  if (stopOrRestart == STOP) {
    store_.iqs7222c_deviceRDY = false;
  }
}

/**
 * @name   writeRandomBytes
 * @brief  A method that writes a specified number of bytes to a specified
 *         address, the bytes to write are supplied using an array pointer.
 *         This method is used by all other methods of this class which
 *         write data to the IQS7222C device.
 * @param  memoryAddress -> The memory address at which to start writing the
 *         bytes to. See the "iqs7222c_addresses.h" file.
 *         numBytes      -> The number of bytes that must be written.
 *         bytesArray    -> The array which stores the bytes which will be
 *                          written to the memory location.
 *         stopOrRestart -> A boolean that specifies whether the communication
 *                          window should remain open or be closed of transfer.
 *                          False keeps it open, true closes it. Use the STOP
 *                          and RESTART definitions.
 * @retval No value is returned, only the IQS device registers are altered.
 * @note   Uses standard Arduino "Wire" library which is for I2C communication.
 *         Take note that a full array cannot be passed to a function in C++.
 *         Pass an array to the function by using only its name, e.g. "bytesArray",
 *         without the square brackets, this passes a pointer to the
 *         array. The values to be written must be loaded into the array prior
 *         to passing it to the function.
 */
void IQS7222CComponent::writeRandomBytes16(uint16_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[],
                                           bool stopOrRestart) {
  this->write_register16(memoryAddress, bytesArray, numBytes, stopOrRestart);
  // // Select the device with the address of "_deviceAddress" and start communication.
  // Wire.beginTransmission(_deviceAddress);
  // /* Specify the memory address where the IQS7222C must start saving the data,
  // as designated by the "memoryAddress" variable.*/
  // uint8_t addr_h, addr_l;
  // addr_h = memoryAddress >> 8;
  // addr_l = memoryAddress;
  // Wire.write(addr_h);
  // Wire.write(addr_l);
  // // Write the bytes as specified in the array which "arrayAddress" pointer points to.
  // for (int i = 0; i < numBytes; i++) {
  //   Wire.write(bytesArray[i]);
  // }
  // // End the transmission, user decides to STOP or RESTART.
  // Wire.endTransmission(stopOrRestart);

  /* Always manually close the RDY window after a STOP is sent to prevent
     writing while ready closes */
  if (stopOrRestart == STOP) {
    store_.iqs7222c_deviceRDY = false;
  }
}

/**
 * @name   getBit
 * @brief  A method that returns the chosen bit value of the provided byte.
 * @param  data       -> byte of which a given bit value needs to be calculated.
 * @param  bit_number -> a number between 0 and 7 representing the bit in
 *                       question.
 * @retval The boolean value of the specific bit requested.
 */
bool IQS7222CComponent::getBit(uint8_t data, uint8_t bit_number) { return (data & (1 << bit_number)) >> bit_number; }

/**
 * @name   setBit
 * @brief  A method that returns the chosen bit value of the provided byte.
 * @param  data       -> byte of which a given bit value needs to be calculated.
 * @param  bit_number -> a number between 0 and 7 representing the bit in question.
 * @retval Returns an 8-bit unsigned integer value of the given data byte with
 *         the requested bit set.
 */
uint8_t IQS7222CComponent::setBit(uint8_t data, uint8_t bit_number) { return (data |= 1UL << bit_number); }

/**
 * @name   clearBit
 * @brief  A method that returns the chosen bit value of the provided byte.
 * @param  data       -> byte of which a given bit value needs to be calculated.
 * @param  bit_number -> a number between 0 and 7 representing the bit in question.
 * @retval Returns an 8-bit unsigned integer value of the given data byte with
 *         the requested bit cleared.
 */
uint8_t IQS7222CComponent::clearBit(uint8_t data, uint8_t bit_number) { return (data &= ~(1UL << bit_number)); }

/**
 * @name   force_I2C_communication
 * @brief  A method which writes to memory address 0xFF to open a
 *         communication window on the IQS7222C.
 * @retval None
 * @note   Uses standard Arduino "Wire" library which is for I2C communication.
 */
void IQS7222CComponent::force_I2C_communication(void) {
  /*Ensure RDY is HIGH at the moment*/
  if (!store_.iqs7222c_deviceRDY) {
    uint8_t force_communication = 0xFF;
    this->write_register(0, &force_communication, 1, STOP);
  }
  // if (!store_.iqs7222c_deviceRDY) {
  //   uint8_t data = 0xff;
  //   esphome::i2c::WriteBuffer buffers[2];
  //   buffers[0].data = &data;
  //   buffers[0].len = 1;
  //   bus_->writev(address_, buffers, 2, true);

  //   // /* Select the device with the address of "DEMO_IQS7222C_ADDR" and start
  //   //   communication. */
  //   // Wire.beginTransmission(_deviceAddress);

  //   // /* Write to memory address 0xFF that will prompt the IQS7222C to open a
  //   // communication window.*/
  //   // Wire.write(0xFF);

  //   // /* End the transmission, the user decides to STOP or RESTART. */
  //   // Wire.endTransmission(STOP);
  // }
}

void IQS7222CComponent::hard_reset_() {
  this->mclr_pin_->digital_write(true);
  delay(100);
  this->mclr_pin_->digital_write(false);
  delay(150);
  this->mclr_pin_->digital_write(true);
  delay(100);
}

void IQS7222CComponent::hard_comms_request_() {
  if (!store_.iqs7222c_deviceRDY) {
    // Knocking on the door to the IQS7222C.
    // To avoid triggering own interrupt procedure interrupts are disabled in the block.
    InterruptLock lock;
    this->interrupt_pin_->pin_mode(gpio::FLAG_OUTPUT);
    this->interrupt_pin_->setup();
    this->interrupt_pin_->digital_write(false);
    delay(5);
    this->interrupt_pin_->digital_write(true);
    this->interrupt_pin_->setup();
    this->interrupt_pin_->pin_mode(gpio::FLAG_INPUT);
  }
}

}  // namespace iqs7222c
}  // namespace esphome
