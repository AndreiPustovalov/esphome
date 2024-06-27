import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID
from esphome import pins

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["binary_sensor", "output"]
CODEOWNERS = ["@latonita"]

CONF_RDY_PIN = "rdy_pin"
CONF_MCLR_PIN = "mclr_pin"
CONF_TEST_MODE = "test_mode"
CONF_INIT_DELAY = "init_delay"

iqs7222c_ns = cg.esphome_ns.namespace("iqs7222c")
CONF_IQS7222C_ID = "iqs7222c_id"
IQS7222CComponent = iqs7222c_ns.class_("IQS7222CComponent", cg.Component, i2c.I2CDevice)

CONF_BUTTON_0 = "button_0"
CONF_BUTTON_1 = "button_1"
CONF_BUTTON_2 = "button_2"
CONF_BUTTON_3 = "button_3"
CONF_BUTTON_4 = "button_4"
CONF_BUTTON_5 = "button_5"
CONF_BUTTON_6 = "button_6"
CONF_BUTTON_7 = "button_7"
CONF_BUTTON_8 = "button_8"
CONF_BUTTON_9 = "button_9"

BUTTONS = [
    CONF_BUTTON_0,
    CONF_BUTTON_1,
    CONF_BUTTON_2,
    CONF_BUTTON_3,
    CONF_BUTTON_4,
    CONF_BUTTON_5,
    CONF_BUTTON_6,
    CONF_BUTTON_7,
    CONF_BUTTON_8,
    CONF_BUTTON_9,
]

CONF_PROX_THRESHOLD = "prox_threshold"
CONF_ENTER_EXIT = "enter_exit"
CONF_TOUCH_THRESHOLD = "touch_threshold"
CONF_TOUCH_HYSTERESIS = "touch_hysteresis"
CONF_PROX_EVENT_TIMEOUT = "prox_event_timeout"
CONF_TOUCH_EVENT_TIMEOUT = "touch_event_timeout"

CONF_CH_0 = "ch_0"
CONF_CH_1 = "ch_1"
CONF_CH_2 = "ch_2"
CONF_CH_3 = "ch_3"
CONF_CH_4 = "ch_4"
CONF_CH_5 = "ch_5"
CONF_CH_6 = "ch_6"
CONF_CH_7 = "ch_7"
CONF_CH_8 = "ch_8"
CONF_CH_9 = "ch_9"

CHANNELS = [
    CONF_CH_0,
    CONF_CH_1,
    CONF_CH_2,
    CONF_CH_3,
    CONF_CH_4,
    CONF_CH_5,
    CONF_CH_6,
    CONF_CH_7,
    CONF_CH_8,
    CONF_CH_9,
]

CONF_SETUP = "setup"
CONF_ATI_SETTINGS = "ati_settings"
CONF_MULTIPLIERS = "multipliers"
CONF_ATI_COMPENSATION = "ati_compensation"
CONF_REF_PTR = "ref_ptr"
CONF_REFMASK = "refmask"


IQS_BUTTON_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_PROX_THRESHOLD, default=0x0A): cv.hex_uint8_t,
        cv.Optional(CONF_ENTER_EXIT, default=0x12): cv.hex_uint8_t,
        cv.Optional(CONF_TOUCH_THRESHOLD, default=0x32): cv.hex_uint8_t,
        cv.Optional(CONF_TOUCH_HYSTERESIS, default=0): cv.hex_uint8_t,
        cv.Optional(CONF_PROX_EVENT_TIMEOUT, default=0): cv.hex_uint8_t,
        cv.Optional(CONF_TOUCH_EVENT_TIMEOUT, default=0): cv.hex_uint8_t,
    }
)

IQS_CHANNEL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SETUP): cv.hex_uint16_t,
        cv.Required(CONF_ATI_SETTINGS): cv.hex_uint16_t,
        cv.Required(CONF_MULTIPLIERS): cv.hex_uint16_t,
        cv.Required(CONF_ATI_COMPENSATION): cv.hex_uint16_t,
        cv.Required(CONF_REF_PTR): cv.hex_uint16_t,
        cv.Required(CONF_REFMASK): cv.hex_uint16_t,
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IQS7222CComponent),
            cv.Required(CONF_RDY_PIN): pins.internal_gpio_input_pin_schema,
            cv.Required(CONF_MCLR_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_TEST_MODE, default=False): cv.boolean,
            cv.Optional(
                CONF_INIT_DELAY, default=0
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_BUTTON_0): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_1): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_2): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_3): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_4): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_5): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_6): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_7): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_8): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_BUTTON_9): IQS_BUTTON_SCHEMA,
            cv.Optional(CONF_CH_0): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_1): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_2): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_3): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_4): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_5): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_6): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_7): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_8): IQS_CHANNEL_SCHEMA,
            cv.Optional(CONF_CH_9): IQS_CHANNEL_SCHEMA,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x44))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_rdy_pin(await cg.gpio_pin_expression(config[CONF_RDY_PIN])))
    cg.add(var.set_mclr_pin(await cg.gpio_pin_expression(config[CONF_MCLR_PIN])))
    cg.add(var.set_enable_test_mode(config[CONF_TEST_MODE]))
    cg.add(var.set_init_delay_ms(config[CONF_INIT_DELAY]))

    # iterate through BUTTONS array
    for idx, button in enumerate(BUTTONS):
        if button in config:
            b = config[button]
            cg.add(
                var.set_button_config(
                    idx,
                    b[CONF_PROX_THRESHOLD],
                    b[CONF_ENTER_EXIT],
                    b[CONF_TOUCH_THRESHOLD],
                    b[CONF_TOUCH_HYSTERESIS],
                    b[CONF_PROX_EVENT_TIMEOUT],
                    b[CONF_TOUCH_EVENT_TIMEOUT],
                )
            )

    # iterate through CHANNELS array
    for idx, channel in enumerate(CHANNELS):
        if channel in config:
            c = config[channel]
            cg.add(
                var.set_channel_config(
                    idx,
                    c[CONF_SETUP],
                    c[CONF_ATI_SETTINGS],
                    c[CONF_MULTIPLIERS],
                    c[CONF_ATI_COMPENSATION],
                    c[CONF_REF_PTR],
                    c[CONF_REFMASK],
                )
            )

    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
