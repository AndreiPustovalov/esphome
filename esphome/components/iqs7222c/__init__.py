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

CONF_PROX_THRESHOLD = "prox_threshold"
CONF_ENTER_EXIT = "enter_exit"
CONF_TOUCH_THRESHOLD = "touch_threshold"
CONF_TOUCH_HYSTERESIS = "touch_hysteresis"
CONF_PROX_EVENT_TIMEOUT = "prox_event_timeout"
CONF_TOUCH_EVENT_TIMEOUT = "touch_event_timeout"


IQS_BUTTON_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_PROX_THRESHOLD, default=0x0A): cv.int_range(min=0, max=255),
        cv.Optional(CONF_ENTER_EXIT, default=0x12): cv.int_range(min=0, max=255),
        cv.Optional(CONF_TOUCH_THRESHOLD, default=0x32): cv.int_range(min=0, max=255),
        cv.Optional(CONF_TOUCH_HYSTERESIS, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_PROX_EVENT_TIMEOUT, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_TOUCH_EVENT_TIMEOUT, default=0): cv.int_range(min=0, max=255),
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

    if CONF_BUTTON_0 in config:
        b = config[CONF_BUTTON_0]
        cg.add(
            var.set_button_config(
                0,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_1 in config:
        b = config[CONF_BUTTON_1]
        cg.add(
            var.set_button_config(
                1,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_2 in config:
        b = config[CONF_BUTTON_2]
        cg.add(
            var.set_button_config(
                2,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_3 in config:
        b = config[CONF_BUTTON_3]
        cg.add(
            var.set_button_config(
                3,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_4 in config:
        b = config[CONF_BUTTON_5]
        cg.add(
            var.set_button_config(
                4,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_5 in config:
        b = config[CONF_BUTTON_5]
        cg.add(
            var.set_button_config(
                5,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_6 in config:
        b = config[CONF_BUTTON_6]
        cg.add(
            var.set_button_config(
                6,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_7 in config:
        b = config[CONF_BUTTON_7]
        cg.add(
            var.set_button_config(
                7,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_8 in config:
        b = config[CONF_BUTTON_8]
        cg.add(
            var.set_button_config(
                8,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )
    if CONF_BUTTON_9 in config:
        b = config[CONF_BUTTON_9]
        cg.add(
            var.set_button_config(
                9,
                b[CONF_PROX_THRESHOLD],
                b[CONF_ENTER_EXIT],
                b[CONF_TOUCH_THRESHOLD],
                b[CONF_TOUCH_HYSTERESIS],
                b[CONF_PROX_EVENT_TIMEOUT],
                b[CONF_TOUCH_EVENT_TIMEOUT],
            )
        )

    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
