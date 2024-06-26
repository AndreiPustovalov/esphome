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

    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
