import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_INTERRUPT_PIN
from esphome import pins

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["binary_sensor", "output"]
CODEOWNERS = ["@latonita"]

iqs7222c_ns = cg.esphome_ns.namespace("iqs7222c")
CONF_IQS7222C_ID = "iqs7222c_id"
IQS7222CComponent = iqs7222c_ns.class_("IQS7222CComponent", cg.Component, i2c.I2CDevice)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IQS7222CComponent),
            cv.Optional(CONF_INTERRUPT_PIN): pins.internal_gpio_input_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x44))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(
        var.set_interrupt_pin(await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN]))
    )

    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
