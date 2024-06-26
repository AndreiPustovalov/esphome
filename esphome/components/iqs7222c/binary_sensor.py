import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_CHANNEL
from . import iqs7222c_ns, IQS7222CComponent, CONF_IQS7222C_ID

DEPENDENCIES = ["iqs7222c"]
IQS7222CButton = iqs7222c_ns.class_("IQS7222CButton", binary_sensor.BinarySensor)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(IQS7222CButton).extend(
    {
        cv.GenerateID(CONF_IQS7222C_ID): cv.use_id(IQS7222CComponent),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=7),
    }
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    hub = await cg.get_variable(config[CONF_IQS7222C_ID])
    cg.add(var.set_channel(config[CONF_CHANNEL]))

    cg.add(hub.register_button(var))
