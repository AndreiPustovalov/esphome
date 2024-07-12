from esphome.cpp_generator import MockObj

global_ns = MockObj("", "")
void = global_ns.namespace("void")
nullptr = global_ns.namespace("nullptr")
float_ = global_ns.namespace("float")
double = global_ns.namespace("double")
bool_ = global_ns.namespace("bool")
int_ = global_ns.namespace("int")
std_ns = global_ns.namespace("std")
std_shared_ptr = std_ns.class_("shared_ptr")
std_string = std_ns.class_("string")
std_string_ref = std_ns.namespace("string &")
std_vector = std_ns.class_("vector")
uint8 = global_ns.namespace("uint8_t")
uint16 = global_ns.namespace("uint16_t")
uint32 = global_ns.namespace("uint32_t")
uint64 = global_ns.namespace("uint64_t")
int16 = global_ns.namespace("int16_t")
int32 = global_ns.namespace("int32_t")
int64 = global_ns.namespace("int64_t")
size_t = global_ns.namespace("size_t")
const_char_ptr = global_ns.namespace("const char *")
NAN = global_ns.namespace("NAN")
esphome_ns = global_ns  # using namespace esphome;
App = esphome_ns.App
EntityBase = esphome_ns.class_("EntityBase")
Component = esphome_ns.class_("Component")
ComponentPtr = Component.operator("ptr")
PollingComponent = esphome_ns.class_("PollingComponent", Component)
Application = esphome_ns.class_("Application")
optional = esphome_ns.class_("optional")
arduino_json_ns = global_ns.namespace("ArduinoJson")
JsonObject = arduino_json_ns.class_("JsonObject")
JsonObjectConst = arduino_json_ns.class_("JsonObjectConst")
Controller = esphome_ns.class_("Controller")
GPIOPin = esphome_ns.class_("GPIOPin")
InternalGPIOPin = esphome_ns.class_("InternalGPIOPin", GPIOPin)
gpio_ns = esphome_ns.namespace("gpio")
gpio_Flags = gpio_ns.enum("Flags", is_class=True)
EntityCategory = esphome_ns.enum("EntityCategory")
Parented = esphome_ns.class_("Parented")
ESPTime = esphome_ns.struct("ESPTime")
