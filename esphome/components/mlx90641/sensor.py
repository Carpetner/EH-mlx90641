import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor#, text_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

CODEOWNERS = ["@yabadabadoo"]
DEPENDENCIES = ["i2c"]

CONF_AMBIENT = "ambient"
CONF_EMISSIVITY = "emissivity"
CONF_OBJECT = "object"
#CONF_FRAME = "frame

mlx90641_ns = cg.esphome_ns.namespace("mlx90641")
MLX90641Component = mlx90641_ns.class_(
    "MLX90641Component", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MLX90641Component),
            cv.Optional(CONF_AMBIENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_OBJECT): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ).extend(
                {
                    cv.Optional(CONF_EMISSIVITY, default=1.0): cv.percentage,
                }
            ),
#            cv.Optional(CONF_FRAME): text_sensor.text_sensor_schema(
#            ),              
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x33))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_AMBIENT in config:
        sens = await sensor.new_sensor(config[CONF_AMBIENT])
        cg.add(var.set_ambient_sensor(sens))

    if CONF_OBJECT in config:
        sens = await sensor.new_sensor(config[CONF_OBJECT])
        cg.add(var.set_object_sensor(sens))

        cg.add(var.set_emissivity(config[CONF_OBJECT][CONF_EMISSIVITY]))

#    if CONF_FRAME in config:
#        sens = await text_sensor.new_text_sensor(config[CONF_FRAME])
#        cg.add(var.set_frame_sensor(sens))
