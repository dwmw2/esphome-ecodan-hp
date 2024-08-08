import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart
from esphome import pins
from esphome.const import CONF_ID, CONF_RX_PIN, CONF_TX_PIN

CODEOWNERS = ["@gekkekoe"]

AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor", "uart"]

CONF_ECODAN_ID = "ecodan_id"

hub_ns = cg.esphome_ns.namespace('ecodan')

ECODAN = hub_ns.class_('EcodanHeatpump', cg.PollingComponent)
ECODAN_CLIMATE = hub_ns.class_('EcodanClimate', climate.Climate, cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(ECODAN),
        cv.Optional(CONF_RX_PIN, default=2): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_TX_PIN, default=1): pins.internal_gpio_output_pin_number,
    }
    ).extend(cv.polling_component_schema('1000ms')
    .extend(uart.UART_DEVICE_SCHEMA))


async def to_code(config):
    hp = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(hp, config)
    await uart.register_uart_device(hp, config)
