#include "ecodan.h"

#if ARDUINO_ARCH_ESP32
#include <esp_task_wdt.h>
#endif

namespace esphome {
namespace ecodan 
{
    bool EcodanHeatpump::serial_tx(uart::UARTComponent *uart, Message& msg)
    {
        if (!uart)
        {
            ESP_LOGE(TAG, "Serial connection unavailable for tx");
            return false;
        }
#if 0
        if (port.availableForWrite() < msg.size())
        {
            ESP_LOGI(TAG, "Serial tx buffer size: %u", port.availableForWrite());
            return false;
        }
#endif
        msg.set_checksum();
        {
            std::lock_guard<std::mutex> lock{portWriteMutex};
            uart->write_array(msg.buffer(), msg.size());
        }
        //port.flush(true);

        //ESP_LOGV(TAG, msg.debug_dump_packet().c_str());

        return true;
    }

    bool EcodanHeatpump::serial_rx(uart::UARTComponent *uart, Message& msg)
    {
        uint8_t data;
        bool skipping = false;

        while (uart->available() && uart->read_byte(&data)) {

            if (data == HEADER_MAGIC_A)
                skipping = false;

            switch (msg.append_byte(data)) {
            case MsgValid::MESSAGE_SKIPPED_BYTE:
                skipping = true;
                if (!skipping) {
                    ESP_LOGE(TAG, "Dropping serial data; header magic mismatch");
                    skipping = true;
                }
                break;

            case MsgValid::MESSAGE_HEADER_INVALID:
                ESP_LOGI(TAG, "Serial port message appears invalid, skipping payload...");
                msg = Message();
                break;

            case MsgValid::MESSAGE_CHECKSUM_INVALID:
                ESP_LOGI(TAG, "Serial port message checksum invalid");
                msg = Message();
                continue;

            case MsgValid::MESSAGE_EXTENDED:
                continue;

            case MsgValid::MESSAGE_COMPLETED:
                return true;

            case MsgValid::MESSAGE_ALREADY_DONE: /* Should never happen */
                ESP_LOGE(TAG, "Trying to extend already-complete message");
                msg = Message();
                break;
            }
        }
        return false;
    }

}
}
