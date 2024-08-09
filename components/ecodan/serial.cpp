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
            // Discard bytes until we see one that might reasonably be
            // the first byte of a packet, complaining only once.
            if (msg.get_write_offset() == 0 && data != HEADER_MAGIC_A) {
                if (!skipping) {
                    ESP_LOGE(TAG, "Dropping serial data; header magic mismatch");
                    skipping = true;
                }
                continue;
            }
            skipping = false;

            // Add the byte to the packet.
            msg.append_byte(data);

            // If the header is now complete, check it for sanity.
            if (msg.get_write_offset() == HEADER_SIZE && !msg.verify_header()) {
                ESP_LOGI(TAG, "Serial port message appears invalid, skipping payload...");
                msg = Message();
                continue;
            }

            // If we don't yet have the full header, or if we do have the
            // header but not yet the full payload, keep going.
            if (msg.get_write_offset() <= HEADER_SIZE ||
                msg.get_write_offset() < msg.size()) {
                continue;
            }

            // Got full packet. Verify its checksum.
            if (!msg.verify_checksum()) {
                ESP_LOGI(TAG, "Serial port message checksum invalid");
                msg = Message();
                continue;
            }

            return true;
        }

        return false;
    }

}
}
