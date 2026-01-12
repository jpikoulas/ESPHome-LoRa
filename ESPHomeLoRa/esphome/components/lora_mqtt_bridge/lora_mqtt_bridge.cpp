#include "lora_mqtt_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include "esphome/components/mqtt/mqtt_client.h"
#include "LoRa.h"
#include <iostream>
#include <sstream>
volatile bool esphome::lora_mqtt_bridge::Lora_MQTT_BridgeComponent::receivedLoRaP = false;
volatile int esphome::lora_mqtt_bridge::Lora_MQTT_BridgeComponent::_packet_size = 0;

// External debug counters from LoRa.cpp (declared in global namespace)
extern volatile uint32_t g_lora_irq_count;
extern volatile uint32_t g_lora_packet_count;
extern volatile int g_lora_last_parse_result;

namespace esphome
{
    namespace lora_mqtt_bridge
    {
        static const char *const TAG = "lora_mqtt_bridge.sensor";

        static uint32_t last_debug_time = 0;
        static uint32_t last_irq_count = 0;

        void Lora_MQTT_BridgeComponent::loop()
        {
            // Print debug status every 30 seconds
            uint32_t now = millis();
            if (now - last_debug_time >= 30000) {
                last_debug_time = now;
                ESP_LOGI(TAG, "LoRa status: IRQ count=%lu, packets=%lu, last_parse=%d, flag=%d",
                         (unsigned long)g_lora_irq_count, (unsigned long)g_lora_packet_count,
                         g_lora_last_parse_result, (int)receivedLoRaP);
                if (g_lora_irq_count == last_irq_count) {
                    ESP_LOGW(TAG, "No IRQs received in last 30s - check DIO1/IRQ wiring!");
                }
                last_irq_count = g_lora_irq_count;
            }

            if (receivedLoRaP)
            {
                receivedLoRaP = false;
                ESP_LOGI(TAG, "*** LoRa packet received! Size: %d bytes ***", _packet_size);

                char received_string[251];
                char config_topic[] = "%s/sensor/%s/%s/config";
                char sensor_topic[] = "%s/sensor/%s/state";
                char binary_config_topic[] = "%s/binary_sensor/%s/%s/config";
                char binary_sensor_topic[] = "%s/binary_sensor/%s/state";

                char topic[250];
                char macStr[18];
                StaticJsonDocument<500> doc;
                JsonObject dev;
                std::string json;
                std::string message_type;
                mqtt::MQTTDiscoveryInfo discovery_info;

                // received data
                memset(&received_string, 0, sizeof(received_string));
                for (int i = 0; i < _packet_size; i++)
                {
                    received_string[i] = (char)LoRa.read();
                }

                ESP_LOGI(TAG, "Raw received data: '%s'", received_string);
                ESP_LOGI(TAG, "RSSI: %d dBm", LoRa.packetRssi());

                // tokenize the received string
                char *tokens[13];
                int argc;
                this->split(tokens, &argc, received_string, ':', 1);

                ESP_LOGI(TAG, "Parsed %d tokens (expecting 11)", argc);

                // if we didn't get 11 elements, this wasn't a message from our sensors
                if (argc != 11)
                {
                    ESP_LOGW(TAG, "Invalid packet format - expected 11 tokens, got %d. Ignoring.", argc);
                    return;
                }

                ESP_LOGI(TAG, "Valid packet: %s:%s:%s:%s:%s:%s:%s:%s:%s:%s:%s", tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5], tokens[6], tokens[7], tokens[8], tokens[9], tokens[10]);

                // check for binary_sensor or sensor
                message_type = tokens[2];
                if (message_type.compare("binary_sensor") == 0)
                {
                    if (strlen(tokens[3]) != 0)
                    {
                        doc["name"] = tokens[3];
                    }
                    if (strlen(tokens[0]) != 0)
                    {
                        std::string stat_t = tokens[0];
                        stat_t += "/binary_sensor/";
                        stat_t += tokens[3];
                        stat_t += "/state";
                        doc["stat_t"] = stat_t;
                    }
                    if (strlen(tokens[0]) != 0)
                    {
                        std::string uniq_id = tokens[0];
                        uniq_id += "_";
                        uniq_id += tokens[3];
                        doc["uniq_id"] = uniq_id;
                    }
                    dev = doc.createNestedObject("dev");
                    if (strlen(tokens[0]) != 0)
                    {
                        dev["ids"] = tokens[0];
                        dev["name"] = tokens[0];
                    }
                    dev["sw"] = tokens[8];
                    dev["mdl"] = tokens[9];
                    dev["mf"] = "espressif";
                    serializeJson(doc, json);

                    // make and send the config topic
                    discovery_info = mqtt::global_mqtt_client->get_discovery_info();
                    memset(&topic, 0, sizeof(topic));
                    snprintf(topic, sizeof(topic), binary_config_topic, discovery_info.prefix.c_str(), tokens[0], tokens[3]);
                    mqtt::global_mqtt_client->publish(topic, json.c_str(), json.length(), 2, true);

                    // make and send the state topic
                    memset(&topic, 0, sizeof(topic));
                    snprintf(topic, sizeof(topic), binary_sensor_topic, tokens[0], tokens[3]);
                    mqtt::global_mqtt_client->publish(topic, tokens[5], strlen(tokens[5]), 2, true);
                }
                else
                {
                    if (strlen(tokens[1]) != 0)
                    {
                        doc["dev_cla"] = tokens[1];
                    }
                    if (strlen(tokens[4]) != 0)
                    {
                        doc["unit_of_meas"] = tokens[4];
                    }
                    if (strlen(tokens[2]) != 0)
                    {
                        doc["stat_cla"] = tokens[2];
                    }
                    if (strlen(tokens[3]) != 0)
                    {
                        doc["name"] = tokens[3];
                    }
                    if (strlen(tokens[6]) != 0)
                    {
                        std::string icon = tokens[6];
                        icon += ":";
                        icon += tokens[7];
                        doc["icon"] = icon;
                    }
                    if (strlen(tokens[0]) != 0)
                    {
                        std::string stat_t = tokens[0];
                        stat_t += "/sensor/";
                        stat_t += tokens[3];
                        stat_t += "/state";
                        doc["stat_t"] = stat_t;
                    }
                    if (strlen(tokens[0]) != 0)
                    {
                        std::string uniq_id = tokens[0];
                        uniq_id += "_";
                        uniq_id += tokens[3];
                        doc["uniq_id"] = uniq_id;
                    }
                    dev = doc.createNestedObject("dev");
                    if (strlen(tokens[0]) != 0)
                    {
                        dev["ids"] = tokens[0];
                        dev["name"] = tokens[0];
                    }
                    dev["sw"] = tokens[8];
                    dev["mdl"] = tokens[9];
                    dev["mf"] = "espressif";
                    serializeJson(doc, json);

                    // make and send the config topic
                    discovery_info = mqtt::global_mqtt_client->get_discovery_info();
                    memset(&topic, 0, sizeof(topic));
                    snprintf(topic, sizeof(topic), config_topic, discovery_info.prefix.c_str(), tokens[0], tokens[3]);
                    mqtt::global_mqtt_client->publish(topic, json.c_str(), json.length(), 2, true);

                    // make and send the state topic
                    memset(&topic, 0, sizeof(topic));
                    snprintf(topic, sizeof(topic), sensor_topic, tokens[0], tokens[3]);
                    mqtt::global_mqtt_client->publish(topic, tokens[5], strlen(tokens[5]), 2, true);
                }

                // create RSSI message
                json = "";
                doc["name"] = "rssi";
                doc["dev_cla"] = "SIGNAL_STRENGTH";
                doc["unit_of_meas"] = "dBm";
                doc["stat_cla"] = "measurement";
                doc["icon"] = "mdi:wifi";

                std::string stat_t = tokens[0];
                stat_t += "/sensor/";
                stat_t += "rssi";
                stat_t += "/state";
                doc["stat_t"] = stat_t;

                std::string uniq_id = tokens[0];
                ;
                uniq_id += "_";
                uniq_id += "rssi";
                doc["uniq_id"] = uniq_id;

                serializeJson(doc, json);

                // make and send the rssi config topic
                discovery_info = mqtt::global_mqtt_client->get_discovery_info();
                memset(&topic, 0, sizeof(topic));
                snprintf(topic, sizeof(topic), config_topic, discovery_info.prefix.c_str(), tokens[0], "rssi");
                mqtt::global_mqtt_client->publish(topic, json.c_str(), json.length(), 2, true);

                // make and send the rssi state topic
                memset(&topic, 0, sizeof(topic));
                snprintf(topic, sizeof(topic), sensor_topic, tokens[0], "rssi");
                std::string last_rssi_str = std::to_string(LoRa.packetRssi());
                mqtt::global_mqtt_client->publish(topic, last_rssi_str.c_str(), last_rssi_str.length(), 2, true);
            }
        }

        float Lora_MQTT_BridgeComponent::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

        void Lora_MQTT_BridgeComponent::setup()
        {
            ESP_LOGI(TAG, "Setting up LoRa MQTT Bridge...");
            auto *cs_internal = (InternalGPIOPin *)_cs;
            int cs_pin = cs_internal->get_pin();
            auto *reset_internal = (InternalGPIOPin *)_reset;
            int reset_pin = reset_internal->get_pin();
            auto *dio0_internal = (InternalGPIOPin *)_dio0;
            int dio0_pin = dio0_internal->get_pin();

            // DIO1 is optional (only needed for SX1262/SX1268)
            int dio1_pin = -1;
            if (_dio1 != nullptr) {
                auto *dio1_internal = (InternalGPIOPin *)_dio1;
                dio1_pin = dio1_internal->get_pin();
            }

            ESP_LOGI(TAG, "LoRa pins: CS=%d, RST=%d, DIO0/IRQ=%d, DIO1/BUSY=%d", cs_pin, reset_pin, dio0_pin, dio1_pin);
            ESP_LOGI(TAG, "LoRa config: chip_type=%d, freq=%ld, bw=%ld, sf=%ld, cr=%ld, sync=0x%02lX",
                     _chip_type, _frequency, _bandwidth, _spread, _coding, _sync);

            // Set chip type before initialization
            LoRa.setChipType((LoRaChipType)_chip_type);
            LoRa.setPins(cs_pin, reset_pin, dio0_pin, dio1_pin);
            if (!LoRa.begin(_frequency))
            {
                this->mark_failed();
                ESP_LOGE(TAG, "Error initializing LoRa - check wiring and pins!");
                return;
            }
            ESP_LOGI(TAG, "LoRa radio initialized successfully");
            LoRa.setSyncWord(_sync);
            LoRa.setCodingRate4(_coding);
            LoRa.setSpreadingFactor(_spread);
            LoRa.setSignalBandwidth(_bandwidth);
            LoRa.onReceive(Lora_MQTT_BridgeComponent::call_on_data_recv_callback);
            LoRa.receive();
            ESP_LOGI(TAG, "LoRa MQTT Bridge ready - listening for packets");
        }

        void Lora_MQTT_BridgeComponent::receivecallback(int packetSize)
        {
            _packet_size = packetSize;
            receivedLoRaP = true;
            // Note: Can't use ESP_LOG in ISR, but this sets the flag for loop() to process
        }

        void Lora_MQTT_BridgeComponent::call_on_data_recv_callback(int packetSize)
        {
            Lora_MQTT_BridgeComponent().receivecallback(packetSize);
        }

        // Static counter for periodic status
        static uint32_t last_status_time = 0;

        void Lora_MQTT_BridgeComponent::split(char **argv, int *argc, char *string, const char delimiter, int allowempty)
        {
            *argc = 0;
            do
            {
                if (*string && (*string != delimiter || allowempty))
                {
                    argv[(*argc)++] = string;
                }
                while (*string && *string != delimiter)
                    string++;
                if (*string)
                    *string++ = 0;
                if (!allowempty)
                    while (*string && *string == delimiter)
                        string++;
            } while (*string);
        }
    } // namespace lora_mqtt_bridge
} // namespace esphome
