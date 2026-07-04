#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"

//static const char *const TAG = "dali";
static const bool DEBUG_LOG_RXTX = false; // NOTE: Will probably trigger WDT

using namespace esphome;
using namespace dali;

void DaliBusComponent::setup() {
    m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
    m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT);
    DALI_LOGI("DALI bus ready");

    if (m_discovery) {
        // Optional: reset devices on the bus so we are in a known-good state.
        // Can help if devices are not responding to anything.
        if (false) {
            this->resetBus();
            esp_task_wdt_reset();
        }

        if (dali.bus_manager.isControlGearPresent()) {
            DALI_LOGD("Detected control gear on bus");
        } else {
            DALI_LOGW("No control gear detected on bus!");
        }

        // for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
        //     if (m_addresses[i] != 0) {
        //         DALI_LOGD("Static config addr: %.2x", i);
        //     }
        // }

        if (this->m_initialize_addresses != DaliInitMode::DiscoverOnly) {
            if (this->m_initialize_addresses == DaliInitMode::InitializeAll) {
                DALI_LOGI("Randomizing addresses for *all* DALI devices");
                dali.bus_manager.initialize(ASSIGN_ALL); 
            } 
            else if (this->m_initialize_addresses == DaliInitMode::InitializeUnassigned) {
                // Only randomize devices without an assigned short address
                DALI_LOGI("Randomizing addresses for unassigned DALI devices");
                dali.bus_manager.initialize(ASSIGN_UNINITIALIZED); 
            }

            dali.bus_manager.randomize();
            dali.bus_manager.terminate();

            // Seem to need a delay to allow time for devices to randomize...
            delay(50);
        }

        DALI_LOGI("Begin device discovery...");
        dali.bus_manager.startAddressScan(); // All devices

        // Keep track of short addresses to detect duplicates
        bool duplicate_detected = false;
        bool is_discovered[ADDR_SHORT_MAX+1];
        for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
            is_discovered[i] = false;
        }

        uint8_t count = 0;
        short_addr_t short_addr = 0xFF;
        uint32_t long_addr = 0;
        while (dali.bus_manager.findNextAddress(short_addr, long_addr)) {
            count++;
            delay(1); // yield to ESP stack
            esp_task_wdt_reset();

            // if (short_addr == 0xFF) {
            //     if (this->m_initialize_addresses) {
                    
            //         //dali.bus_manager.programShortAddress(count);
            //         // short_addr_t new_addr = 1;
            //         // programShortAddress(new_addr);
            
            //         // port.sendSpecialCommand(DaliSpecialCommand::QUERY_SHORT_ADDRESS, 0);
            //         // out_short_addr = port.receiveBackwardFrame();
            
            //         // if (out_short_addr != new_addr) {
            //         //     DALI_LOGE("Could not program short address");
            //         //     out_short_addr = 0xFF;
            //         // }

            //         short_addr_t new_addr = count;

            //         dali.bus_manager.programShortAddress(new_addr);

            //         dali.port.sendSpecialCommand(DaliSpecialCommand::QUERY_SHORT_ADDRESS, 0);
            //         short_addr = dali.port.receiveBackwardFrame();
            
            //         if (short_addr != new_addr) {
            //             DALI_LOGE("  Could not program short address");
            //             continue;
            //         }
            //     }
            //     else {
            //         // You'll need to assign a short address before the device will respond to commands.
            //         // However it will still respond to BROADCAST brightness updates...
            //         DALI_LOGW("  No short address assigned!");
            //         continue;
            //     }
            // }

            if (short_addr <= ADDR_SHORT_MAX) {
                DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);

                // Duplicate detection
                if (is_discovered[short_addr]) {
                    if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                        DALI_LOGW("  WARNING: Duplicate short address detected!");
                        duplicate_detected = true;
                        // TODO: Maybe don't register the component in this case?
                        // Brightness control will work, but reported capabilities will not be correct.
                    }
                    else {
                        // Assign a new address for this
                        short_addr++;
                        DALI_LOGD("  Duplicate short address detected, assigning a new address: %.2x", short_addr);

                        if (!dali.bus_manager.programShortAddress(short_addr)) {
                            DALI_LOGE("  Could not program short address");
                            short_addr = 0xFF;
                            continue;
                        }
                    }
                }
                else {
                    is_discovered[short_addr] = true;
                }

                // Dynamic component creation (if not defined in YAML)
                if (m_addresses[short_addr]) {
                    DALI_LOGD("  Ignoring, already defined");
                }
                else {
                    m_addresses[short_addr] = long_addr;
                    create_light_component(short_addr, long_addr);
                }
            }
            else if (short_addr == 0xFF) {
                if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                    DALI_LOGI("  Device %.6x @ --", long_addr);
                    // You'll need to assign a short address before the device will respond to commands.
                    // However it will still respond to BROADCAST brightness updates...
                    DALI_LOGW("  No short address assigned!");
                    continue;
                }
                else {
                    short_addr = 1;
                    DALI_LOGI("  Assigning short address: %.2x", short_addr);

                    if (!dali.bus_manager.programShortAddress(short_addr)) {
                        DALI_LOGE("  Could not program short address");
                        short_addr = 0xFF;
                        continue;
                    }

                    DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);
                }
            }
        }

        DALI_LOGD("No more devices found!");
        dali.bus_manager.endAddressScan();

        if (duplicate_detected) {
            DALI_LOGW("Duplicate short addresses detected on the bus!");
            DALI_LOGW("  Devices may report inconsistent capabilities.");
            DALI_LOGW("  You should fix your address assignments.");
        }
    }
}

void DaliBusComponent::create_light_component(short_addr_t short_addr, uint32_t long_addr) {
    // Runtime creation of light entities is no longer supported by ESPHome (entity
    // registration is compile-time only). Discovery is therefore informational: log each
    // device that isn't already defined in YAML so the user can add a matching light block.
    DALI_LOGI("Discovered DALI device %.6x at short address %d", long_addr, short_addr);
    DALI_LOGI("  Add to YAML to control it:");
    DALI_LOGI("    light:");
    DALI_LOGI("      - platform: dali");
    DALI_LOGI("        name: \"DALI Light %d\"", short_addr);
    DALI_LOGI("        address: %d", short_addr);
}

void DaliBusComponent::loop() {
    // Once all components have finished setup, publish each lamp's real (already-read)
    // state to Home Assistant and enable normal bus writes. Done from the bus loop because
    // dynamically created lights don't reliably get their own loop() called. Iterating every
    // loop (rather than one-shot) also covers lights that register slightly late.
    for (auto* light : m_lights) {
        if (!light->boot_applied()) {
            light->apply_boot_state();
        }
    }

    // Periodic status: poll each lamp's live state once a minute, log it, and sync
    // external changes (hardware switches, other DALI masters) to Home Assistant.
    // Lamps are polled one per tick, spread across the minute, so a single loop pass
    // never blocks on more than one lamp's worth of bus queries.
    if (!m_lights.empty()) {
        uint32_t interval = 60000 / m_lights.size();
        uint32_t now = millis();
        if (now - m_last_status_poll >= interval) {
            m_last_status_poll = now;
            if (m_status_poll_index >= m_lights.size()) {
                m_status_poll_index = 0;
            }
            m_lights[m_status_poll_index]->refresh_lamp_state();
            m_status_poll_index++;
        }
    }
}

void DaliBusComponent::dump_config() {

}

#define QUARTER_BIT_PERIOD 208
#define HALF_BIT_PERIOD 416
#define BIT_PERIOD 833

void DaliBusComponent::writeBit(bool bit) {
    // NOTE: output is inverted - HIGH will pull the bus to 0V (logic low)
    bit = !bit;
    m_txPin->digital_write(bit ? LOW : HIGH);
    delayMicroseconds(HALF_BIT_PERIOD-6);
    m_txPin->digital_write(bit ? HIGH : LOW);
    delayMicroseconds(HALF_BIT_PERIOD-6);
}

void DaliBusComponent::writeByte(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        writeBit(b & 0x80);
        b <<= 1;
    }
}

void DaliBusComponent::resetBus() {
    DALI_LOGD("Resetting bus");
    m_txPin->digital_write(HIGH);
    delay(1000);
    m_txPin->digital_write(LOW);
}

void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("TX: %02x %02x", address, data);
        delayMicroseconds(BIT_PERIOD*8);
        //Serial.print("TX: "); Serial.print(address, HEX); Serial.print(" "); Serial.println(data, HEX);
    }

    {
        // This is timing critical
        InterruptLock lock;

        writeBit(1); // START bit
        writeByte(address);
        writeByte(data);
        m_txPin->digital_write(LOW);
    }

    // Non critical delay
    delayMicroseconds(HALF_BIT_PERIOD*2);
    delayMicroseconds(BIT_PERIOD*4); // Optional, for clarity in scope trace
}

uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    unsigned long startTime = millis();

    // Wait for a *valid* START bit, not just a high level: the RX line can glitch or
    // sit high while the bus recovers from our own forward frame, and decoding that
    // as data (typically 0xFF, all bits high) turned query NACKs into positive
    // replies - e.g. an off lamp that simply doesn't answer QUERY_LAMP_POWER_ON was
    // read as answering "yes".
    //
    // Decoding is edge-synchronized: Manchester guarantees a transition in the middle
    // of every bit, and the sample clock is re-synced to each one. Open-loop sampling
    // at our own nominal clock drifted into the second (complementary) half-bit on
    // gear whose bit clock runs slightly fast/slow, corrupting trailing bits to 1s
    // (observed as e.g. status 0x80 reading back as 0x9f/0xbf/0xff on some brands).
    while (true) {
        if (millis() - startTime >= timeout_ms) {
            if (DEBUG_LOG_RXTX) {
                DALI_LOGD("RX: 00 (NACK)");
            }
            return 0;
        }

        if (m_rxPin->digital_read() == LOW) {
            continue; // Idle, keep waiting for a rising edge
        }

        // Rising edge seen: validate the start bit and decode (timing critical)
        uint8_t data = 0;
        bool valid = true;
        {
            InterruptLock lock;

            uint32_t t0 = micros();

            // A start bit is a '1': high first half, falling edge at mid-bit.
            delayMicroseconds(QUARTER_BIT_PERIOD);
            if (m_rxPin->digital_read() != HIGH) {
                continue; // Glitch shorter than a half-bit; lock releases, keep scanning
            }

            // Find the mid-bit falling edge to synchronize the bit clock
            uint32_t t_mid = 0;
            while ((uint32_t)(micros() - t0) <= (HALF_BIT_PERIOD + QUARTER_BIT_PERIOD)) {
                if (m_rxPin->digital_read() == LOW) {
                    t_mid = micros();
                    break;
                }
            }
            if (t_mid == 0) {
                valid = false; // No falling edge in time: stuck-high line, not a start bit
            }

            for (int i = 0; valid && i < 8; i++) {
                // Sample the first half of this bit (3/4 period after the last mid-bit
                // transition); the level there is the bit value.
                while ((uint32_t)(micros() - t_mid) < (BIT_PERIOD - QUARTER_BIT_PERIOD)) { }
                bool level = m_rxPin->digital_read();
                data = (data << 1) | (level ? 1 : 0);

                // Re-sync on the mid-bit transition that must follow (a possible
                // bit-boundary transition at +1/2 period is already behind us here).
                bool flipped = false;
                while ((uint32_t)(micros() - t_mid) <= (BIT_PERIOD + HALF_BIT_PERIOD)) {
                    if (m_rxPin->digital_read() != level) {
                        flipped = true;
                        t_mid = micros();
                        break;
                    }
                }
                if (!flipped) {
                    valid = false; // Framing error: Manchester always transitions mid-bit
                }
            }

            if (valid) {
                delayMicroseconds(BIT_PERIOD*2); // Wait for STOP bits
            }
        }

        if (valid) {
            if (DEBUG_LOG_RXTX) {
                DALI_LOGD("RX: %02x", data);
            }
            // Minimum time before we can send another forward frame
            delayMicroseconds(BIT_PERIOD*8);
            return data;
        }
        // Not a valid frame (glitch or framing error): keep scanning until timeout.
        // The InterruptLock is released between attempts so WiFi/etc can still run.
    }
}
