
#include <esphome.h>
#include "esphome_dali_light.h"
#include "esphome/core/log.h"

using namespace esphome;
using namespace dali;

using namespace esphome::light;

static const char *const TAG = "dali.light";

#define DALI_MAX_BRIGHTNESS_F (254.0f)

void dali::DaliLight::setup_state(light::LightState *state) {
    // Initialization code for DaliLight
    this->state_ = state;

    // Exclude broadcast and group addresses
    if ((this->address_ != ADDR_BROADCAST) && ((this->address_ & ADDR_GROUP_MASK) == 0)) {
        ESP_LOGD(TAG, "Querying DALI device capabilities...");
        if (bus->dali.isDevicePresent(address_)) {
            ESP_LOGD(TAG, "DALI[%.2x] Is Present", address_);

            // Query the min/max brightness range. The bit-banged bus can occasionally NACK,
            // so retry a few times before falling back to the spec defaults. A bad (zero)
            // range would otherwise make the "on" command map to DALI level 0 (== OFF),
            // i.e. the lamp could be turned off but never on.
            uint8_t min_level = 0, max_level = 0;
            for (int attempt = 0; attempt < 3; attempt++) {
                min_level = bus->dali.lamp.getMinLevel(address_);
                max_level = bus->dali.lamp.getMaxLevel(address_);
                if (min_level != 0 && max_level != 0 && min_level <= max_level) {
                    break;
                }
            }
            if (min_level == 0 || max_level == 0 || min_level > max_level) {
                ESP_LOGW(TAG, "DALI[%.2x] Invalid min/max (%d/%d), using defaults 1/254",
                         address_, min_level, max_level);
                min_level = 1;
                max_level = 254;
            }
            this->dali_level_min_ = min_level;
            this->dali_level_max_ = max_level;
            this->dali_level_range_ = (float)(dali_level_max_ - this->dali_level_min_ + 1);
            ESP_LOGD(TAG, "Reported min:%d max:%d", this->dali_level_min_, this->dali_level_max_);

            // NOTE: Some DALI controllers report their device type is LED(6) even though they do also support color temperature,
            // so let's explicitly check if they respond to this:
            this->tc_supported_ = bus->dali.color.isTcCapable(address_);
            if (tc_supported_) {
                ESP_LOGD(TAG, "DALI[%.2x] Supports color temperature", address_);

                // TODO: Don't seem to be getting the full range here?
                // Tc(cool)=153, Tc(warm)=370
                uint16_t coolest = bus->dali.color.queryParameter(address_, DaliColorParam::ColourTemperatureTcCoolest);
                uint16_t warmest = bus->dali.color.queryParameter(address_, DaliColorParam::ColourTemperatureTcWarmest);

                ESP_LOGD(TAG, "Tc(cool)=%d, Tc(warm)=%d", coolest, warmest);
                if (coolest > COLOR_MIREK_WARMEST || warmest > COLOR_MIREK_WARMEST) {
                    ESP_LOGW(TAG, "Tc min/max is out of range!");
                } else {
                    // Store reported coolest/warmest mired values for mapping.
                    // NOTE: Not updating the configuration-provided warm/cool values, those are for UI only.
                    // Ultimately we don't really want to trust the mired range reported by the dimmer
                    // as it depends on the LED strip attached. So we map the UI range into the reported range.
                    this->dali_tc_coolest_ = (float)coolest;
                    //this->dali_tc_warmest_ = (float)warmest;
                }
            }
            else {
                ESP_LOGD(TAG, "Does not support color temperature");
            }

            ESP_LOGD(TAG, "Sending configuration to device...");

            if (this->brightness_curve_.has_value()) {
                switch (this->brightness_curve_.value()) {
                    case DaliLedDimmingCurve::LOGARITHMIC: ESP_LOGD(TAG, "Setting brightness curve to LOGARITHMIC"); break;
                    case DaliLedDimmingCurve::LINEAR:      ESP_LOGD(TAG, "Setting brightness curve to LINEAR"); break;
                }
                bus->dali.led.setDimmingCurve(address_, this->brightness_curve_.value());
            }

            if (this->fade_rate_.has_value()) {
                ESP_LOGD(TAG, "Setting fade rate: %d", this->fade_rate_.value());
                bus->dali.lamp.setFadeRate(0, this->fade_rate_.value());
            }
            if (this->fade_time_.has_value()) {
                ESP_LOGD(TAG, "Setting fade time: %d", this->fade_time_.value());
                bus->dali.lamp.setFadeTime(0, this->fade_time_.value());
            }

            // bus->dali.lamp.setMinLevel(address_, 1);
            // bus->dali.lamp.setMaxLevel(address_, 254);

            // Query the lamp's actual state so we can reflect it (and NOT overwrite it) on
            // boot. We deliberately do not write the bus here - apply_boot_state() will
            // publish this to Home Assistant once setup completes.
            uint8_t current_level = bus->dali.lamp.getCurrentLevel(address_);

            // 0xFF (MASK) means "unknown" - treat as off. Level 0 is off.
            bool is_on = (current_level != 0 && current_level != 0xFF);
            this->boot_state_.state = is_on;
            if (is_on) {
                // Inverse of write_state()'s min..max mapping, clamped to 0..1.
                float brightness = 1.0f;
                if (this->dali_level_max_ > this->dali_level_min_) {
                    brightness = (float)(current_level - this->dali_level_min_) /
                                 (float)(this->dali_level_max_ - this->dali_level_min_);
                    if (brightness < 0.0f) brightness = 0.0f;
                    if (brightness > 1.0f) brightness = 1.0f;
                }
                this->boot_state_.brightness = brightness;
                ESP_LOGD(TAG, "Restore brightness level: %.2f (raw %d)", brightness, current_level);
            }

            if (tc_supported_) {
                uint16_t current_temperature = bus->dali.color.getColorTemperature(address_);
                if (current_temperature != 0) {
                    // Convert mireds to 0..1 range
                    this->boot_state_.color_temp =
                        (current_temperature - dali_tc_coolest_) / (dali_tc_warmest_ - dali_tc_coolest_);
                    ESP_LOGD(TAG, "Restore colour temperature: %.2f", this->boot_state_.color_temp);
                }
            }

            // The bus component drives apply_boot_state() after all setup completes, which
            // publishes this state to Home Assistant without writing to the bus.
            bus->register_light(this);
        }
        else {
            ESP_LOGW(TAG, "DALI device at addr %.2x not found!", address_);
            // Device not found: don't suppress writes (nothing to restore).
            this->writes_enabled_ = true;
            this->boot_applied_ = true;
        }

        //bus->dali.dumpStatusForDevice(address_);
    }
    else {
        // Broadcast / group addresses: we cannot read a single state back, so don't try to
        // restore and don't suppress writes.
        // TODO: How do we detect color temperature support for broadcast and group addresses?
        this->writes_enabled_ = true;
        this->boot_applied_ = true;
    }


    // if (this->color_mode_.has_value()) {
    //     if (this->color_mode_.value() == DaliColorMode::COLOR_TEMPERATURE) {
    //         tc_supported_ = true;
    //         ESP_LOGD(TAG, "Override: enable color temperature support");
    //     } else {
    //         tc_supported_ = false;
    //         ESP_LOGD(TAG, "Override: disable color temperature support");
    //     }
    // }
}

void dali::DaliLight::apply_boot_state() {
    if (this->boot_applied_) {
        return;
    }

    // Publish the lamp's actual hardware state to Home Assistant. The LightCall triggers
    // write_state(), but writes_enabled_ is still false so the bus is NOT touched - the
    // physical lamp keeps whatever state it powered up in.
    if (this->state_ != nullptr) {
        auto call = this->state_->make_call();
        call.set_state(this->boot_state_.state);
        call.set_brightness_if_supported(this->boot_state_.brightness);
        // NOTE: color temperature is intentionally not forced here, to avoid unit ambiguity
        // in the LightCall. On/off + brightness are the states we must get right.
        call.set_save(false);
        call.set_publish(true);
        call.perform();
    }

    this->boot_applied_ = true;
    // Now allow normal control to reach the bus.
    this->writes_enabled_ = true;
    ESP_LOGD(TAG, "DALI[%d] boot state applied (on=%d, b=%.2f)",
             address_, this->boot_state_.state, this->boot_state_.brightness);
}

light::LightTraits dali::DaliLight::get_traits() {
    light::LightTraits traits;

    // NOTE: This is called repeatedly, do not perform any bus queries here...

    // Force a color mode irrespective of what the device itself says it supports
    // eg. you can convert a CT capable device to a plain brighness device,
    // or force colour temperature support and hope the device recognizes the command...
    if (this->color_mode_.has_value()) {
        switch (this->color_mode_.value()) {
            case DaliColorMode::COLOR_TEMPERATURE: 
                this->tc_supported_ = true;
                traits.set_supported_color_modes({light::ColorMode::COLOR_TEMPERATURE});
                traits.set_min_mireds(this->cold_white_temperature_);
                traits.set_max_mireds(this->warm_white_temperature_);
                break;
            case DaliColorMode::BRIGHTNESS:
                this->tc_supported_ = false;
                traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
                break;
            case DaliColorMode::ON_OFF:
                this->tc_supported_ = false;
                traits.set_supported_color_modes({light::ColorMode::ON_OFF});
                break;
        }
    }
    else {
        // Device reports color temperature support
        if (this->tc_supported_) {
            traits.set_supported_color_modes({light::ColorMode::COLOR_TEMPERATURE});
            traits.set_min_mireds(this->cold_white_temperature_);
            traits.set_max_mireds(this->warm_white_temperature_);
        }
        else {
            traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
        }
    }

    return traits;
}

void dali::DaliLight::write_state(light::LightState *state) {
    bool on;
    float brightness;
    float color_temperature;

    static uint16_t last_temperature = 0;

    // Suppress writes until the boot state has been read and published. This prevents
    // ESPHome's setup-time call.perform() (and apply_boot_state()) from resetting the lamp
    // on restart.
    if (!this->writes_enabled_) {
        return;
    }

    state->current_values_as_binary(&on);
    if (!on) {
        // Short cut: send power off command
        //bus->dali.lamp.turnOff(address_); // no fade
        bus->dali.lamp.setBrightness(address_, 0); // fade
        return;
    }

    if (tc_supported_) {
        state->current_values_as_ct(&color_temperature, &brightness);

        // Map temperature 0..1 to reported TC coolest/warmest mireds
        // NOTE: Not using the configuration warm/cool colours - these may not match the reported range of the DALI device.
        float color_temperature_mired = (color_temperature * (dali_tc_warmest_ - dali_tc_coolest_)) + dali_tc_coolest_;

        uint16_t dali_color_temperature = static_cast<uint16_t>(color_temperature_mired);

        // Only update if temperature has changed, to allow faster brightness changes
        if (dali_color_temperature != last_temperature) {
            last_temperature = dali_color_temperature;

            ESP_LOGD(TAG, "DALI[%d] Tc=%d", address_, dali_color_temperature);

            // IMPORTANT: Do not set start_fade (activate), or the color temperature fade will
            // be cancelled when we next call setBrightness, and no color change will occur.
            bus->dali.color.setColorTemperature(address_, dali_color_temperature, false);
        }
    } else {
        state->current_values_as_brightness(&brightness);
    }

    int dali_brightness = static_cast<int>(brightness * (this->dali_level_max_ - this->dali_level_min_) + this->dali_level_min_);
    if (dali_brightness < this->dali_level_min_) dali_brightness = this->dali_level_min_;
    if (dali_brightness > this->dali_level_max_) dali_brightness = this->dali_level_max_;

    // Safety net: an "on" state must never emit DALI level 0 (== OFF), otherwise the lamp
    // could be turned off but never on.
    if (dali_brightness < 1) dali_brightness = 1;

    ESP_LOGD(TAG, "DALI[%d] B=%.2f (%d)", address_, brightness, dali_brightness);
    bus->dali.lamp.setBrightness(address_, (uint8_t)dali_brightness);
}
