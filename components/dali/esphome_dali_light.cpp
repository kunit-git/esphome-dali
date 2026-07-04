
#include <esphome.h>
#include <cmath>
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
        // The bit-banged bus can NACK a single query (especially the first transaction
        // after boot), so retry presence detection before giving up. A false negative here
        // is dangerous: it takes the "not found" path below, which enables writes and lets
        // ESPHome's restore (default OFF) turn the lamp off - the exact thing we must avoid.
        bool present = false;
        for (int attempt = 0; attempt < 5; attempt++) {
            if (bus->dali.isDevicePresent(address_)) {
                present = true;
                break;
            }
            delay(10);
        }
        if (present) {
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
            // DAPC level 0xFF (255) is the "MASK"/stop-fading value and is NOT a real
            // arc-power level - valid levels are 1..254. A gear reporting 255 as its max
            // (or a misread on the bit-banged bus) would otherwise make the "on" command
            // at 100% map to DALI level 255, which the lamp ignores - so it could be turned
            // off but never back on at full brightness.
            if (max_level == 0xFF) {
                ESP_LOGW(TAG, "DALI[%.2x] Reported max 255 (MASK), capping to 254", address_);
                max_level = 254;
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
            uint8_t color_features = bus->dali.color.getColorFeatures(address_);
            this->tc_supported_ = (color_features & (uint8_t)DaliColorFeature::TC_CAPABLE) != 0;

            // DT8 RGBWAF: bits 5-7 report the number of colour channels (3=RGB, 4+=RGBW)
            uint8_t rgbwaf_channels = (color_features >> 5) & 0x07;
            this->rgb_supported_ = (rgbwaf_channels >= 3);
            this->rgbw_supported_ = (rgbwaf_channels >= 4);
            if (rgb_supported_) {
                ESP_LOGD(TAG, "DALI[%.2x] Supports RGB%s (%d RGBWAF channels)",
                         address_, rgbw_supported_ ? "W" : "", rgbwaf_channels);
            }

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
            //
            // On/off is decided by QUERY LAMP POWER ON, not by the actual-level query:
            // some gear/bus timing yields a consistent non-zero level byte even while the
            // lamp is off, which made Home Assistant report off lamps as on. QUERY LAMP
            // POWER ON answers "yes" as exactly 0xFF and an off lamp sends no reply at
            // all (reads 0), so neither a NACK nor line noise can fabricate an "on".
            bool is_on = false;
            for (int attempt = 0; attempt < 3 && !is_on; attempt++) {
                is_on = bus->dali.lamp.isLampPoweredOn(address_);
            }
            this->hw_state_.state = is_on;

            if (is_on) {
                // Read the brightness. A single read is not trustworthy on the bit-banged
                // bus, so keep reading until two consecutive reads agree.
                uint8_t current_level = 0;
                {
                    uint8_t prev = 0;
                    bool have_prev = false;
                    for (int attempt = 0; attempt < 6; attempt++) {
                        uint8_t lvl = bus->dali.lamp.getCurrentLevel(address_);
                        if (have_prev && lvl == prev) {
                            current_level = lvl;
                            break;
                        }
                        prev = lvl;
                        have_prev = true;
                        current_level = lvl; // best effort if no two reads ever agree
                        delay(5);
                    }
                }

                float brightness = this->level_to_brightness_(current_level);
                this->hw_state_.brightness = brightness;
                ESP_LOGD(TAG, "DALI[%.2x] Restore: on, brightness %.2f (raw level %d)",
                         address_, brightness, current_level);
            } else {
                ESP_LOGD(TAG, "DALI[%.2x] Restore: off", address_);
            }

            // NOTE: RGB(W) channel levels are intentionally not restored on boot: reading
            // them back needs 4-6 QUERY_COLOR_VALUE round-trips on the flaky bit-banged
            // bus, and the byte layout of 8-bit dim levels in the 16-bit colour value
            // register varies between gear. On/off + brightness are the states that matter.
            if (tc_supported_) {
                uint16_t current_temperature = bus->dali.color.getColorTemperature(address_);
                if (current_temperature != 0) {
                    // Convert mireds to 0..1 range
                    this->hw_state_.color_temp =
                        (current_temperature - dali_tc_coolest_) / (dali_tc_warmest_ - dali_tc_coolest_);
                    ESP_LOGD(TAG, "Restore colour temperature: %.2f", this->hw_state_.color_temp);
                }
            }

            // The bus component drives apply_boot_state() after all setup completes, which
            // publishes this state to Home Assistant without writing to the bus.
            bus->register_light(this);
        }
        else {
            ESP_LOGW(TAG, "DALI device at addr %.2x not found!", address_);
            // This may be a false negative on the flaky bus, or a genuinely absent device.
            // Either way do NOT enable writes here: that lets ESPHome's boot restore (the
            // last-saved/default HA state) drive the bus and switch a real lamp on or off,
            // which is exactly the boot-time toggling we must avoid. Keep writes suppressed
            // and route through the normal boot flow - apply_boot_state() publishes a safe
            // "off" to Home Assistant and then enables writes for normal control. The bus is
            // never written as a side effect of booting.
            this->hw_state_.state = false;
            bus->register_light(this);
        }

        //bus->dali.dumpStatusForDevice(address_);
    }
    else {
        // Broadcast / group addresses: we cannot read a single state back. Still route
        // through the suppressed boot flow so the restore never drives the bus on boot;
        // apply_boot_state() publishes "off" and then enables writes for normal control.
        // TODO: How do we detect color temperature support for broadcast and group addresses?
        this->hw_state_.state = false;
        bus->register_light(this);
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

float dali::DaliLight::level_to_brightness_(uint8_t level) {
    // Inverse of write_state()'s min..max mapping, clamped to 0..1.
    // 0 and 0xFF (MASK) are not valid levels for a lamp we know is on -
    // fall back to full brightness rather than guessing.
    float brightness = 1.0f;
    if (level != 0 && level != 0xFF && this->dali_level_max_ > this->dali_level_min_) {
        brightness = (float)(level - this->dali_level_min_) /
                     (float)(this->dali_level_max_ - this->dali_level_min_);
        if (brightness < 0.0f) brightness = 0.0f;
        if (brightness > 1.0f) brightness = 1.0f;
        // write_state() receives a gamma-corrected brightness before mapping it
        // to a DALI level, so this must apply the inverse. Without it, the value
        // would get gamma-crushed a second time on the next turn-on, mapping to
        // a near-minimum DALI level that looks off.
        float gamma = (this->state_ != nullptr) ? this->state_->get_gamma_correct() : 1.0f;
        if (gamma > 0.0f) {
            brightness = powf(brightness, 1.0f / gamma);
        }
    }
    // A lamp sitting at its minimum level maps to brightness 0, which a
    // LightCall would coerce back to "off" - keep it publishable as "on".
    if (brightness < 0.01f) brightness = 0.01f;
    return brightness;
}

void dali::DaliLight::publish_hw_state_(bool is_on, uint8_t level) {
    if (this->state_ == nullptr || !this->writes_enabled_) {
        return; // Boot flow (apply_boot_state) handles the initial publish
    }

    // Back off shortly after our own commands: a DAPC starts a gear-side fade that can
    // take seconds, and syncing an intermediate level back to HA would look like an
    // external change (and publish a value the user never picked).
    if ((uint32_t)(millis() - this->last_write_ms_) < 10000) {
        return;
    }

    float brightness = is_on ? this->level_to_brightness_(level) : this->hw_state_.brightness;

    const auto& remote = this->state_->remote_values;
    bool changed = (is_on != remote.is_on()) ||
                   (is_on && fabsf(brightness - remote.get_brightness()) > 0.03f);
    if (!changed) {
        return;
    }

    ESP_LOGI(TAG, "DALI[%.2x] External change detected, updating HA: %s (brightness %.2f)",
             address_, is_on ? "ON" : "OFF", brightness);

    this->hw_state_.state = is_on;
    if (is_on) {
        this->hw_state_.brightness = brightness;
    }

    auto call = this->state_->make_call();
    call.set_state(is_on);
    if (is_on) {
        call.set_brightness_if_supported(brightness);
    }
    call.set_save(false);
    call.set_publish(true);
    // No transition: this reflects a state the lamp already has.
    call.set_transition_length(0);
    call.perform();
    // Swallow the write_state() echo queued by the perform() - see write_state().
    this->hw_write_guard_ = true;
}

void dali::DaliLight::refresh_lamp_state() {
    if ((this->address_ == ADDR_BROADCAST) || ((this->address_ & ADDR_GROUP_MASK) != 0)) {
        return; // No single lamp state to read for group/broadcast addresses
    }

    // Live status poll, driven once a minute per lamp from the bus loop(). Queries are
    // deliberately single-shot (no retry loops): each extra query blocks the main loop
    // for another bus frame round-trip.
    // Raw bytes are included for bus diagnostics: power_on answers "yes" as exactly
    // 0xFF ("no" = no reply, reads 0); status bit2 mirrors "lamp arc power on".
    uint8_t power_on = bus->sendQueryCommand(address_, DaliCommand::QUERY_LAMP_POWER_ON);
    uint8_t status = bus->sendQueryCommand(address_, DaliCommand::QUERY_STATUS);

    if (power_on != 0xFF) {
        ESP_LOGI(TAG, "DALI[%.2x] Lamp state: OFF (power_on=0x%02x, status=0x%02x)",
                 address_, power_on, status);
        this->publish_hw_state_(false, 0);
        return;
    }

    uint8_t level = bus->dali.lamp.getCurrentLevel(address_);
    this->publish_hw_state_(true, level);

    if (rgb_supported_) {
        uint8_t r = bus->dali.color.getReportedDimLevel(address_, DaliColorParam::ReportRedDimLevel);
        uint8_t g = bus->dali.color.getReportedDimLevel(address_, DaliColorParam::ReportGreenDimLevel);
        uint8_t b = bus->dali.color.getReportedDimLevel(address_, DaliColorParam::ReportBlueDimLevel);
        if (rgbw_supported_) {
            uint8_t w = bus->dali.color.getReportedDimLevel(address_, DaliColorParam::ReportWhiteDimLevel);
            ESP_LOGI(TAG, "DALI[%.2x] Lamp state: ON, level %d/%d, RGBW=(%d,%d,%d,%d) (status=0x%02x)",
                     address_, level, dali_level_max_, r, g, b, w, status);
        } else {
            ESP_LOGI(TAG, "DALI[%.2x] Lamp state: ON, level %d/%d, RGB=(%d,%d,%d) (status=0x%02x)",
                     address_, level, dali_level_max_, r, g, b, status);
        }
    } else {
        ESP_LOGI(TAG, "DALI[%.2x] Lamp state: ON, level %d/%d (status=0x%02x)",
                 address_, level, dali_level_max_, status);
    }
}

void dali::DaliLight::apply_boot_state() {
    if (this->boot_applied_) {
        return;
    }

    // Publish the lamp's actual hardware state to Home Assistant. NOTE: the LightCall
    // queues one write_state() for the *next* loop iteration - after writes_enabled_ has
    // been flipped below - so the hw_write_guard_ set below is what actually keeps this
    // publish off the bus. The physical lamp keeps whatever state it powered up in.
    if (this->state_ != nullptr) {
        auto call = this->state_->make_call();
        call.set_state(this->hw_state_.state);
        call.set_brightness_if_supported(this->hw_state_.brightness);
        // NOTE: color temperature is intentionally not forced here, to avoid unit ambiguity
        // in the LightCall. On/off + brightness are the states we must get right.
        call.set_save(false);
        call.set_publish(true);
        // No fade: this is a state-reflection publish, not a real change. A transition would
        // keep driving write_state() from loop() after we flip writes_enabled_ below, which
        // could push the bus toward the (possibly off) boot value.
        call.set_transition_length(0);
        call.perform();
    }

    this->boot_applied_ = true;
    // The perform() above queued exactly one write_state() (immediate, transition 0).
    // Arm the guard so that echo is swallowed instead of written back onto the bus -
    // see write_state().
    this->hw_write_guard_ = true;
    // Now allow normal control to reach the bus.
    this->writes_enabled_ = true;
    ESP_LOGD(TAG, "DALI[%d] boot state applied (on=%d, b=%.2f)",
             address_, this->hw_state_.state, this->hw_state_.brightness);
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
                this->rgb_supported_ = false;
                this->rgbw_supported_ = false;
                traits.set_supported_color_modes({light::ColorMode::COLOR_TEMPERATURE});
                traits.set_min_mireds(this->cold_white_temperature_);
                traits.set_max_mireds(this->warm_white_temperature_);
                break;
            case DaliColorMode::RGB:
                this->tc_supported_ = false;
                this->rgb_supported_ = true;
                this->rgbw_supported_ = false;
                traits.set_supported_color_modes({light::ColorMode::RGB});
                break;
            case DaliColorMode::RGBW:
                this->tc_supported_ = false;
                this->rgb_supported_ = true;
                this->rgbw_supported_ = true;
                traits.set_supported_color_modes({light::ColorMode::RGB_WHITE});
                break;
            case DaliColorMode::BRIGHTNESS:
                this->tc_supported_ = false;
                this->rgb_supported_ = false;
                this->rgbw_supported_ = false;
                traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
                break;
            case DaliColorMode::ON_OFF:
                this->tc_supported_ = false;
                this->rgb_supported_ = false;
                this->rgbw_supported_ = false;
                traits.set_supported_color_modes({light::ColorMode::ON_OFF});
                break;
        }
    }
    else {
        // Use what the device reported during setup. A DT8 gear can support several
        // colour types (e.g. RGBWAF and Tc); expose each as a Home Assistant mode.
        // NOTE: Built via the initializer-list overload, which exists in both the old
        // (std::set) and new (ColorModeMask) LightTraits APIs.
        if (this->rgbw_supported_) {
            if (this->tc_supported_) {
                traits.set_supported_color_modes({light::ColorMode::RGB_WHITE, light::ColorMode::COLOR_TEMPERATURE});
            } else {
                traits.set_supported_color_modes({light::ColorMode::RGB_WHITE});
            }
        } else if (this->rgb_supported_) {
            if (this->tc_supported_) {
                traits.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::COLOR_TEMPERATURE});
            } else {
                traits.set_supported_color_modes({light::ColorMode::RGB});
            }
        } else if (this->tc_supported_) {
            traits.set_supported_color_modes({light::ColorMode::COLOR_TEMPERATURE});
        } else {
            traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
        }

        if (this->tc_supported_) {
            traits.set_min_mireds(this->cold_white_temperature_);
            traits.set_max_mireds(this->warm_white_temperature_);
        }
    }

    return traits;
}

// Map a 0..1 colour channel value to a DALI RGBWAF dim level (0..254; 255 is MASK)
static inline uint8_t to_dali_channel(float value) {
    int level = static_cast<int>(value * 254.0f + 0.5f);
    if (level < 0) level = 0;
    if (level > 254) level = 254;
    return (uint8_t)level;
}

void dali::DaliLight::write_state(light::LightState *state) {
    bool on;
    float brightness;
    float color_temperature;

    // Suppress writes until the boot state has been read and published. This prevents
    // ESPHome's setup-time call.perform() (and apply_boot_state()) from resetting the lamp
    // on restart.
    if (!this->writes_enabled_) {
        return;
    }

    state->current_values_as_binary(&on);

    // One-shot hardware-state guard: publishes in apply_boot_state() and
    // publish_hw_state_() reflect a state the lamp already has, but they also queue this
    // write_state() call. Writing it back would be redundant at best - and if the read
    // was corrupted by the bit-banged bus, it would *change* the lamp (e.g. turn on a
    // lamp that is actually off). Swallow this one write while it still matches the
    // published hardware state; anything that differs is a real command and goes through.
    if (this->hw_write_guard_) {
        this->hw_write_guard_ = false;
        bool same_state = (on == this->hw_state_.state);
        bool same_brightness = !this->hw_state_.state ||
            fabsf(state->current_values.get_brightness() - this->hw_state_.brightness) < 0.01f;
        if (same_state && same_brightness) {
            return;
        }
    }

    if (!on) {
        // Short cut: send power off command
        //bus->dali.lamp.turnOff(address_); // no fade
        this->last_write_ms_ = millis();
        bus->dali.lamp.setBrightness(address_, 0); // fade
        return;
    }

    auto color_mode = state->current_values.get_color_mode();

    if (rgb_supported_ && (color_mode & light::ColorCapability::RGB)) {
        const auto& values = state->current_values;

        // The RGBWAF dim levels define the colour point only; overall intensity is
        // driven by the DAPC (arc power) frame below. color_brightness carries the
        // magnitude of the HA-requested RGB colour (ESPHome normalizes rgb so that
        // max(r,g,b) == 1), so it must be folded back into the channel levels.
        float color_brightness = values.get_color_brightness();
        uint8_t r = to_dali_channel(values.get_red() * color_brightness);
        uint8_t g = to_dali_channel(values.get_green() * color_brightness);
        uint8_t b = to_dali_channel(values.get_blue() * color_brightness);

        bool is_rgbw = rgbw_supported_ && (color_mode & light::ColorCapability::WHITE);
        uint8_t w = is_rgbw ? to_dali_channel(values.get_white()) : 0;

        // Only update if the colour has changed, to allow faster brightness changes
        if (!last_color_valid_ ||
            r != last_rgbw_[0] || g != last_rgbw_[1] || b != last_rgbw_[2] || w != last_rgbw_[3]) {
            last_color_valid_ = true;
            last_rgbw_[0] = r;
            last_rgbw_[1] = g;
            last_rgbw_[2] = b;
            last_rgbw_[3] = w;

            ESP_LOGD(TAG, "DALI[%d] RGBW=(%d,%d,%d,%d)", address_, r, g, b, w);

            // IMPORTANT: Do not set start_fade (activate), or the color fade will be
            // cancelled when we next call setBrightness, and no color change will occur.
            // The DAPC frame below activates the temporary colour values.
            bus->dali.color.setRGB(address_, r, g, b, false);
            if (is_rgbw) {
                // Amber/Freecolour channels (if any) are left unchanged (MASK)
                bus->dali.color.setWAF(address_, w, 0xFF, 0xFF, false);
            }
        }
    }
    else if (tc_supported_ && (color_mode & light::ColorCapability::COLOR_TEMPERATURE)) {
        state->current_values_as_ct(&color_temperature, &brightness);

        // Map temperature 0..1 to reported TC coolest/warmest mireds
        // NOTE: Not using the configuration warm/cool colours - these may not match the reported range of the DALI device.
        float color_temperature_mired = (color_temperature * (dali_tc_warmest_ - dali_tc_coolest_)) + dali_tc_coolest_;

        uint16_t dali_color_temperature = static_cast<uint16_t>(color_temperature_mired);

        // Only update if temperature has changed, to allow faster brightness changes
        if (dali_color_temperature != last_temperature_) {
            last_temperature_ = dali_color_temperature;

            ESP_LOGD(TAG, "DALI[%d] Tc=%d", address_, dali_color_temperature);

            // IMPORTANT: Do not set start_fade (activate), or the color temperature fade will
            // be cancelled when we next call setBrightness, and no color change will occur.
            bus->dali.color.setColorTemperature(address_, dali_color_temperature, false);
        }
    }

    state->current_values_as_brightness(&brightness);

    int dali_brightness = static_cast<int>(brightness * (this->dali_level_max_ - this->dali_level_min_) + this->dali_level_min_);
    if (dali_brightness < this->dali_level_min_) dali_brightness = this->dali_level_min_;
    if (dali_brightness > this->dali_level_max_) dali_brightness = this->dali_level_max_;

    // Safety net: an "on" state must never emit DALI level 0 (== OFF) nor 255 (== MASK /
    // stop-fading, a no-op), otherwise the lamp could be turned off but never on.
    if (dali_brightness < 1) dali_brightness = 1;
    if (dali_brightness > 254) dali_brightness = 254;

    ESP_LOGD(TAG, "DALI[%d] B=%.2f (%d)", address_, brightness, dali_brightness);
    this->last_write_ms_ = millis();
    bus->dali.lamp.setBrightness(address_, (uint8_t)dali_brightness);
}
