#pragma once

// Copy this file to config.h beside the sketch and edit that local copy.
// config.example.h is versioned; config.h is ignored by Git.

// ============================================================
// SERIAL DEBUG OUTPUT
// ============================================================

// Set to true for serial debug output, false for completely silent UART.
#define SERIAL_OUTPUT_ENABLED   true

// ============================================================
// HARDWARE CONFIGURATION
// ============================================================

#define RELAY_PIN               16
#define STATUS_LED_PIN          23
#define PWM_PIN                 25

#define PWM_FREQUENCY_HZ        100
#define PWM_RESOLUTION_BITS     10
// LEDC channel used only with Arduino-ESP32 2.x.
#define PWM_CHANNEL             0

// Vehicle-side duty; the transistor driver inverts the ESP32 signal.
// Confirm the correct vehicle-side behavior with your measurements.
#define OPEN_PWM_DUTY_PERCENT   85

#define RELAY_ON                HIGH
#define RELAY_OFF               LOW

#define LED_ON                  HIGH
#define LED_OFF                 LOW

// ============================================================
// WIFI CONFIGURATION
// ============================================================

#define AP_HIDE_SSID            false
#define AP_SSID                 "Exhaust"
#define AP_PASSWORD             "Exhaust123"
#define AP_CHANNEL              6

// Local hostname without the .local suffix: http://exhaust.local
#define MDNS_HOSTNAME           "exhaust"

// ============================================================
// OPTIONAL FAILSAFE
// ============================================================

// If enabled, AUTO is selected when the web UI has not refreshed
// the heartbeat within FAILSAFE_TIMEOUT_MS.
// The web UI sends a heartbeat every 5 seconds.
// This is NOT a substitute for proper electrical/mechanical safety.
#define FAILSAFE_ENABLED        false
#define FAILSAFE_TIMEOUT_MS     30000UL

// ============================================================
// EMERGENCY SHUTDOWN
// ============================================================

#define EMERGENCY_ENABLED       true

// Physical emergency reset button between the configured GPIO and GND.
// Active LOW with the internal pull-up; GPIO0 is the onboard boot button.
#define EMERGENCY_RESET_PIN     0
#define EMERGENCY_RESET_HOLD_MS  5000UL

// ============================================================
// PERSISTENT STORAGE
// ============================================================

// ESP32 NVS (flash). Keep these names stable to retain stored settings.
#define NVS_NAMESPACE           "exhaust"
#define NVS_EMERGENCY_KEY       "emergency"
#define NVS_REMEMBER_MODE_KEY   "rememberMode"
#define NVS_START_MODE_KEY      "startMode"
