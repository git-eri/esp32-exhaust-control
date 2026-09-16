# ESP32 Exhaust Controller

## Hardware

Target: ESP32-WROOM-32E

| Function | GPIO |
|---|---:|
| Relay 1 | GPIO16 |
| Status LED | GPIO23 |
| PWM output | GPIO25 |

Relay logic is based on the supplied AliExpress example:

- HIGH = relay ON
- LOW = relay OFF

PWM is initially configured as:

- Frequency: 100 Hz
- Resolution: 10 bit
- OPEN: 100 %

## Modes

### AUTO
- Relay OFF
- PWM OFF
- Original vehicle controller remains in control

### OPEN
- Relay ON
- PWM = `OPEN_PWM_DUTY_PERCENT`

### MANUAL
- Relay ON
- PWM controlled by the 0–100 % slider

# ESP32 Exhaust Controller – Emergency OFF

The red EMERGENCY OFF button is shown at the top of the web UI.

After confirmation:
- Relay OFF
- PWM OFF
- Status LED OFF
- HTTP server stopped
- Wi-Fi AP disabled
- ESP32 remains powered

A reset or power cycle is required to start again.

Configure:
```cpp
#define EMERGENCY_ENABLED true
```

This is a software shutdown, not a physical power disconnect.

## First test

1. Install the ESP32 board package in Arduino IDE.
2. Open `ESP32_Exhaust_Controller.ino`.
3. Select the correct ESP32-WROOM-32E board.
4. Upload.
5. Open Serial Monitor at 115200 baud.
6. Connect your phone to Wi-Fi:
   - SSID: `Auspuff-ESP32`
   - Password: `Auspuff123`
7. Open `http://192.168.4.1`.

## Important electrical note

GPIO25 is a 3.3 V logic output. Do NOT connect GPIO25 directly to the vehicle's 12 V PWM line.

Use the transistor/driver circuit between GPIO25 and the 12 V circuit. The exact circuit should be checked against the transistor you purchased and the input requirements of the vehicle controller.

## Later oscilloscope calibration

After measuring the original signal, adjust:

```cpp
#define PWM_FREQUENCY_HZ        100
#define OPEN_PWM_DUTY_PERCENT   100
```

If the vehicle controller expects, for example, 10 % for closed and 87 % for open, the software can later be changed to map the 0–100 % slider to that measured range.

## Optional failsafe

Currently disabled:

```cpp
#define FAILSAFE_ENABLED false
```

If enabled, the ESP32 switches back to AUTO when no browser heartbeat has been received for 30 seconds:

```cpp
#define FAILSAFE_ENABLED        true
#define FAILSAFE_TIMEOUT_MS     30000UL
```

This is intentionally disabled in the first version so that behavior is not changed beyond what was requested.

## Notes

The web page is embedded in the ESP32 firmware. No internet connection or external web server is required.
