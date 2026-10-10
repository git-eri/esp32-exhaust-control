# ESP32 Exhaust Controller

ESP32 firmware for a relay board that switches a BMW exhaust valve between the original vehicle controller and a local PWM signal. Control is available through an embedded web page over the ESP32's Wi-Fi access point. No internet connection or external web server is required.

## Hardware

Target: ESP32-WROOM-32E.

| Function | GPIO | Logic |
|---|---:|---|
| Relay 1 | 16 | HIGH = ON, LOW = OFF |
| Status LED | 23 | HIGH = ON, LOW = OFF |
| PWM output | 25 | 3.3 V output to the inverting transistor driver |
| Physical emergency reset | 0 | Onboard button to GND, active LOW |

The measured working PWM settings are:

- Frequency: 100 Hz.
- Resolution: 10 bit.
- OPEN: 85 % duty on the vehicle side.
- The transistor stage inverts the signal, so GPIO25 produces approximately 15 % HIGH duty for OPEN.

`OPEN_PWM_DUTY_PERCENT` describes the vehicle-side duty. `pwmOff()` sets the ESP32 duty to zero, leaving GPIO25 LOW. After the inverting driver this corresponds to a constant HIGH level; AUTO relies on the relay returning control to the original vehicle controller.

GPIO25 must connect through the intended transistor/driver stage, not directly to the vehicle's 12 V PWM line.

## Modes and startup

| Mode | Relay | ESP32 PWM | Status LED |
|---|---|---|---|
| AUTO | OFF | GPIO25 held LOW | OFF |
| OPEN | ON | 100 Hz, approximately 15 % GPIO duty / 85 % vehicle duty | ON |

PWM setup is checked; Arduino-ESP32 3.x write results are checked as well. If PWM cannot be configured or a reported write failure occurs, OPEN is rejected or the outputs fall back to AUTO. The HTTP status exposes `pwmReady` so the web page can report the fault.

With **Remember last setting** disabled, the controller starts in AUTO. With it enabled, AUTO/OPEN changes are stored in ESP32 NVS and restored on the next normal boot. Outputs are applied before storing the new mode. Storage errors are reported, so a mode may already be active even when saving it for the next boot fails.

Missing NVS namespaces or keys use normal first-boot defaults. Other errors reading the emergency state keep Wi-Fi OFF with AUTO outputs; errors reading mode preferences fall back to AUTO.

## Emergency OFF

The web page's emergency button requests this sequence:

1. Relay OFF, PWM LOW, status LED OFF.
2. Store the emergency latch in NVS.
3. Send a response reporting the emergency and persistence result.
4. Stop HTTP, mDNS and the Wi-Fi access point.

The web page shows a confirmed shutdown only after receiving a valid emergency response. If the connection drops before that response arrives, it reports that confirmation is missing. If storing the latch fails, the outputs still switch to AUTO and communication stops, but a power cycle may lose the emergency latch.

After a successfully stored emergency, reset or power cycling keeps the outputs in AUTO and Wi-Fi OFF. To clear it, keep the ESP32 powered, release the onboard GPIO0 button, then press and hold it continuously for 5 seconds. Wi-Fi restarts in AUTO only if clearing the stored latch succeeds.

The remembered startup mode is preserved when clearing an emergency. For example, if OPEN was remembered beforehand, the physical reset returns to AUTO immediately, but a later normal power cycle restores OPEN.

GPIO0 also selects the ESP32 boot mode. Keep the button released when powering on or resetting the board; holding it LOW during reset enters the download bootloader instead of running this firmware.

Emergency OFF is a software shutdown that returns control to the original vehicle controller. The ESP32 remains powered.

## Upload and connect

1. Install the Arduino-ESP32 board package in Arduino IDE.
2. Open `esp32-exhaust-control.ino` and select the appropriate ESP32-WROOM-32E board and port.
3. Upload the sketch.
4. Connect your phone to the access point:
   - SSID: `WiFi`
   - Password: `Exhaust123`
5. Open `http://192.168.4.1` or `http://exhaust.local`.

The access point does not advertise a default gateway over DHCP, allowing local controller access while the phone can continue using mobile data for internet traffic. The direct IP address is available if the phone does not resolve the mDNS hostname.

Serial debug output is disabled by default. For firmware diagnostics, set `SERIAL_OUTPUT_ENABLED` to `true` and open Serial Monitor at 115200 baud.

## Configuration

The main settings are defined near the start of the sketch:

```cpp
#define SERIAL_OUTPUT_ENABLED   false
#define PWM_FREQUENCY_HZ        100
#define PWM_RESOLUTION_BITS     10
#define OPEN_PWM_DUTY_PERCENT   85
#define EMERGENCY_ENABLED       true
#define EMERGENCY_RESET_HOLD_MS 5000UL
#define FAILSAFE_ENABLED        false
#define FAILSAFE_TIMEOUT_MS     30000UL
```

The optional communication failsafe remains disabled. When enabled, OPEN returns to AUTO if no browser heartbeat arrives for 30 seconds. The web page sends a heartbeat every 5 seconds. Phone backgrounding or suspension can stop these heartbeats; account for that behavior before enabling the failsafe.

HTML, CSS and JavaScript remain embedded in the sketch. The web page communicates with the local `/api/status`, `/api/mode`, `/api/preferences`, `/api/emergency` and `/api/heartbeat` endpoints. Status also exposes `persistenceOk` for storage diagnostics.

## Manual verification

Use a bench setup to check these behaviors before connecting to the vehicle:

1. Switch between AUTO and OPEN. Verify relay and LED states, and measure 100 Hz / approximately 15 % HIGH duty at GPIO25 and 85 % after the inverting driver in OPEN.
2. With remembering disabled, reboot after OPEN and verify AUTO. Enable remembering, select OPEN, reboot and verify that OPEN is restored.
3. Trigger Emergency OFF. Verify the outputs switch to AUTO, then Wi-Fi disappears. Power cycle and verify the stored emergency keeps Wi-Fi OFF. Release the button, hold it for 5 seconds, and verify Wi-Fi returns in AUTO.
4. Disconnect Wi-Fi before sending an emergency request. Verify the browser reports missing confirmation and does not claim a confirmed shutdown.
5. In a fault-injection test build, force PWM setup/write and NVS write failures. Verify OPEN cannot keep the relay ON after a PWM fault; persistence errors are reported; and a failed emergency-latch clear does not restart Wi-Fi.
