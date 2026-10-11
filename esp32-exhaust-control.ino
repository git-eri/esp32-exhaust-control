/*
  ESP32 Exhaust Controller
  Target: ESP32-WROOM-32E
  Arduino IDE / Arduino-ESP32

  Hardware:
    GPIO16 = Relay 1 (HIGH = ON, LOW = OFF)
    GPIO23 = Status LED (HIGH = ON)
    GPIO25 = PWM output, 100 Hz

  Modes:
    AUTO   = relay OFF, PWM OFF
    OPEN   = relay ON, PWM at OPEN_PWM_DUTY_PERCENT

  Communication:
    1) Wi-Fi AP + Web UI

  IMPORTANT:
    GPIO25 is a 3.3 V logic output. Do NOT connect it directly to a 12 V circuit.
    Use the intended transistor/driver stage between GPIO25 and the vehicle's
    12 V PWM input.

  Notes:
    - Emergency shutdown disables Wi-Fi.
    - Emergency state is stored persistently in ESP32 NVS.
    - On reboot, stored emergency keeps Wi-Fi OFF.
    - GPIO0 button (active LOW) clears emergency after release-then-press.
*/

#include <WiFi.h>
#include <esp_netif.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Preferences.h>
#include <nvs.h>

// Local settings: copy config.example.h to config.h before compiling.
#include "config.h"

#include "web_ui.h"

#if SERIAL_OUTPUT_ENABLED
  #define SERIAL_LOG_BEGIN(baud) Serial.begin(baud)
  #define SERIAL_PRINT(x) Serial.print(x)
  #define SERIAL_PRINTLN(x) Serial.println(x)
#else
  #define SERIAL_LOG_BEGIN(baud) do {} while (0)
  #define SERIAL_PRINT(x) do {} while (0)
  #define SERIAL_PRINTLN(x) do {} while (0)
#endif

// ============================================================
// GLOBALS
// ============================================================

WebServer server(80);
Preferences preferences;

bool communicationServicesStarted = false;
bool pwmReady = false;
bool persistenceOk = true;

bool emergencyResetButtonArmed = false;
bool lastEmergencyResetButtonState = HIGH;
unsigned long lastEmergencyResetButtonChangedAt = 0;

enum ControlMode {
  MODE_AUTO,
  MODE_OPEN
};

ControlMode currentMode = MODE_AUTO;
bool rememberLastMode = false;
ControlMode rememberedMode = MODE_AUTO;

unsigned long lastHeartbeat = 0;

bool emergencyShutdown = false;

// Outputs are disabled before the HTTP reply; Wi-Fi stops afterwards in loop().
bool communicationShutdownRequested = false;

// LEDC channel is only needed with Arduino-ESP32 2.x.
#if defined(ESP_ARDUINO_VERSION_MAJOR)
  #if ESP_ARDUINO_VERSION_MAJOR < 3
    #define USE_LEDC_OLD_API
  #endif
#else
  #define USE_LEDC_OLD_API
#endif

// ============================================================
// PERSISTENT EMERGENCY STATE
// ============================================================

bool saveEmergencyState(bool active) {
  if (!preferences.begin(NVS_NAMESPACE, false)) {
    SERIAL_PRINTLN("NVS: ERROR opening namespace for write");
    persistenceOk = false;
    return false;
  }

  const bool saved = (preferences.putBool(NVS_EMERGENCY_KEY, active) == 1);
  preferences.end();
  persistenceOk = saved;

  if (!saved) {
    SERIAL_PRINTLN("NVS: ERROR saving emergency state");
    return false;
  }

  SERIAL_PRINT("NVS: emergency = ");
  SERIAL_PRINTLN(active ? "true" : "false");
  return true;
}

// Unlike Preferences getters, this preserves the reason for a failed read.
// Missing keys/namespaces are normal on the first boot; other errors are not.
esp_err_t readStoredByte(const char* key, uint8_t& value) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) return err;

  err = nvs_get_u8(handle, key, &value);
  nvs_close(handle);
  return err;
}

bool loadEmergencyState() {
  uint8_t storedState = 0;
  const esp_err_t err = readStoredByte(NVS_EMERGENCY_KEY, storedState);

  if (err == ESP_ERR_NVS_NOT_FOUND) return false;
  if (err != ESP_OK || storedState > 1) {
    SERIAL_PRINTLN("NVS: ERROR reading emergency state -> stay offline");
    persistenceOk = false;
    return true;
  }

  const bool active = (storedState == 1);

  SERIAL_PRINT("NVS: emergency = ");
  SERIAL_PRINTLN(active ? "true" : "false");

  return active;
}

// ============================================================
// PERSISTENT START MODE
// ============================================================

bool saveModePreference(bool remember, ControlMode mode) {
  if (mode != MODE_AUTO && mode != MODE_OPEN) {
    persistenceOk = false;
    return false;
  }

  if (!preferences.begin(NVS_NAMESPACE, false)) {
    SERIAL_PRINTLN("NVS: ERROR opening namespace for mode preference");
    persistenceOk = false;
    return false;
  }

  // Store the selected mode before enabling it, so a failed mode write cannot
  // accidentally enable a stale startup mode.
  bool saved = true;
  if (remember) {
    saved = (preferences.putUChar(NVS_START_MODE_KEY,
                                 static_cast<uint8_t>(mode)) == 1);
  }
  if (saved) {
    saved = (preferences.putBool(NVS_REMEMBER_MODE_KEY, remember) == 1);
  }

  preferences.end();
  persistenceOk = saved;

  if (!saved) {
    SERIAL_PRINTLN("NVS: ERROR saving mode preference");
    return false;
  }

  SERIAL_PRINT("NVS: remember mode = ");
  SERIAL_PRINTLN(remember ? "true" : "false");
  return true;
}

void loadModePreference() {
  rememberLastMode = false;
  rememberedMode = MODE_AUTO;
  uint8_t storedRemember = 0;
  uint8_t storedMode = static_cast<uint8_t>(MODE_AUTO);
  const esp_err_t rememberErr = readStoredByte(NVS_REMEMBER_MODE_KEY, storedRemember);
  const esp_err_t modeErr = readStoredByte(NVS_START_MODE_KEY, storedMode);

  if ((rememberErr != ESP_OK && rememberErr != ESP_ERR_NVS_NOT_FOUND) ||
      (modeErr != ESP_OK && modeErr != ESP_ERR_NVS_NOT_FOUND) ||
      storedRemember > 1 || storedMode > static_cast<uint8_t>(MODE_OPEN)) {
    SERIAL_PRINTLN("NVS: ERROR reading mode preference -> AUTO");
    persistenceOk = false;
    return;
  }

  rememberLastMode = (storedRemember == 1);
  rememberedMode = static_cast<ControlMode>(storedMode);
  SERIAL_PRINT("NVS: remember mode = ");
  SERIAL_PRINTLN(rememberLastMode ? "true" : "false");

  SERIAL_PRINT("NVS: startup mode = ");
  SERIAL_PRINTLN(rememberedMode == MODE_OPEN ? "OPEN" : "AUTO");
}

// ============================================================
// SOFTAP DHCP CONFIGURATION
// ============================================================

// Do not advertise the ESP32 as the client's default gateway.
// This keeps the Wi-Fi connection available for local access to the
// controller while allowing the iPhone to keep using mobile data for
// Internet traffic. DHCP Option 3 (Router) is disabled explicitly.
void configureSoftApDhcpNoDefaultGateway() {
  esp_netif_t* apNetif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

  if (apNetif == nullptr) {
    SERIAL_PRINTLN("WiFi DHCP: WIFI_AP_DEF not found");
    return;
  }

  esp_err_t err = esp_netif_dhcps_stop(apNetif);

  if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
    SERIAL_PRINT("WiFi DHCP: stop failed: ");
    SERIAL_PRINTLN(esp_err_to_name(err));
    return;
  }

  uint8_t routerOptionEnabled = 0;

  err = esp_netif_dhcps_option(
    apNetif,
    ESP_NETIF_OP_SET,
    ESP_NETIF_ROUTER_SOLICITATION_ADDRESS,
    &routerOptionEnabled,
    sizeof(routerOptionEnabled)
  );

  if (err != ESP_OK) {
    SERIAL_PRINT("WiFi DHCP: disabling Router option failed: ");
    SERIAL_PRINTLN(esp_err_to_name(err));
  } else {
    SERIAL_PRINTLN("WiFi DHCP: Default Gateway / DHCP Option 3 disabled");
  }

  err = esp_netif_dhcps_start(apNetif);

  if (err != ESP_OK) {
    SERIAL_PRINT("WiFi DHCP: restart failed: ");
    SERIAL_PRINTLN(esp_err_to_name(err));
  }
}

// ============================================================
// COMMUNICATION SERVICES
// ============================================================

void registerHttpRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/preferences", HTTP_POST, handlePreferences);

  if (EMERGENCY_ENABLED) {
    server.on("/api/emergency", HTTP_POST, handleEmergency);
  }

  server.on("/api/heartbeat", HTTP_POST, handleHeartbeat);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
}

void startCommunicationServices() {
  if (communicationServicesStarted) return;
  if (emergencyShutdown) return;

  WiFi.mode(WIFI_AP);

  WiFi.softAP(
    AP_SSID,
    AP_PASSWORD,
    AP_CHANNEL,
    AP_HIDE_SSID
  );

  // Important for iPhone/iOS: do not advertise the ESP32 as the
  // default gateway. Local traffic to 192.168.4.1 still works,
  // while Internet traffic can remain on the mobile-data connection.
  configureSoftApDhcpNoDefaultGateway();

  IPAddress ip = WiFi.softAPIP();

  // Start mDNS so http://exhaust.local can be used in addition
  // to the direct AP address http://192.168.4.1.
  startMdns();

  registerHttpRoutes();
  server.begin();

  lastHeartbeat = millis();
  communicationServicesStarted = true;

  SERIAL_PRINTLN();
  SERIAL_PRINTLN("==================================");
  SERIAL_PRINTLN("Communication services STARTED");
  SERIAL_PRINT("WiFi SSID: ");
  SERIAL_PRINTLN(AP_SSID);
  SERIAL_PRINT("WiFi IP:   ");
  SERIAL_PRINTLN(ip);
  SERIAL_PRINTLN("==================================");
}

void stopCommunicationServices() {
  if (communicationServicesStarted) {
    server.stop();
  }

  MDNS.end();

  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  communicationServicesStarted = false;

  SERIAL_PRINTLN("Communication services STOPPED");
}

// ============================================================
// EMERGENCY RESET BUTTON
// ============================================================

void clearEmergencyAndRestart() {
  SERIAL_PRINTLN("Physical emergency reset accepted");

  // Require a new release/press even if clearing NVS fails. A held button
  // must not cause repeated flash writes on every loop iteration.
  emergencyResetButtonArmed = false;
  lastEmergencyResetButtonState = digitalRead(EMERGENCY_RESET_PIN);
  lastEmergencyResetButtonChangedAt = millis();

  // Keep the lock active if its persistent state could not be cleared.
  if (!saveEmergencyState(false)) {
    SERIAL_PRINTLN("Emergency reset failed -> stay offline");
    return;
  }

  emergencyShutdown = false;
  communicationShutdownRequested = false;

  currentMode = MODE_AUTO;

  // Always return to safe AUTO outputs.
  setRelay(false);
  pwmOff();
  digitalWrite(STATUS_LED_PIN, LED_OFF);

  startCommunicationServices();

  SERIAL_PRINTLN("Emergency cleared -> AUTO");
}

void handleEmergencyResetButton() {
  if (!emergencyShutdown) return;

  const bool pressed = (digitalRead(EMERGENCY_RESET_PIN) == LOW);
  const unsigned long now = millis();

  // IMPORTANT:
  // If GPIO0 was held during boot, do not clear emergency immediately.
  // First require a full release, then a new press.
  if (!emergencyResetButtonArmed) {
    if (!pressed) {
      emergencyResetButtonArmed = true;
      lastEmergencyResetButtonState = HIGH;
      lastEmergencyResetButtonChangedAt = now;

      SERIAL_PRINTLN("Emergency reset button armed (released)");
    }
    return;
  }

  // Detect button state changes.
  const bool lastPressed = (lastEmergencyResetButtonState == LOW);

  if (pressed != lastPressed) {
    lastEmergencyResetButtonState = pressed ? LOW : HIGH;
    lastEmergencyResetButtonChangedAt = now;

    if (pressed) {
      SERIAL_PRINTLN("Emergency reset button pressed - hold for 5 seconds");
    }
  }

  // Emergency is cleared only after a continuous 5-second press.
  if (pressed &&
      (now - lastEmergencyResetButtonChangedAt >= EMERGENCY_RESET_HOLD_MS)) {
    clearEmergencyAndRestart();
  }
}

// ============================================================
// PWM FUNCTIONS
// ============================================================


void markPwmUnavailable() {
  pwmReady = false;
#ifdef USE_LEDC_OLD_API
  ledcDetachPin(PWM_PIN);
#else
  ledcDetach(PWM_PIN);
#endif
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  SERIAL_PRINTLN("PWM: ERROR -> OPEN unavailable");
}

bool pwmOff() {
  if (!pwmReady) {
    digitalWrite(PWM_PIN, LOW);
    return false;
  }

#ifdef USE_LEDC_OLD_API
  // Arduino-ESP32 2.x has no return value for writes or pin attachment.
  ledcWrite(PWM_CHANNEL, 0);
#else
  if (!ledcWrite(PWM_PIN, 0)) {
    markPwmUnavailable();
    return false;
  }
#endif
  return true;
}

bool pwmWritePercent(uint8_t percent) {
  if (!pwmReady) return false;
  if (percent > 100) percent = 100;
  
  uint8_t invertedPercent = 100 - percent;

  const uint32_t maxDuty = (1UL << PWM_RESOLUTION_BITS) - 1UL;
  const uint32_t duty =
      ((uint32_t)invertedPercent * maxDuty) / 100UL;

#ifdef USE_LEDC_OLD_API
  ledcWrite(PWM_CHANNEL, duty);
#else
  if (!ledcWrite(PWM_PIN, duty)) {
    markPwmUnavailable();
    return false;
  }
#endif
  return true;
}

void setupPwm() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);

#ifdef USE_LEDC_OLD_API
  pwmReady = (ledcSetup(PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS) > 0);
  if (pwmReady) {
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  }
#else
  pwmReady = ledcAttach(PWM_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#endif

  if (!pwmReady) {
    markPwmUnavailable();
    return;
  }
  pwmOff();
}

// ============================================================
// OUTPUT CONTROL
// ============================================================

void setRelay(bool on) {
  digitalWrite(RELAY_PIN, on ? RELAY_ON : RELAY_OFF);
}

void applyOutputs() {
  if (emergencyShutdown || !pwmReady) {
    currentMode = MODE_AUTO;
    setRelay(false);
    pwmOff();
    digitalWrite(STATUS_LED_PIN, LED_OFF);
    return;
  }

  switch (currentMode) {
    case MODE_AUTO:
      // Original vehicle controller gets control.
      setRelay(false);
      pwmOff();
      digitalWrite(STATUS_LED_PIN, LED_OFF);
      break;

    case MODE_OPEN:
      // Only connect the override signal after LEDC accepted the PWM write.
      if (!pwmWritePercent(OPEN_PWM_DUTY_PERCENT)) {
        currentMode = MODE_AUTO;
        setRelay(false);
        pwmOff();
        digitalWrite(STATUS_LED_PIN, LED_OFF);
        break;
      }
      setRelay(true);
      digitalWrite(STATUS_LED_PIN, LED_ON);
      break;
  }
}

const char* modeToString() {
  switch (currentMode) {
    case MODE_AUTO: return "AUTO";
    case MODE_OPEN: return "OPEN";
  }
  return "AUTO";
}

bool setMode(ControlMode mode) {
  if (emergencyShutdown) return false;

  if (mode != MODE_AUTO && mode != MODE_OPEN) {
    return false;
  }

  currentMode = mode;
  // Respond to the requested mode before any potentially slow flash writes.
  applyOutputs();
  if (currentMode != mode) return false;

  if (rememberLastMode && (rememberedMode != mode || !persistenceOk)) {
    if (!saveModePreference(true, mode)) return false;
    rememberedMode = mode;
  }

  return true;
}

// ============================================================
// JSON STATUS
// ============================================================

String makeStatusJson() {
  uint8_t effectiveDuty = 0;

  if (currentMode == MODE_OPEN) {
    effectiveDuty = OPEN_PWM_DUTY_PERCENT;
  }

  String json = "{";
  json += "\"mode\":\"";
  json += modeToString();
  json += "\",";
  json += "\"duty\":";
  json += String(effectiveDuty);
  json += ",";
  json += "\"relay\":";
  json += (currentMode == MODE_AUTO ? "false" : "true");
  json += ",";
  json += "\"frequency\":";
  json += String(PWM_FREQUENCY_HZ);
  json += ",";
  json += "\"failsafe\":";
  json += (FAILSAFE_ENABLED ? "true" : "false");
  json += ",";
  json += "\"emergency\":";
  json += (emergencyShutdown ? "true" : "false");
  json += ",";
  json += "\"rememberMode\":";
  json += (rememberLastMode ? "true" : "false");
  json += ",";
  json += "\"pwmReady\":";
  json += (pwmReady ? "true" : "false");
  json += ",";
  json += "\"persistenceOk\":";
  json += (persistenceOk ? "true" : "false");
  json += "}";

  return json;
}

// ============================================================
// EMERGENCY SHUTDOWN
// ============================================================

void emergencyShutdownNow() {
  if (emergencyShutdown) return;

  emergencyShutdown = true;
  currentMode = MODE_AUTO;

  // SAFE OUTPUTS FIRST.
  setRelay(false);
  pwmOff();
  digitalWrite(STATUS_LED_PIN, LED_OFF);

  // Persist the emergency state so it survives power cycles.
  saveEmergencyState(true);

  SERIAL_PRINTLN("==================================");
  SERIAL_PRINTLN("EMERGENCY SHUTDOWN");
  SERIAL_PRINTLN("Relay: OFF");
  SERIAL_PRINTLN("PWM:   OFF");
  SERIAL_PRINTLN("WiFi:  OFF");
  SERIAL_PRINTLN("Hold physical GPIO0 button for 5 seconds to reset.");
  SERIAL_PRINTLN("==================================");

  // The HTTP handler reports the output and storage result before loop()
  // stops communication. Loss of Wi-Fi alone is not a shutdown confirmation.
}

// ============================================================
// LOCAL HTTP API
// ============================================================
// The local web UI and HTTP API are available for local control.
// ============================================================
// WEB SERVER HANDLERS
// ============================================================

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json", makeStatusJson());
}

void handleMode() {
  if (emergencyShutdown) {
    server.send(503, "text/plain", "Emergency shutdown active");
    return;
  }

  if (!server.hasArg("mode")) {
    server.send(400, "text/plain", "Missing mode");
    return;
  }

  String mode = server.arg("mode");
  mode.toUpperCase();

  ControlMode requestedMode;
  if (mode == "AUTO") {
    requestedMode = MODE_AUTO;
  } else if (mode == "OPEN") {
    requestedMode = MODE_OPEN;
  } else {
    server.send(400, "text/plain", "Invalid mode");
    return;
  }

  lastHeartbeat = millis();
  if (!setMode(requestedMode)) {
    if (requestedMode == MODE_OPEN && currentMode != MODE_OPEN) {
      server.send(503, "text/plain; charset=utf-8",
                  "PWM nicht verfügbar. AUTO bleibt aktiv.");
    } else {
      server.send(500, "text/plain; charset=utf-8",
                  "Modus angewendet, aber die Starteinstellung konnte nicht gespeichert werden.");
    }
    return;
  }
  server.send(200, "application/json", makeStatusJson());
}

void handlePreferences() {
  if (emergencyShutdown) {
    server.send(503, "text/plain", "Emergency shutdown active");
    return;
  }

  if (!server.hasArg("remember")) {
    server.send(400, "text/plain", "Missing remember");
    return;
  }

  String value = server.arg("remember");
  value.toLowerCase();

  bool enabled;
  if (value == "true" || value == "1" || value == "on") {
    enabled = true;
  } else if (value == "false" || value == "0" || value == "off") {
    enabled = false;
  } else {
    server.send(400, "text/plain", "Invalid remember value");
    return;
  }

  if (enabled == rememberLastMode && persistenceOk) {
    server.send(200, "application/json", makeStatusJson());
    return;
  }

  if (!saveModePreference(enabled, currentMode)) {
    server.send(500, "text/plain; charset=utf-8",
                "Die Starteinstellung konnte nicht gespeichert werden.");
    return;
  }

  // Reflect only successfully stored preferences in the UI.
  rememberLastMode = enabled;
  if (enabled) rememberedMode = currentMode;

  server.send(200, "application/json", makeStatusJson());
}

void handleEmergency() {
  if (!EMERGENCY_ENABLED) {
    server.send(404, "text/plain", "Emergency disabled");
    return;
  }

  // Disable outputs and check persistence before confirming the shutdown.
  // Keep HTTP alive just long enough to send this result; loop() stops Wi-Fi.
  emergencyShutdownNow();
  String response = "{\"emergency\":true,\"persistenceOk\":";
  response += (persistenceOk ? "true" : "false");
  response += "}";
  server.send(200, "application/json", response);
  communicationShutdownRequested = true;
}

void handleHeartbeat() {
  if (emergencyShutdown) return;

  lastHeartbeat = millis();
  server.send(200, "text/plain", "OK");
}

// ============================================================
// mDNS
// ============================================================

void startMdns() {
  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);

    SERIAL_PRINT("mDNS: http://");
    SERIAL_PRINT(MDNS_HOSTNAME);
    SERIAL_PRINTLN(".local");
  } else {
    SERIAL_PRINTLN("mDNS: start failed");
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  SERIAL_LOG_BEGIN(115200);
  delay(200);

  // ----------------------------------------------------------
  // Hardware / safe outputs.
  // ----------------------------------------------------------

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  // Physical emergency reset button:
  // GPIO0 -> button -> GND
  pinMode(EMERGENCY_RESET_PIN, INPUT_PULLUP);

  setRelay(false);
  digitalWrite(STATUS_LED_PIN, LED_OFF);

  currentMode = MODE_AUTO;
  setupPwm();
  applyOutputs();

  // ----------------------------------------------------------
  // Load persistent emergency state.
  // ----------------------------------------------------------

  if (EMERGENCY_ENABLED) {
    emergencyShutdown = loadEmergencyState();
  } else {
    emergencyShutdown = false;
  }

  // Load the optional remembered AUTO/OPEN startup mode.
  loadModePreference();

  communicationShutdownRequested = false;

  // If emergency was stored in NVS, stay completely offline.
  if (emergencyShutdown) {
    setRelay(false);
    pwmOff();
    digitalWrite(STATUS_LED_PIN, LED_OFF);

    // Do NOT start Wi-Fi or HTTP server.
    // The physical GPIO0 button is the only reset path.
    emergencyResetButtonArmed = false;
    lastEmergencyResetButtonState =
      (digitalRead(EMERGENCY_RESET_PIN) == LOW) ? LOW : HIGH;
    lastEmergencyResetButtonChangedAt = millis();

    SERIAL_PRINTLN();
    SERIAL_PRINTLN("==================================");
    SERIAL_PRINTLN("ESP32 Exhaust Controller");
    SERIAL_PRINTLN("PERSISTENT EMERGENCY ACTIVE");
    SERIAL_PRINTLN("Relay: OFF");
    SERIAL_PRINTLN("PWM:   OFF");
    SERIAL_PRINTLN("WiFi:  OFF");
    SERIAL_PRINTLN("Release GPIO0 first, then press it");
    SERIAL_PRINTLN("to clear emergency.");
    SERIAL_PRINTLN("==================================");

    return;
  }

  // ----------------------------------------------------------
  // Normal startup.
  // ----------------------------------------------------------

  if (rememberLastMode) {
    currentMode = rememberedMode;
  } else {
    currentMode = MODE_AUTO;
  }

  applyOutputs();
  startCommunicationServices();

  SERIAL_PRINTLN();
  SERIAL_PRINTLN("==================================");
  SERIAL_PRINTLN("ESP32 Exhaust Controller");
  SERIAL_PRINTLN("==================================");
  SERIAL_PRINT("Mode: ");
  SERIAL_PRINTLN(currentMode == MODE_OPEN ? "OPEN" : "AUTO");
  SERIAL_PRINTLN("==================================");
}


// ============================================================
// LOOP
// ============================================================

void loop() {
  // Finish an acknowledged HTTP shutdown before accepting another command.
  if (communicationShutdownRequested) {
    communicationShutdownRequested = false;
    stopCommunicationServices();
    return;
  }

  // ----------------------------------------------------------
  // Physical emergency reset button.
  // This must also run while communications are OFF.
  // ----------------------------------------------------------

  if (emergencyShutdown) {
    handleEmergencyResetButton();

    if (emergencyShutdown) {
      // Keep outputs in the safe state.
      setRelay(false);
      pwmOff();
      digitalWrite(STATUS_LED_PIN, LED_OFF);

      // Conservative emergency loop timing: outputs remain safely OFF while
      // avoiding unnecessary CPU spinning during the offline state.
      delay(20);
      return;
    }
  }

  // ----------------------------------------------------------
  // Wi-Fi web server.
  // ----------------------------------------------------------

  if (communicationServicesStarted) {
    server.handleClient();
  }

  // ----------------------------------------------------------
  // Optional communication failsafe.
  // ----------------------------------------------------------

  if (communicationServicesStarted &&
      FAILSAFE_ENABLED &&
      currentMode != MODE_AUTO &&
      (millis() - lastHeartbeat > FAILSAFE_TIMEOUT_MS)) {

    SERIAL_PRINTLN("FAILSAFE: No heartbeat -> AUTO");

    setMode(MODE_AUTO);
    lastHeartbeat = millis();

    }

  // 5 ms keeps the controller responsive while reducing needless CPU load
  // compared with a 1 ms busy loop.
  delay(5);
}
