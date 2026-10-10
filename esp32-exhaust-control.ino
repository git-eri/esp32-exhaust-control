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

// Set to true for serial debug output, false for completely silent UART.
#define SERIAL_OUTPUT_ENABLED   true

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
// HARDWARE CONFIGURATION
// ============================================================

#define RELAY_PIN               16
#define STATUS_LED_PIN          23
#define PWM_PIN                 25

#define PWM_FREQUENCY_HZ        100
#define PWM_RESOLUTION_BITS     10

// These are the current working values.
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
#define AP_SSID                 "WiFi"
#define AP_PASSWORD             "Exhaust123"
#define AP_CHANNEL              6

// Local hostname: http://exhaust.local
#define MDNS_HOSTNAME           "exhaust"


// ============================================================
// OPTIONAL FAILSAFE
// ============================================================

// If enabled, AUTO is selected when the web UI has not refreshed
// the heartbeat within FAILSAFE_TIMEOUT_MS.
//
// The web UI sends a heartbeat every 5 seconds.
//
// IMPORTANT:
// This is NOT a substitute for proper electrical/mechanical safety.
#define FAILSAFE_ENABLED        false
#define FAILSAFE_TIMEOUT_MS     30000UL

// ============================================================
// EMERGENCY SHUTDOWN
// ============================================================


#define EMERGENCY_ENABLED       true

// Physical emergency reset button.
// Wire the button between GPIO0 and GND.
// GPIO0 is active LOW and uses the internal pull-up.
#define EMERGENCY_RESET_PIN     0
#define EMERGENCY_RESET_HOLD_MS     5000UL

// Persistent emergency state in ESP32 NVS (flash).
#define NVS_NAMESPACE           "exhaust"
#define NVS_EMERGENCY_KEY       "emergency"
#define NVS_REMEMBER_MODE_KEY   "rememberMode"
#define NVS_START_MODE_KEY      "startMode"

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

#ifdef USE_LEDC_OLD_API
  #define PWM_CHANNEL 0
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
// WEB PAGE
// ============================================================

// Local WLAN web UI.
// This page uses only the local HTTP API.
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<meta name="theme-color" content="#4da3ff">
<title>Exhaust Control</title>

<style>
  :root {
    --bg: #0f1115;
    --card: #191c23;
    --card2: #222631;
    --text: #f4f5f7;
    --muted: #9ba3b2;
    --accent: #4da3ff;
    --danger: #ff6262;
    --ok: #42d392;
    --border: #303643;
  }

  * { box-sizing: border-box; }

  body {
    margin: 0;
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
    min-height: 100vh;
  }

  .wrap {
    width: 100%;
    max-width: 520px;
    margin: 0 auto;
    padding: 18px;
  }

  h1 {
    margin: 8px 0 4px;
    font-size: 28px;
    text-align: center;
  }

  .subtitle {
    text-align: center;
    color: var(--muted);
    margin-bottom: 20px;
  }

  .card {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 18px;
    padding: 18px;
    margin-bottom: 14px;
  }

  .connection {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    color: var(--muted);
    font-size: 14px;
  }

  .dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--muted);
  }

  .transport {
    text-align: center;
    color: var(--muted);
    font-size: 12px;
    margin-top: 7px;
  }

  .modes {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
  }

  .modes button {
    min-height: 110px;
    font-size: 28px;
    font-weight: 800;
  }

  .emergency {
    width: 100%;
    min-height: 68px;
    background: #6f1d1d;
    border: 2px solid #ff4b4b;
    color: white;
    font-size: 20px;
    font-weight: 800;
  }

  .emergency:active {
    background: #ff3030;
  }

  .emergency-hint {
    text-align: center;
    color: var(--muted);
    font-size: 11px;
    margin-top: 8px;
  }

  button {
    border: 1px solid var(--border);
    background: var(--card2);
    color: var(--text);
    border-radius: 14px;
    min-height: 58px;
    font-size: 17px;
    font-weight: 700;
    touch-action: manipulation;
    -webkit-tap-highlight-color: transparent;
  }

  button.active {
    background: var(--accent);
    border-color: var(--accent);
    color: white;
  }

  button:disabled, input:disabled {
    opacity: 0.45;
  }

  .message {
    margin-top: 10px;
    color: var(--muted);
    font-size: 13px;
    line-height: 1.45;
    text-align: center;
  }

  .message.error {
    color: var(--danger);
  }

  .stats.stale .stat-value {
    opacity: 0.45;
  }

  .stats {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }

  .stat {
    background: var(--card2);
    border-radius: 13px;
    padding: 12px;
  }

  .stat-label {
    color: var(--muted);
    font-size: 12px;
    margin-bottom: 4px;
  }

  .stat-value {
    font-size: 18px;
    font-weight: 700;
  }

  .warning {
    color: var(--muted);
    font-size: 12px;
    line-height: 1.45;
    text-align: center;
  }
</style>
</head>

<body>
<div class="wrap">

  <h1>Exhaust Control</h1>
  <div class="subtitle">ESP32 Controller</div>

  <div class="card">
    <button id="btnEmergency" class="emergency" onclick="emergencyShutdown()">⚠ NOT AUS</button>
  </div>

  <div class="card">
    <div class="modes">
      <button id="btnAuto" onclick="setMode('AUTO')" disabled>AUTO</button>
      <button id="btnOpen" onclick="setMode('OPEN')" disabled>OFFEN</button>
    </div>
  </div>

  <div class="card">
    <label style="display:flex;align-items:center;gap:12px;cursor:pointer;">
      <input id="rememberMode" type="checkbox" disabled
             onchange="setRememberMode(this.checked)"
             style="width:22px;height:22px;accent-color:var(--accent);">
      <span>
        <strong>Letzte Einstellung merken</strong><br>
      </span>
    </label>
  </div>

  <div class="card">
    <div class="connection">
      <span id="connectionDot" class="dot"></span>
      <span id="connectionText">Verbinde …</span>
    </div>
    <div id="transportText" class="transport">Warte auf Status</div>
    <div id="commandMessage" class="message" role="status" aria-live="polite" hidden></div>
    <div id="deviceWarning" class="message error" role="status" aria-live="polite" hidden></div>
  </div>

  <div class="card">
    <div id="statusStats" class="stats stale">
      <div class="stat">
        <div class="stat-label">MODUS</div>
        <div id="modeValue" class="stat-value">—</div>
      </div>

      <div class="stat">
        <div class="stat-label">PWM</div>
        <div id="pwmValue" class="stat-value">—</div>
      </div>

      <div class="stat">
        <div class="stat-label">RELAIS</div>
        <div id="relayValue" class="stat-value">—</div>
      </div>

      <div class="stat">
        <div class="stat-label">FREQUENZ</div>
        <div id="frequencyValue" class="stat-value">—</div>
      </div>
    </div>
  </div>

</div>

<script>
const REQUEST_TIMEOUT_MS = 4000;
const activeRequests = new Set();
let latestStatus = null;
let connected = false;
let commandBusy = false;
let emergencyPending = false;
let emergencyViewActive = false;
let pollingActive = false;
let statusTimer = null;
let heartbeatTimer = null;
let requestGeneration = 0;
let statusRevision = 0;

async function request(url, method = 'GET', expectJson = true) {
  const controller = new AbortController();
  activeRequests.add(controller);
  const timeout = setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS);

  try {
    const response = await fetch(url, {
      method,
      cache: 'no-store',
      signal: controller.signal
    });

    if (!response.ok) {
      const error = new Error('ESP32 hat den Befehl abgelehnt.');
      error.httpStatus = response.status;
      throw error;
    }

    return expectJson ? await response.json() : await response.text();
  } finally {
    clearTimeout(timeout);
    activeRequests.delete(controller);
  }
}

function showCommandMessage(message, isError = true) {
  if (emergencyViewActive) return;
  const element = document.getElementById('commandMessage');
  element.textContent = message;
  element.hidden = !message;
  element.classList.toggle('error', isError);
}

function updateControls() {
  if (emergencyViewActive) return;
  const disabled = commandBusy || emergencyPending || !connected;
  document.getElementById('btnAuto').disabled = disabled;
  document.getElementById('btnOpen').disabled =
    disabled || (latestStatus && latestStatus.pwmReady === false);
  document.getElementById('rememberMode').disabled = disabled;
  document.getElementById('btnEmergency').disabled = emergencyPending;
}

function setConnection(isConnected) {
  if (emergencyViewActive) return;
  connected = isConnected;
  document.getElementById('connectionText').textContent =
    connected ? 'Verbunden' : 'Verbindung verloren';
  document.getElementById('connectionDot').style.background =
    connected ? 'var(--ok)' : 'var(--danger)';
  document.getElementById('transportText').textContent = connected
    ? 'Lokales WLAN'
    : (latestStatus ? 'Angezeigte Werte sind nicht mehr aktuell' : 'Status nicht verfügbar');
  document.getElementById('statusStats').classList.toggle('stale', !connected);
  updateControls();
}

function updateUI(data) {
  if (emergencyPending || emergencyViewActive) return;
  latestStatus = data;

  document.getElementById('modeValue').textContent = data.mode;
  document.getElementById('pwmValue').textContent = data.duty + ' %';
  document.getElementById('relayValue').textContent = data.relay ? 'EIN' : 'AUS';
  document.getElementById('frequencyValue').textContent = data.frequency + ' Hz';

  document.getElementById('btnAuto').classList.toggle('active', data.mode === 'AUTO');
  document.getElementById('btnOpen').classList.toggle('active', data.mode === 'OPEN');
  document.getElementById('rememberMode').checked = !!data.rememberMode;

  const warnings = [];
  if (data.pwmReady === false) warnings.push('PWM nicht verfügbar. OFFEN ist gesperrt.');
  if (data.persistenceOk === false) {
    warnings.push('Dauerhafte Einstellungen konnten nicht gelesen oder gespeichert werden.');
  }
  const warning = document.getElementById('deviceWarning');
  warning.textContent = warnings.join(' ');
  warning.hidden = warnings.length === 0;
  setConnection(true);
}

async function getStatus() {
  if (commandBusy || emergencyPending || emergencyViewActive) return;
  const generation = requestGeneration;
  const revision = statusRevision;

  try {
    const data = await request('/api/status');
    if (generation === requestGeneration && revision === statusRevision) updateUI(data);
  } catch (e) {
    if (generation === requestGeneration && revision === statusRevision) setConnection(false);
  }
}

async function sendCommand(url, failureMessage) {
  if (commandBusy || emergencyPending || emergencyViewActive) return;
  const generation = requestGeneration;
  commandBusy = true;
  ++statusRevision;
  showCommandMessage('', false);
  updateControls();

  try {
    const data = await request(url, 'POST');
    if (generation !== requestGeneration) return;
    commandBusy = false;
    updateUI(data);
  } catch (e) {
    if (generation !== requestGeneration) return;
    commandBusy = false;
    showCommandMessage(failureMessage);
    if (latestStatus) {
      document.getElementById('rememberMode').checked = !!latestStatus.rememberMode;
    }
    await getStatus();
  } finally {
    if (generation === requestGeneration) {
      commandBusy = false;
      updateControls();
    }
  }
}

async function setMode(mode) {
  await sendCommand('/api/mode?mode=' + encodeURIComponent(mode),
    'Modusänderung nicht bestätigt. Bitte den angezeigten Zustand prüfen.');
}

async function setRememberMode(enabled) {
  await sendCommand('/api/preferences?remember=' + (enabled ? 'true' : 'false'),
    'Speichern nicht bestätigt. Bitte den angezeigten Zustand prüfen.');
}

function stopPolling() {
  pollingActive = false;
  ++requestGeneration;
  clearTimeout(statusTimer);
  clearTimeout(heartbeatTimer);
  statusTimer = null;
  heartbeatTimer = null;
  for (const controller of activeRequests) controller.abort();
}

async function emergencyShutdown() {
  if (emergencyPending || emergencyViewActive) return;
  emergencyPending = true;
  stopPolling();
  updateControls();
  showCommandMessage('Abschaltung wird angefordert …', false);

  try {
    const data = await request('/api/emergency', 'POST');
    if (data.emergency !== true) throw new Error('Abschaltung nicht bestätigt');

    emergencyViewActive = true;
    stopPolling();
    const persistenceWarning = data.persistenceOk === false
      ? '<p style="color:#ff6262;font-size:13px">Abschaltung aktiv, aber Speicherung fehlgeschlagen.<br>Nach einem Stromausfall ist die Abschaltung möglicherweise nicht mehr aktiv.</p>'
      : '';

    document.body.innerHTML = `
      <div style="min-height:100vh;display:flex;align-items:center;justify-content:center;
                  text-align:center;padding:25px;font-family:Arial,sans-serif;
                  background:#0f1115;color:#fff;">
        <div>
          <div style="font-size:64px">⚠</div>
          <h1>EMERGENCY OFF</h1>
          <p style="color:#9ba3b2">Abschaltung bestätigt<br>Relais AUS<br>PWM AUS<br>WLAN wird abgeschaltet</p>
          ${persistenceWarning}
          <p style="color:#9ba3b2;font-size:13px">
            ESP32 bleibt stromversorgt.<br>
            Zum Zurücksetzen den GPIO0-Taster zuerst loslassen,<br>
            dann 5 Sekunden gedrückt halten.
          </p>
          <button onclick="location.reload()"
                  style="margin-top:18px;width:100%;min-height:58px;padding:12px 18px;
                         border:1px solid #303643;border-radius:14px;background:#222631;
                         color:#fff;font-size:17px;font-weight:700;cursor:pointer;
                         touch-action:manipulation;">
            🔄 Aktualisieren
          </button>
        </div>
      </div>`;
  } catch (e) {
    emergencyPending = false;
    commandBusy = false;
    showCommandMessage('Abschaltung nicht bestätigt. Erneut versuchen oder den Zustand am Gerät prüfen.');
    setConnection(false);
    startPolling();
  }
}

async function heartbeat() {
  try {
    await request('/api/heartbeat', 'POST', false);
  } catch (e) {
    // Status polling provides connection feedback without hiding command errors.
  }
}

async function pollStatus() {
  const generation = requestGeneration;
  await getStatus();
  if (pollingActive && generation === requestGeneration) {
    statusTimer = setTimeout(pollStatus, 1000);
  }
}

async function pollHeartbeat() {
  const generation = requestGeneration;
  await heartbeat();
  if (pollingActive && generation === requestGeneration) {
    heartbeatTimer = setTimeout(pollHeartbeat, 5000);
  }
}

function startPolling() {
  if (pollingActive || emergencyPending || emergencyViewActive) return;
  pollingActive = true;
  pollStatus();
  heartbeatTimer = setTimeout(pollHeartbeat, 5000);
}

startPolling();
</script>

</body>
</html>
)rawliteral";

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
