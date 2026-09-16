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
    MANUAL = relay ON, PWM controlled by slider 0..100 %

  Communication:
    1) Wi-Fi AP + Web UI
    2) Bluetooth Low Energy (BLE) in parallel
       - intended as a fallback / alternative control path
       - compatible with Web Bluetooth clients such as Bluefy

  IMPORTANT:
    GPIO25 is a 3.3 V logic output. Do NOT connect it directly to a 12 V circuit.
    Use the intended transistor/driver stage between GPIO25 and the vehicle's
    12 V PWM input.

  BLE protocol:
    Service UUID:
      7f6e0001-6d8b-4c7a-9a11-3e5c2b8d1001

    Command characteristic (WRITE / WRITE WITHOUT RESPONSE):
      7f6e0002-6d8b-4c7a-9a11-3e5c2b8d1001

    Status characteristic (READ / NOTIFY):
      7f6e0003-6d8b-4c7a-9a11-3e5c2b8d1001

    Commands are plain UTF-8 text:
      MODE:AUTO
      MODE:OPEN
      MODE:MANUAL
      DUTY:0..100
      HEARTBEAT
      EMERGENCY

    Status is JSON, for example:
      {"mode":"MANUAL","duty":50,"relay":true,"frequency":100,"failsafe":true}

  Notes:
    - BLE and Wi-Fi control the same internal controller state.
    - Emergency shutdown disables both Wi-Fi and BLE.
    - A reset or power-cycle is required after emergency shutdown.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
#define OPEN_PWM_DUTY_PERCENT   90
#define CLOSED_PWM_DUTY_PERCENT 10

#define RELAY_ON                HIGH
#define RELAY_OFF               LOW

#define LED_ON                  HIGH
#define LED_OFF                 LOW

// ============================================================
// WIFI CONFIGURATION
// ============================================================

#define AP_HIDE_SSID            false
#define AP_SSID                 "Exhaust-ESP32"
#define AP_PASSWORD             "Exhaust123"
#define AP_CHANNEL              6


// ============================================================
// BLUETOOTH LOW ENERGY CONFIGURATION
// ============================================================

#define BLE_ENABLED             false
#define BLE_DEVICE_NAME         "Exhaust-ESP32"

#define BLE_SERVICE_UUID        "7f6e0001-6d8b-4c7a-9a11-3e5c2b8d1001"
#define BLE_COMMAND_UUID        "7f6e0002-6d8b-4c7a-9a11-3e5c2b8d1001"
#define BLE_STATUS_UUID         "7f6e0003-6d8b-4c7a-9a11-3e5c2b8d1001"

// Send status notifications periodically when a BLE client is connected.
#define BLE_STATUS_INTERVAL_MS  500UL

// ============================================================
// OPTIONAL FAILSAFE
// ============================================================

// If enabled, AUTO is selected when neither communication path
// has refreshed the heartbeat within FAILSAFE_TIMEOUT_MS.
//
// The web UI sends a heartbeat every 5 seconds.
// A BLE client should send "HEARTBEAT" periodically.
//
// IMPORTANT:
// This is NOT a substitute for proper electrical/mechanical safety.
#define FAILSAFE_ENABLED        false
#define FAILSAFE_TIMEOUT_MS     30000UL

// ============================================================
// EMERGENCY SHUTDOWN
// ============================================================

#define EMERGENCY_ENABLED       true

// ============================================================
// GLOBALS
// ============================================================

WebServer server(80);

enum ControlMode {
  MODE_AUTO,
  MODE_OPEN,
  MODE_MANUAL
};

ControlMode currentMode = MODE_AUTO;
uint8_t manualDutyPercent = 0;

unsigned long lastHeartbeat = 0;
unsigned long lastBleStatus = 0;

bool emergencyShutdown = false;

// BLE objects
BLEServer* bleServer = nullptr;
BLECharacteristic* bleCommandCharacteristic = nullptr;
BLECharacteristic* bleStatusCharacteristic = nullptr;

bool bleClientConnected = false;

// BLE callbacks only set this flag. The actual shutdown is performed
// from loop(), after the callback has returned.
volatile bool emergencyShutdownRequested = false;

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
// PWM FUNCTIONS
// ============================================================

void pwmOff() {
#ifdef USE_LEDC_OLD_API
  ledcWrite(PWM_CHANNEL, 0);
#else
  ledcWrite(PWM_PIN, 0);
#endif
}

void pwmWritePercent(uint8_t percent) {
  if (percent > 100) percent = 100;

  const uint32_t maxDuty = (1UL << PWM_RESOLUTION_BITS) - 1UL;
  const uint32_t duty = ((uint32_t)percent * maxDuty) / 100UL;

#ifdef USE_LEDC_OLD_API
  ledcWrite(PWM_CHANNEL, duty);
#else
  ledcWrite(PWM_PIN, duty);
#endif
}

void setupPwm() {
#ifdef USE_LEDC_OLD_API
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
#else
  ledcAttach(PWM_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#endif

  pwmOff();
}

// ============================================================
// OUTPUT CONTROL
// ============================================================

void setRelay(bool on) {
  digitalWrite(RELAY_PIN, on ? RELAY_ON : RELAY_OFF);
}

void applyOutputs() {
  if (emergencyShutdown) {
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
      setRelay(true);
      pwmWritePercent(OPEN_PWM_DUTY_PERCENT);
      digitalWrite(STATUS_LED_PIN, LED_ON);
      break;

    case MODE_MANUAL:
      setRelay(true);
      pwmWritePercent(manualDutyPercent);
      digitalWrite(STATUS_LED_PIN, LED_ON);
      break;
  }
}

const char* modeToString() {
  switch (currentMode) {
    case MODE_AUTO:   return "AUTO";
    case MODE_OPEN:   return "OPEN";
    case MODE_MANUAL: return "MANUAL";
  }
  return "AUTO";
}

void setMode(ControlMode mode) {
  if (emergencyShutdown) return;

  currentMode = mode;
  applyOutputs();
}

// ============================================================
// JSON STATUS
// ============================================================

String makeStatusJson() {
  uint8_t effectiveDuty = 0;

  if (currentMode == MODE_OPEN) {
    effectiveDuty = OPEN_PWM_DUTY_PERCENT;
  } else if (currentMode == MODE_MANUAL) {
    effectiveDuty = manualDutyPercent;
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
  json += "\"ble\":";
  json += (bleClientConnected ? "true" : "false");
  json += ",";
  json += "\"emergency\":";
  json += (emergencyShutdown ? "true" : "false");
  json += "}";

  return json;
}

// ============================================================
// BLE STATUS
// ============================================================

void bleNotifyStatus() {
  if (!BLE_ENABLED) return;
  if (!bleClientConnected) return;
  if (bleStatusCharacteristic == nullptr) return;
  if (emergencyShutdown) return;

  String status = makeStatusJson();

  bleStatusCharacteristic->setValue(status.c_str());
  bleStatusCharacteristic->notify();
}

// ============================================================
// BLE SERVER CALLBACKS
// ============================================================

class ExhaustBLEServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;

    Serial.println("BLE: client connected");

    bleNotifyStatus();
  }

  void onDisconnect(BLEServer* server) override {
    bleClientConnected = false;

    Serial.println("BLE: client disconnected");

    if (!emergencyShutdown) {
      server->getAdvertising()->start();
      Serial.println("BLE: advertising restarted");
    }
  }
};

void processCommand(String command) {
  if (emergencyShutdown) return;

  command.trim();
  command.toUpperCase();

  if (command.length() == 0) return;

  Serial.print("BLE command: ");
  Serial.println(command);

  if (command == "HEARTBEAT") {
    lastHeartbeat = millis();
    bleNotifyStatus();
    return;
  }

  if (command == "EMERGENCY") {
    if (EMERGENCY_ENABLED) {
      emergencyShutdownRequested = true;
    }
    return;
  }

  if (command == "MODE:AUTO") {
    setMode(MODE_AUTO);
    lastHeartbeat = millis();
    bleNotifyStatus();
    return;
  }

  if (command == "MODE:OPEN") {
    setMode(MODE_OPEN);
    lastHeartbeat = millis();
    bleNotifyStatus();
    return;
  }

  if (command == "MODE:MANUAL") {
    setMode(MODE_MANUAL);
    lastHeartbeat = millis();
    bleNotifyStatus();
    return;
  }

  if (command.startsWith("DUTY:")) {
    String valueString = command.substring(5);
    int value = valueString.toInt();

    if (value < 0) value = 0;
    if (value > 100) value = 100;

    manualDutyPercent = (uint8_t)value;
    setMode(MODE_MANUAL);
    lastHeartbeat = millis();

    bleNotifyStatus();
    return;
  }

  Serial.println("BLE: unknown command");
}

class ExhaustBLECommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String command = characteristic->getValue();

    if (command.length() == 0) return;

    // Allow one or more newline-separated commands.
    int start = 0;

    while (start < command.length()) {
      int end = command.indexOf('\n', start);

      if (end < 0) {
        end = command.length();
      }

      String oneCommand = command.substring(start, end);
      oneCommand.trim();

      if (oneCommand.length() > 0) {
        processCommand(oneCommand);
      }

      start = end + 1;
    }
  }
};

void setupBle() {
  if (!BLE_ENABLED) return;

  BLEDevice::init(BLE_DEVICE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ExhaustBLEServerCallbacks());

  BLEService* service = bleServer->createService(BLE_SERVICE_UUID);

  bleCommandCharacteristic = service->createCharacteristic(
    BLE_COMMAND_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  bleCommandCharacteristic->setCallbacks(
    new ExhaustBLECommandCallbacks()
  );

  bleStatusCharacteristic = service->createCharacteristic(
    BLE_STATUS_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  // Required by many Web Bluetooth clients for notifications.
  bleStatusCharacteristic->addDescriptor(new BLE2902());

  bleStatusCharacteristic->setValue(makeStatusJson().c_str());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();

  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.println("BLE: initialized");
  Serial.print("BLE name: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.print("BLE service UUID: ");
  Serial.println(BLE_SERVICE_UUID);
}

void stopBle() {
  if (!BLE_ENABLED) return;

  bleClientConnected = false;

  BLEDevice::stopAdvertising();

  // Release the BLE stack.
  BLEDevice::deinit(true);

  bleServer = nullptr;
  bleCommandCharacteristic = nullptr;
  bleStatusCharacteristic = nullptr;

  Serial.println("BLE: OFF");
}

// ============================================================
// EMERGENCY SHUTDOWN
// ============================================================

void emergencyShutdownNow() {
  if (emergencyShutdown) return;

  emergencyShutdown = true;
  emergencyShutdownRequested = false;

  // SAFE OUTPUTS FIRST.
  setRelay(false);
  pwmOff();
  digitalWrite(STATUS_LED_PIN, LED_OFF);

  Serial.println("==================================");
  Serial.println("EMERGENCY SHUTDOWN");
  Serial.println("Relay: OFF");
  Serial.println("PWM:   OFF");
  Serial.println("WiFi:  OFF");
  Serial.println("BLE:   OFF");
  Serial.println("Reset/power-cycle required.");
  Serial.println("==================================");

  // Stop communication after the outputs are safe.
  server.stop();

  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  stopBle();
}

// ============================================================
// WEB PAGE
// ============================================================

// Local WLAN web UI.
// This page uses only the local HTTP API and contains no BLE/Web-Bluetooth code.
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
    background: var(--ok);
  }

  .transport {
    text-align: center;
    color: var(--muted);
    font-size: 12px;
    margin-top: 7px;
  }

  .modes {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 9px;
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

  .manual {
    display: none;
  }

  .manual.visible {
    display: block;
  }

  .value {
    text-align: center;
    font-size: 48px;
    font-weight: 800;
    margin: 4px 0 14px;
  }

  input[type=range] {
    width: 100%;
    height: 44px;
    accent-color: var(--accent);
  }

  .range-labels {
    display: flex;
    justify-content: space-between;
    color: var(--muted);
    font-size: 13px;
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

  <div class="card">
    <button class="emergency" onclick="emergencyShutdown()">⚠ EMERGENCY OFF</button>
    <div class="emergency-hint">Relais AUS · PWM AUS · WLAN AUS</div>
  </div>

  <h1>Exhaust Control</h1>
  <div class="subtitle">ESP32 Controller</div>

  <div class="card">
    <div class="connection">
      <span id="connectionDot" class="dot"></span>
      <span id="connectionText">Verbunden</span>
    </div>
    <div id="transportText" class="transport">Lokales WLAN</div>
  </div>

  <div class="card">
    <div class="modes">
      <button id="btnAuto" onclick="setMode('AUTO')">AUTO</button>
      <button id="btnOpen" onclick="setMode('OPEN')">OPEN</button>
      <button id="btnManual" onclick="setMode('MANUAL')">MANUAL</button>
    </div>
  </div>

  <div id="manualCard" class="card manual">
    <div style="text-align:center;color:#9ba3b2;margin-bottom:3px">
      Manuelle PWM
    </div>

    <div id="dutyValue" class="value">0 %</div>

    <input id="slider"
           type="range"
           min="0"
           max="100"
           value="0"
           step="1"
           oninput="sliderChanged(this.value)">

    <div class="range-labels">
      <span>0 %</span>
      <span>100 %</span>
    </div>
  </div>

  <div class="card">
    <div class="stats">
      <div class="stat">
        <div class="stat-label">MODUS</div>
        <div id="modeValue" class="stat-value">AUTO</div>
      </div>

      <div class="stat">
        <div class="stat-label">PWM</div>
        <div id="pwmValue" class="stat-value">0 %</div>
      </div>

      <div class="stat">
        <div class="stat-label">RELAIS</div>
        <div id="relayValue" class="stat-value">AUS</div>
      </div>

      <div class="stat">
        <div class="stat-label">FREQUENZ</div>
        <div id="frequencyValue" class="stat-value">100 Hz</div>
      </div>
    </div>
  </div>

  <div class="warning">
    AUTO schaltet das Relais ab und deaktiviert die PWM-Ausgabe.
    Die 12-V-PWM muss über eine geeignete Treiber-/Transistorschaltung
    erzeugt werden.
  </div>

</div>

<script>
let currentMode = 'AUTO';
let sliderTimer = null;

function updateUI(data) {
  currentMode = data.mode;

  document.getElementById('modeValue').textContent = data.mode;
  document.getElementById('pwmValue').textContent = data.duty + ' %';
  document.getElementById('relayValue').textContent = data.relay ? 'EIN' : 'AUS';
  document.getElementById('frequencyValue').textContent = data.frequency + ' Hz';

  document.getElementById('btnAuto').classList.toggle('active', data.mode === 'AUTO');
  document.getElementById('btnOpen').classList.toggle('active', data.mode === 'OPEN');
  document.getElementById('btnManual').classList.toggle('active', data.mode === 'MANUAL');

  const manualCard = document.getElementById('manualCard');
  manualCard.classList.toggle('visible', data.mode === 'MANUAL');

  if (data.mode === 'MANUAL') {
    document.getElementById('slider').value = data.duty;
    document.getElementById('dutyValue').textContent = data.duty + ' %';
  }

  document.getElementById('connectionText').textContent = 'Verbunden';
  document.getElementById('transportText').textContent = 'Lokales WLAN';
}

async function getStatus() {
  try {
    const response = await fetch('/api/status', {cache: 'no-store'});
    const data = await response.json();
    updateUI(data);
  } catch (e) {
    document.getElementById('connectionText').textContent = 'Verbindung verloren';
    document.getElementById('transportText').textContent = '';
  }
}

async function setMode(mode) {
  try {
    await fetch('/api/mode?mode=' + encodeURIComponent(mode), {
      method: 'POST',
      cache: 'no-store'
    });
    await getStatus();
  } catch (e) {}
}

function sliderChanged(value) {
  document.getElementById('dutyValue').textContent = value + ' %';

  clearTimeout(sliderTimer);

  sliderTimer = setTimeout(async () => {
    try {
      await fetch('/api/duty?value=' + encodeURIComponent(value), {
        method: 'POST',
        cache: 'no-store'
      });
      await getStatus();
    } catch (e) {}
  }, 40);
}

async function emergencyShutdown() {
  try {
    await fetch('/api/emergency', {
      method: 'POST',
      cache: 'no-store'
    });
  } catch (e) {
    // Expected: ESP32 disables Wi-Fi immediately.
  }

  document.body.innerHTML = `
    <div style="min-height:100vh;display:flex;align-items:center;justify-content:center;
                text-align:center;padding:25px;font-family:Arial,sans-serif;
                background:#0f1115;color:#fff;">
      <div>
        <div style="font-size:64px">⚠</div>
        <h1>EMERGENCY OFF</h1>
        <p style="color:#9ba3b2">Relais AUS<br>PWM AUS<br>WLAN AUS</p>
        <p style="color:#9ba3b2;font-size:13px">
          ESP32 bleibt stromversorgt.<br>
          Reset oder Power-Cycle zum Neustart.
        </p>
      </div>
    </div>`;
}

async function heartbeat() {
  try {
    await fetch('/api/heartbeat', {
      method: 'POST',
      cache: 'no-store'
    });
  } catch (e) {}
}

getStatus();
setInterval(getStatus, 1000);
setInterval(heartbeat, 5000);
</script>

</body>
</html>
)rawliteral";

// ============================================================
// LOCAL HTTP API
// ============================================================
// The local web UI has been removed to reduce flash usage.
// The HTTP API remains available for local diagnostics/control.
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

  if (mode == "AUTO") {
    setMode(MODE_AUTO);
  } else if (mode == "OPEN") {
    setMode(MODE_OPEN);
  } else if (mode == "MANUAL") {
    setMode(MODE_MANUAL);
  } else {
    server.send(400, "text/plain", "Invalid mode");
    return;
  }

  lastHeartbeat = millis();
  server.send(200, "application/json", makeStatusJson());
  bleNotifyStatus();
}

void handleDuty() {
  if (emergencyShutdown) {
    server.send(503, "text/plain", "Emergency shutdown active");
    return;
  }

  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }

  int value = server.arg("value").toInt();

  if (value < 0) value = 0;
  if (value > 100) value = 100;

  manualDutyPercent = (uint8_t)value;

  // A duty update also switches to MANUAL.
  setMode(MODE_MANUAL);

  lastHeartbeat = millis();
  server.send(200, "application/json", makeStatusJson());
  bleNotifyStatus();
}

void handleEmergency() {
  if (!EMERGENCY_ENABLED) {
    server.send(404, "text/plain", "Emergency disabled");
    return;
  }

  // Respond before communication is disabled.
  server.send(200, "application/json", "{\"emergency\":true}");
  emergencyShutdownRequested = true;
}

void handleHeartbeat() {
  if (emergencyShutdown) return;

  lastHeartbeat = millis();
  server.send(200, "text/plain", "OK");
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  // Set safe outputs BEFORE enabling the rest of the system.
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  setRelay(false);
  digitalWrite(STATUS_LED_PIN, LED_OFF);

  // Always boot in AUTO.
  currentMode = MODE_AUTO;
  manualDutyPercent = 0;

  setupPwm();
  applyOutputs();

  // ----------------------------------------------------------
  // Start ESP32 Wi-Fi access point.
  // ----------------------------------------------------------

  WiFi.mode(WIFI_AP);

  WiFi.softAP(
    AP_SSID,
    AP_PASSWORD,
    AP_CHANNEL,
    AP_HIDE_SSID
  );

  IPAddress ip = WiFi.softAPIP();

  // ----------------------------------------------------------
  // Start BLE in parallel.
  // ----------------------------------------------------------

  setupBle();

  // ----------------------------------------------------------
  // Serial information.
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("==================================");
  Serial.println("ESP32 Exhaust Controller");
  Serial.println("==================================");

  Serial.print("WiFi SSID: ");
  Serial.println(AP_SSID);

  Serial.print("WiFi IP:   ");
  Serial.println(ip);

  Serial.print("PWM pin:   ");
  Serial.println(PWM_PIN);

  Serial.print("PWM freq:  ");
  Serial.print(PWM_FREQUENCY_HZ);
  Serial.println(" Hz");

  Serial.print("BLE name:  ");
  Serial.println(BLE_DEVICE_NAME);

  Serial.print("BLE UUID:  ");
  Serial.println(BLE_SERVICE_UUID);

  Serial.println("Mode: AUTO");
  Serial.println("==================================");


  // ----------------------------------------------------------
  // Local HTTP API server.
  // ----------------------------------------------------------

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/duty", HTTP_POST, handleDuty);

  if (EMERGENCY_ENABLED) {
    server.on("/api/emergency", HTTP_POST, handleEmergency);
  }

  server.on("/api/heartbeat", HTTP_POST, handleHeartbeat);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();

  lastHeartbeat = millis();
  lastBleStatus = millis();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  // Emergency shutdown request may originate from either Wi-Fi or BLE.
  if (emergencyShutdownRequested && !emergencyShutdown) {
    emergencyShutdownNow();
  }

  if (emergencyShutdown) {
    // Keep outputs in the safe state even after communication is off.
    setRelay(false);
    pwmOff();
    digitalWrite(STATUS_LED_PIN, LED_OFF);

    delay(10);
    return;
  }

  // ----------------------------------------------------------
  // Wi-Fi web server.
  // ----------------------------------------------------------

  server.handleClient();

  // ----------------------------------------------------------
  // BLE status notifications.
  // ----------------------------------------------------------

  if (BLE_ENABLED &&
      bleClientConnected &&
      (millis() - lastBleStatus >= BLE_STATUS_INTERVAL_MS)) {

    lastBleStatus = millis();
    bleNotifyStatus();
  }

  // ----------------------------------------------------------
  // Optional communication failsafe.
  // ----------------------------------------------------------

  if (FAILSAFE_ENABLED &&
      currentMode != MODE_AUTO &&
      (millis() - lastHeartbeat > FAILSAFE_TIMEOUT_MS)) {

    Serial.println("FAILSAFE: No heartbeat -> AUTO");

    setMode(MODE_AUTO);
    lastHeartbeat = millis();

    bleNotifyStatus();
  }

  delay(1);
}