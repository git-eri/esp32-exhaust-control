#pragma once

#include <Arduino.h>

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
