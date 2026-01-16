#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "telemetry.h"
#include "radio.h"
#include "motors.h"
#include "types.h"

const char* ssid = "Drone_ESP32";
const char* password = "password123";

DroneState* drone_data;
AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="fr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Drone Dashboard</title>
  <style>
    :root {
      --primary: #3b82f6;
      --primary-hover: #2563eb;
      --success: #22c55e;
      --danger: #ef4444;
      --danger-hover: #dc2626;
      --warning: #f59e0b;
      --dark: #0f172a;
      --card-bg: #1e293b;
      --card-border: #334155;
      --text: #f1f5f9;
      --text-muted: #94a3b8;
      --input-bg: #0f172a;
    }

    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }

    body {
      font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
      background: var(--dark);
      min-height: 100vh;
      color: var(--text);
      padding: 20px;
    }

    .container {
      max-width: 1400px;
      margin: 0 auto;
    }

    header {
      text-align: center;
      margin-bottom: 30px;
      padding: 24px;
      background: var(--card-bg);
      border-radius: 12px;
      border: 1px solid var(--card-border);
    }

    header h1 {
      font-size: clamp(1.5rem, 4vw, 2rem);
      color: var(--text);
      margin-bottom: 4px;
      font-weight: 600;
    }

    header .subtitle {
      color: var(--text-muted);
      font-size: 0.875rem;
    }

    .status-dot {
      display: inline-block;
      width: 8px;
      height: 8px;
      background: var(--success);
      border-radius: 50%;
      margin-right: 8px;
      animation: pulse 2s infinite;
    }

    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }

    .dashboard {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 20px;
    }

    .card {
      background: var(--card-bg);
      border-radius: 12px;
      border: 1px solid var(--card-border);
      padding: 24px;
      transition: border-color 0.2s ease;
    }

    .card:hover {
      border-color: #475569;
    }

    .card-header {
      display: flex;
      align-items: center;
      gap: 12px;
      margin-bottom: 20px;
      padding-bottom: 16px;
      border-bottom: 1px solid var(--card-border);
    }

    .card-header .icon {
      width: 36px;
      height: 36px;
      border-radius: 8px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.1rem;
      background: var(--input-bg);
    }

    .card-header h3 {
      font-size: 1rem;
      font-weight: 600;
      color: var(--text);
    }

    /* Attitude Indicator */
    .attitude-container {
      display: flex;
      justify-content: center;
      gap: 48px;
      flex-wrap: wrap;
    }

    .attitude-box {
      text-align: center;
    }

    .attitude-value {
      font-size: 2.5rem;
      font-weight: 700;
      font-family: 'Consolas', 'Monaco', monospace;
      color: var(--text);
    }

    .attitude-label {
      color: var(--text-muted);
      font-size: 0.75rem;
      text-transform: uppercase;
      letter-spacing: 1.5px;
      margin-top: 4px;
    }

    /* Stats Row */
    .stat-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 10px 0;
      border-bottom: 1px solid var(--card-border);
    }

    .stat-row:last-child {
      border-bottom: none;
    }

    .stat-label {
      color: var(--text-muted);
      font-size: 0.875rem;
    }

    .stat-value {
      font-family: 'Consolas', monospace;
      font-weight: 600;
      font-size: 0.875rem;
      color: var(--text);
    }

    .stat-value.ok { color: var(--success); }
    .stat-value.warning { color: var(--warning); }
    .stat-value.danger { color: var(--danger); }

    /* Radio Sliders */
    .radio-channel {
      margin-bottom: 14px;
    }

    .radio-header {
      display: flex;
      justify-content: space-between;
      margin-bottom: 6px;
    }

    .radio-label {
      color: var(--text-muted);
      font-size: 0.75rem;
      text-transform: uppercase;
      letter-spacing: 1px;
    }

    .radio-value {
      font-family: 'Consolas', monospace;
      color: var(--primary);
      font-weight: 600;
      font-size: 0.875rem;
    }

    input[type="range"] {
      -webkit-appearance: none;
      width: 100%;
      height: 6px;
      background: var(--card-border);
      border-radius: 3px;
    }

    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 16px;
      height: 16px;
      border-radius: 50%;
      background: var(--primary);
      cursor: pointer;
    }

    /* PID Inputs */
    .pid-section {
      margin-bottom: 16px;
    }

    .pid-title {
      font-size: 0.7rem;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 1.5px;
      margin-bottom: 10px;
    }

    .pid-row {
      display: flex;
      align-items: center;
      gap: 12px;
      margin-bottom: 8px;
    }

    .pid-row label {
      width: 50px;
      font-size: 0.875rem;
      color: var(--text-muted);
    }

    .pid-row input {
      flex: 1;
      padding: 10px 12px;
      border: 1px solid var(--card-border);
      border-radius: 6px;
      background: var(--input-bg);
      color: var(--text);
      font-family: 'Consolas', monospace;
      font-size: 0.875rem;
      transition: border-color 0.2s;
    }

    .pid-row input:focus {
      outline: none;
      border-color: var(--primary);
    }

    /* Buttons */
    .btn {
      padding: 12px 20px;
      border: none;
      border-radius: 8px;
      font-weight: 600;
      font-size: 0.875rem;
      cursor: pointer;
      transition: all 0.2s ease;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      width: 100%;
      position: relative;
      overflow: hidden;
    }

    .btn-primary {
      background: var(--primary);
      color: white;
    }

    .btn-primary:hover {
      background: var(--primary-hover);
    }

    .btn-primary.success {
      background: var(--success) !important;
    }

    .btn-danger {
      background: var(--danger);
      color: white;
    }

    .btn-danger:hover {
      background: var(--danger-hover);
    }

    .btn-danger.triggered {
      background: #fbbf24 !important;
      color: #000 !important;
      animation: flash 0.3s ease 2;
    }

    @keyframes flash {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.6; }
    }

    .btn:disabled {
      opacity: 0.6;
      cursor: not-allowed;
    }

    /* Motor Grid */
    .motor-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 10px;
      margin-bottom: 16px;
    }

    .btn-motor {
      padding: 16px;
      border: 2px solid var(--card-border);
      border-radius: 8px;
      background: var(--input-bg);
      color: var(--text);
      font-weight: 700;
      font-size: 1rem;
      cursor: pointer;
      transition: all 0.2s ease;
    }

    .btn-motor:hover {
      border-color: var(--primary);
    }

    .btn-motor.active {
      border-color: var(--primary);
      background: rgba(59, 130, 246, 0.15);
    }

    .motor-slider-container {
      margin: 16px 0;
    }

    .slider-value {
      text-align: center;
      font-family: 'Consolas', monospace;
      font-size: 1.25rem;
      color: var(--primary);
      margin-top: 8px;
    }

    /* Loop Time Indicator */
    .loop-indicator {
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 12px;
      background: var(--input-bg);
      border-radius: 8px;
      margin-top: 16px;
    }

    .loop-bar {
      flex: 1;
      height: 6px;
      background: var(--card-border);
      border-radius: 3px;
      overflow: hidden;
    }

    .loop-fill {
      height: 100%;
      border-radius: 3px;
      transition: width 0.3s, background 0.3s;
    }

    /* Toast Notification */
    .toast {
      position: fixed;
      bottom: 30px;
      left: 50%;
      transform: translateX(-50%) translateY(100px);
      padding: 14px 28px;
      border-radius: 8px;
      font-weight: 600;
      font-size: 0.875rem;
      opacity: 0;
      transition: all 0.3s ease;
      z-index: 1000;
    }

    .toast.show {
      transform: translateX(-50%) translateY(0);
      opacity: 1;
    }

    .toast.success {
      background: var(--success);
      color: white;
    }

    .toast.danger {
      background: var(--danger);
      color: white;
    }

    .toast.warning {
      background: var(--warning);
      color: #000;
    }

    /* Responsive */
    @media (max-width: 768px) {
      body { padding: 12px; }
      .dashboard { gap: 12px; }
      .card { padding: 18px; }
      .attitude-value { font-size: 2rem; }
      .attitude-container { gap: 32px; }
    }

    @media (min-width: 1200px) {
      .dashboard {
        grid-template-columns: repeat(3, 1fr);
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1><span class="status-dot"></span>Drone Dashboard</h1>
      <p class="subtitle">ESP32 Flight Controller — Real-time Telemetry</p>
    </header>

    <div class="dashboard">
      <!-- Attitude Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon">📐</div>
          <h3>Attitude</h3>
        </div>
        <div class="attitude-container">
          <div class="attitude-box">
            <div class="attitude-value" id="ar">0.0</div>
            <div class="attitude-label">Roll °</div>
          </div>
          <div class="attitude-box">
            <div class="attitude-value" id="ap">0.0</div>
            <div class="attitude-label">Pitch °</div>
          </div>
          <div class="attitude-box">
            <div class="attitude-value" id="ay">0.0</div>
            <div class="attitude-label">Yaw °</div>
          </div>
        </div>
        <div class="loop-indicator">
          <span class="stat-label">Loop</span>
          <div class="loop-bar">
            <div class="loop-fill" id="loop_bar"></div>
          </div>
          <span class="stat-value" id="lt">0</span><span style="color:var(--text-muted);font-size:0.75rem;margin-left:2px">µs</span>
        </div>
      </div>

      <!-- Diagnostics Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon">⚡</div>
          <h3>Diagnostics</h3>
        </div>
        <div class="stat-row">
          <span class="stat-label">Radio (RX)</span>
          <span class="stat-value" id="max_rad">0</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">IMU Current</span>
          <span class="stat-value ok" id="cur_imu">0</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">IMU Max</span>
          <span class="stat-value" id="max_imu">0</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">PID / Motors</span>
          <span class="stat-value" id="max_pid">0</span>
        </div>
        <button class="btn btn-primary" style="margin-top:16px" onclick="resetCounters(this)">Reset Counters</button>
      </div>

      <!-- Radio Monitor Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon">📡</div>
          <h3>Radio Monitor</h3>
        </div>
        <div class="radio-channel">
          <div class="radio-header">
            <span class="radio-label">Throttle</span>
            <span class="radio-value" id="val_t">1000</span>
          </div>
          <input type="range" min="1000" max="2000" id="rx_t" disabled>
        </div>
        <div class="radio-channel">
          <div class="radio-header">
            <span class="radio-label">Yaw</span>
            <span class="radio-value" id="val_y">1500</span>
          </div>
          <input type="range" min="1000" max="2000" id="rx_y" disabled>
        </div>
        <div class="radio-channel">
          <div class="radio-header">
            <span class="radio-label">Pitch</span>
            <span class="radio-value" id="val_p">1500</span>
          </div>
          <input type="range" min="1000" max="2000" id="rx_p" disabled>
        </div>
        <div class="radio-channel">
          <div class="radio-header">
            <span class="radio-label">Roll</span>
            <span class="radio-value" id="val_r">1500</span>
          </div>
          <input type="range" min="1000" max="2000" id="rx_r" disabled>
        </div>
      </div>

      <!-- PID Tuning Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon">🎚️</div>
          <h3>PID Tuning</h3>
        </div>
        <div class="pid-section">
          <div class="pid-title">Pitch / Roll</div>
          <div class="pid-row"><label>P</label><input type="number" step="0.1" id="ppr"></div>
          <div class="pid-row"><label>I</label><input type="number" step="0.01" id="ipr"></div>
          <div class="pid-row"><label>D</label><input type="number" step="1.0" id="dpr"></div>
          <div class="pid-row"><label>FF</label><input type="number" step="0.01" id="ffpr"></div>

        </div>
        <div class="pid-section">
          <div class="pid-title">Yaw</div>
          <div class="pid-row"><label>P</label><input type="number" step="0.1" id="py"></div>
          <div class="pid-row"><label>I</label><input type="number" step="0.01" id="iy"></div>
          <div class="pid-row"><label>D</label><input type="number" step="1.0" id="dy"></div>
          <div class="pid-row"><label>FF</label><input type="number" step="0.01" id="ffy"></div>

        </div>
        <div class="pid-section">
          <div class="pid-title">Auto Level</div>
          <div class="pid-row"><label>Level P</label><input type="number" step="1.0" id="pl"></div>
        </div>
        <button class="btn btn-primary" id="pidBtn" onclick="sendPID()">Update PID</button>
      </div>

      <!-- Motor Test Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon">🔧</div>
          <h3>Motor Test</h3>
        </div>
        <div class="motor-grid">
          <button id="btn1" class="btn-motor" onclick="test(1)">M1</button>
          <button id="btn2" class="btn-motor" onclick="test(2)">M2</button>
          <button id="btn3" class="btn-motor" onclick="test(3)">M3</button>
          <button id="btn4" class="btn-motor" onclick="test(4)">M4</button>
        </div>
        <div class="motor-slider-container">
          <input type="range" min="1000" max="1300" value="1000" id="slider" oninput="updateVal(this.value)">
          <div class="slider-value" id="sliderVal">1000</div>
        </div>
        <button class="btn btn-danger" id="stopBtn" onclick="stopAll()">Emergency Stop</button>
      </div>
    </div>
  </div>

  <!-- Toast -->
  <div class="toast" id="toast"></div>

<script>
// Toast notification
function showToast(message, type = 'success') {
  const toast = document.getElementById('toast');
  toast.textContent = message;
  toast.className = 'toast ' + type + ' show';
  setTimeout(() => { toast.classList.remove('show'); }, 2500);
}

// Load PID
function loadPID() {
  fetch('/get_pid').then(res => res.json()).then(data => {
    document.getElementById('ppr').value = data.ppr;
    document.getElementById('ipr').value = data.ipr;
    document.getElementById('dpr').value = data.dpr;
    document.getElementById('py').value = data.py;
    document.getElementById('iy').value = data.iy;
    document.getElementById('dy').value = data.dy;
    document.getElementById('pl').value = data.pl;
    document.getElementById('ffpr').value = data.ffpr;
    document.getElementById('ffy').value = data.ffy;

  });
}

// Send PID with feedback
function sendPID() {
  const btn = document.getElementById('pidBtn');
  const originalText = btn.textContent;
  btn.disabled = true;
  btn.textContent = 'Updating...';
  
  let params = ['ppr','ipr','dpr','py','iy','dy','pl','ffpr','ffy'].map(id => `${id}=${document.getElementById(id).value}`).join('&');
  
  fetch(`/set_pid?${params}`)
    .then(res => {
      if (res.ok) {
        btn.textContent = '✓ Updated';
        btn.classList.add('success');
        showToast('PID parameters updated successfully', 'success');
      } else {
        throw new Error('Failed');
      }
    })
    .catch(() => {
      showToast('Failed to update PID', 'danger');
    })
    .finally(() => {
      setTimeout(() => {
        btn.textContent = originalText;
        btn.classList.remove('success');
        btn.disabled = false;
      }, 1500);
    });
}

// Reset counters with feedback
function resetCounters(btn) {
  const originalText = btn.textContent;
  btn.disabled = true;
  btn.textContent = 'Resetting...';
  
  fetch('/reset_max')
    .then(res => {
      if (res.ok) {
        btn.textContent = '✓ Reset';
        btn.classList.add('success');
        showToast('Counters reset', 'success');
      }
    })
    .finally(() => {
      setTimeout(() => {
        btn.textContent = originalText;
        btn.classList.remove('success');
        btn.disabled = false;
      }, 1200);
    });
}

// Data polling
setInterval(() => {
  fetch('/data').then(res => res.json()).then(data => {
    document.getElementById("ar").innerText = data.ar.toFixed(1);
    document.getElementById("ap").innerText = data.ap.toFixed(1);
    document.getElementById("ay").innerText = data.ay.toFixed(1);  // AJOUT

    // Loop time
    let lt = data.lt;
    let ltEl = document.getElementById("lt");
    let barEl = document.getElementById("loop_bar");
    ltEl.innerText = lt;
    let pct = Math.min((lt / 4000) * 100, 100);
    barEl.style.width = pct + "%";
    if (lt < 3000) { ltEl.className = "stat-value ok"; barEl.style.background = "var(--success)"; }
    else if (lt < 4000) { ltEl.className = "stat-value warning"; barEl.style.background = "var(--warning)"; }
    else { ltEl.className = "stat-value danger"; barEl.style.background = "var(--danger)"; }

    // Diagnostics
    document.getElementById("max_rad").innerText = data.mr;
    document.getElementById("cur_imu").innerText = data.ci;
    document.getElementById("max_imu").innerText = data.mi;
    document.getElementById("max_pid").innerText = data.mp;

    // Radio
    document.getElementById("rx_t").value = data.r3; document.getElementById("val_t").innerText = data.r3;
    document.getElementById("rx_y").value = data.r4; document.getElementById("val_y").innerText = data.r4;
    document.getElementById("rx_p").value = data.r2; document.getElementById("val_p").innerText = data.r2;
    document.getElementById("rx_r").value = data.r1; document.getElementById("val_r").innerText = data.r1;
  });
}, 200);

// Motor test
let activeMotor = 0;
function test(m) {
  activeMotor = m;
  document.querySelectorAll('.btn-motor').forEach(b => b.classList.remove('active'));
  document.getElementById('btn' + m).classList.add('active');
  document.getElementById('slider').value = 1000;
  updateVal(1000);
  showToast('Motor ' + m + ' selected', 'warning');
}

function updateVal(val) {
  document.getElementById('sliderVal').innerText = val;
  if (activeMotor > 0) fetch(`/motor?m=${activeMotor}&val=${val}`);
}

// Emergency stop with feedback
function stopAll() {
  const btn = document.getElementById('stopBtn');
  btn.classList.add('triggered');
  
  activeMotor = 0;
  document.querySelectorAll('.btn-motor').forEach(b => b.classList.remove('active'));
  document.getElementById('slider').value = 1000;
  document.getElementById('sliderVal').innerText = 1000;
  
  fetch('/stop')
    .then(res => {
      if (res.ok) {
        showToast('⚠ EMERGENCY STOP ACTIVATED', 'danger');
      }
    })
    .finally(() => {
      setTimeout(() => { btn.classList.remove('triggered'); }, 600);
    });
}

window.onload = loadPID;
</script>
</body>
</html>
)rawliteral";

void telemetryTask(void * parameter) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"ar\":" + String(drone_data->angle_roll) + ",";
        json += "\"ap\":" + String(drone_data->angle_pitch) + ",";
        json += "\"ay\":" + String(drone_data->angle_yaw) + ",";  // AJOUT
        json += "\"r1\":" + String(drone_data->channel_1) + ",";
        json += "\"r2\":" + String(drone_data->channel_2) + ",";
        json += "\"r3\":" + String(drone_data->channel_3) + ",";
        json += "\"r4\":" + String(drone_data->channel_4) + ",";
        json += "\"lt\":" + String(drone_data->loop_time) + ",";
        json += "\"mr\":" + String(drone_data->max_time_radio) + ",";
        json += "\"mi\":" + String(drone_data->max_time_imu) + ",";
        json += "\"ci\":" + String(drone_data->current_time_imu) + ",";
        json += "\"mp\":" + String(drone_data->max_time_pid) + ",";
        json += "\"gy\":" + String(drone_data->gyro_yaw_input) + ",";     // AJOUT: gyro yaw
        json += "\"poy\":" + String(drone_data->pid_output_yaw);          // AJOUT: sortie PID yaw
        json += "}";
        request->send(200, "application/json", json);
    });
    
    server.on("/reset_max", HTTP_GET, [](AsyncWebServerRequest *request){
        drone_data->max_time_radio = 0;
        drone_data->max_time_imu = 0;
        drone_data->max_time_pid = 0;
        request->send(200, "text/plain", "OK");
    });

    server.on("/get_pid", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"ppr\":" + String(drone_data->p_pitch_roll) + ",";
        json += "\"ipr\":" + String(drone_data->i_pitch_roll) + ",";
        json += "\"dpr\":" + String(drone_data->d_pitch_roll) + ",";
        json += "\"py\":" + String(drone_data->p_yaw) + ",";
        json += "\"iy\":" + String(drone_data->i_yaw) + ",";
        json += "\"dy\":" + String(drone_data->d_yaw) + ",";
        json += "\"ffpr\":" + String(drone_data->ff_pitch_roll) + ",";
        json += "\"ffy\":" + String(drone_data->ff_yaw) + ",";
        json += "\"pl\":" + String(drone_data->p_level);
        json += "}";
        request->send(200, "application/json", json);
    });

    server.on("/set_pid", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasParam("ppr")) drone_data->p_pitch_roll = request->getParam("ppr")->value().toFloat();
        if(request->hasParam("ipr")) drone_data->i_pitch_roll = request->getParam("ipr")->value().toFloat();
        if(request->hasParam("dpr")) drone_data->d_pitch_roll = request->getParam("dpr")->value().toFloat();
        if(request->hasParam("py")) drone_data->p_yaw = request->getParam("py")->value().toFloat();
        if(request->hasParam("iy")) drone_data->i_yaw = request->getParam("iy")->value().toFloat();
        if(request->hasParam("dy")) drone_data->d_yaw = request->getParam("dy")->value().toFloat();
        if(request->hasParam("pl")) drone_data->p_level = request->getParam("pl")->value().toFloat();
        if(request->hasParam("ffpr")) drone_data->ff_pitch_roll = request->getParam("ffpr")->value().toFloat();
        if(request->hasParam("ffy")) drone_data->ff_yaw = request->getParam("ffy")->value().toFloat();

        request->send(200, "text/plain", "OK");
    });

    server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasParam("m") && request->hasParam("val")) {
            int m = request->getParam("m")->value().toInt();
            int val = request->getParam("val")->value().toInt();
            if(drone_data->current_mode == MODE_SAFE || drone_data->current_mode == MODE_WEB_TEST) {
                drone_data->current_mode = MODE_WEB_TEST;
                for(int i=1; i<=4; i++) if(i!=m) drone_data->web_test_vals[i] = 1000;
                if(m>=1 && m<=4) drone_data->web_test_vals[m] = val;
            }
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
        drone_data->current_mode = MODE_SAFE;
        motors_stop();
        for(int i=1; i<=4; i++) drone_data->web_test_vals[i] = 1000;
        request->send(200, "text/plain", "STOPPED");
    });

    server.begin();
    vTaskDelete(NULL);
}

void start_telemetry_task(DroneState* drone_ptr) {
    drone_data = drone_ptr;
    xTaskCreatePinnedToCore(telemetryTask, "WifiTask", 4096, NULL, 1, NULL, 0);
}