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
      --primary: #00d4ff;
      --secondary: #7b2cbf;
      --success: #2ecc71;
      --danger: #e74c3c;
      --warning: #f39c12;
      --dark: #0d1117;
      --card-bg: rgba(30, 35, 45, 0.85);
      --glass: rgba(255, 255, 255, 0.05);
      --border: rgba(255, 255, 255, 0.1);
      --text: #e6edf3;
      --text-muted: #8b949e;
    }

    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }

    body {
      font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
      background: linear-gradient(135deg, #0d1117 0%, #161b22 50%, #1a1f2e 100%);
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
      padding: 20px;
      background: var(--card-bg);
      backdrop-filter: blur(20px);
      border-radius: 20px;
      border: 1px solid var(--border);
    }

    header h1 {
      font-size: clamp(1.5rem, 4vw, 2.5rem);
      background: linear-gradient(135deg, var(--primary), var(--secondary));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
      margin-bottom: 5px;
    }

    header .subtitle {
      color: var(--text-muted);
      font-size: 0.9rem;
    }

    .dashboard {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 20px;
    }

    .card {
      background: var(--card-bg);
      backdrop-filter: blur(20px);
      border-radius: 16px;
      border: 1px solid var(--border);
      padding: 24px;
      transition: transform 0.3s ease, box-shadow 0.3s ease;
    }

    .card:hover {
      transform: translateY(-4px);
      box-shadow: 0 20px 40px rgba(0, 0, 0, 0.3);
    }

    .card-header {
      display: flex;
      align-items: center;
      gap: 12px;
      margin-bottom: 20px;
      padding-bottom: 15px;
      border-bottom: 1px solid var(--border);
    }

    .card-header .icon {
      width: 40px;
      height: 40px;
      border-radius: 10px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.3rem;
    }

    .card-header h3 {
      font-size: 1.1rem;
      font-weight: 600;
    }

    .icon-blue { background: linear-gradient(135deg, #00d4ff33, #00d4ff11); color: var(--primary); }
    .icon-green { background: linear-gradient(135deg, #2ecc7133, #2ecc7111); color: var(--success); }
    .icon-orange { background: linear-gradient(135deg, #f39c1233, #f39c1211); color: var(--warning); }
    .icon-purple { background: linear-gradient(135deg, #7b2cbf33, #7b2cbf11); color: var(--secondary); }
    .icon-red { background: linear-gradient(135deg, #e74c3c33, #e74c3c11); color: var(--danger); }

    /* Attitude Indicator */
    .attitude-container {
      display: flex;
      justify-content: center;
      gap: 40px;
      flex-wrap: wrap;
    }

    .attitude-box {
      text-align: center;
    }

    .attitude-value {
      font-size: 2.5rem;
      font-weight: 700;
      font-family: 'Consolas', monospace;
      background: linear-gradient(135deg, var(--primary), var(--secondary));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
    }

    .attitude-label {
      color: var(--text-muted);
      font-size: 0.85rem;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-top: 5px;
    }

    /* Stats Row */
    .stat-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 12px 0;
      border-bottom: 1px solid var(--border);
    }

    .stat-row:last-child {
      border-bottom: none;
    }

    .stat-label {
      color: var(--text-muted);
      font-size: 0.9rem;
    }

    .stat-value {
      font-family: 'Consolas', monospace;
      font-weight: 600;
      font-size: 1rem;
    }

    .stat-value.ok { color: var(--success); }
    .stat-value.warning { color: var(--warning); }
    .stat-value.danger { color: var(--danger); }

    /* Radio Sliders */
    .radio-channel {
      margin-bottom: 16px;
    }

    .radio-header {
      display: flex;
      justify-content: space-between;
      margin-bottom: 8px;
    }

    .radio-label {
      color: var(--text-muted);
      font-size: 0.85rem;
      text-transform: uppercase;
      letter-spacing: 1px;
    }

    .radio-value {
      font-family: 'Consolas', monospace;
      color: var(--primary);
      font-weight: 600;
    }

    input[type="range"] {
      -webkit-appearance: none;
      width: 100%;
      height: 8px;
      background: linear-gradient(90deg, var(--secondary), var(--primary));
      border-radius: 10px;
      opacity: 0.7;
    }

    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: white;
      box-shadow: 0 2px 10px rgba(0, 0, 0, 0.3);
      cursor: pointer;
    }

    /* PID Inputs */
    .pid-section {
      margin-bottom: 20px;
    }

    .pid-title {
      font-size: 0.8rem;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 12px;
      padding-bottom: 8px;
      border-bottom: 1px solid var(--border);
    }

    .pid-row {
      display: flex;
      align-items: center;
      gap: 12px;
      margin-bottom: 10px;
    }

    .pid-row label {
      width: 60px;
      font-size: 0.9rem;
      color: var(--text-muted);
    }

    .pid-row input {
      flex: 1;
      padding: 10px 14px;
      border: 1px solid var(--border);
      border-radius: 8px;
      background: var(--glass);
      color: var(--text);
      font-family: 'Consolas', monospace;
      font-size: 1rem;
      transition: border-color 0.3s, box-shadow 0.3s;
    }

    .pid-row input:focus {
      outline: none;
      border-color: var(--primary);
      box-shadow: 0 0 0 3px rgba(0, 212, 255, 0.15);
    }

    /* Buttons */
    .btn {
      padding: 12px 24px;
      border: none;
      border-radius: 10px;
      font-weight: 600;
      font-size: 0.95rem;
      cursor: pointer;
      transition: all 0.3s ease;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }

    .btn-primary {
      background: linear-gradient(135deg, var(--primary), var(--secondary));
      color: white;
      width: 100%;
    }

    .btn-primary:hover {
      transform: translateY(-2px);
      box-shadow: 0 10px 30px rgba(0, 212, 255, 0.3);
    }

    .btn-danger {
      background: linear-gradient(135deg, var(--danger), #c0392b);
      color: white;
      width: 100%;
    }

    .btn-danger:hover {
      transform: translateY(-2px);
      box-shadow: 0 10px 30px rgba(231, 76, 60, 0.3);
    }

    /* Motor Grid */
    .motor-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
      margin-bottom: 20px;
    }

    .btn-motor {
      padding: 20px;
      border: 2px solid var(--border);
      border-radius: 12px;
      background: var(--glass);
      color: var(--text);
      font-weight: 700;
      font-size: 1.1rem;
      cursor: pointer;
      transition: all 0.3s ease;
    }

    .btn-motor:hover {
      border-color: var(--primary);
      background: rgba(0, 212, 255, 0.1);
    }

    .btn-motor.active {
      border-color: var(--primary);
      background: linear-gradient(135deg, rgba(0, 212, 255, 0.2), rgba(123, 44, 191, 0.2));
      box-shadow: 0 0 20px rgba(0, 212, 255, 0.2);
    }

    .motor-slider-container {
      margin: 20px 0;
    }

    .slider-value {
      text-align: center;
      font-family: 'Consolas', monospace;
      font-size: 1.5rem;
      color: var(--primary);
      margin-top: 10px;
    }

    /* Loop Time Indicator */
    .loop-indicator {
      display: flex;
      align-items: center;
      gap: 15px;
      padding: 15px;
      background: var(--glass);
      border-radius: 12px;
      margin-top: 15px;
    }

    .loop-bar {
      flex: 1;
      height: 8px;
      background: var(--border);
      border-radius: 10px;
      overflow: hidden;
    }

    .loop-fill {
      height: 100%;
      border-radius: 10px;
      transition: width 0.3s, background 0.3s;
    }

    /* Responsive */
    @media (max-width: 768px) {
      body { padding: 10px; }
      .dashboard { gap: 15px; }
      .card { padding: 18px; }
      .attitude-value { font-size: 2rem; }
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
      <h1>🚁 Drone Dashboard</h1>
      <p class="subtitle">ESP32 Flight Controller — Real-time Telemetry</p>
    </header>

    <div class="dashboard">
      <!-- Attitude Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon icon-blue">📐</div>
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
        </div>
        <div class="loop-indicator">
          <span class="stat-label">Loop</span>
          <div class="loop-bar">
            <div class="loop-fill" id="loop_bar"></div>
          </div>
          <span class="stat-value" id="lt">0</span><span style="color:var(--text-muted)">µs</span>
        </div>
      </div>

      <!-- Diagnostics Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon icon-orange">⚡</div>
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
        <button class="btn btn-primary" style="margin-top:15px" onclick="fetch('/reset_max')">Reset Counters</button>
      </div>

      <!-- Radio Monitor Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon icon-purple">📡</div>
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
          <div class="icon icon-green">🎚️</div>
          <h3>PID Tuning</h3>
        </div>
        <div class="pid-section">
          <div class="pid-title">Pitch / Roll</div>
          <div class="pid-row"><label>P</label><input type="number" step="0.1" id="ppr"></div>
          <div class="pid-row"><label>I</label><input type="number" step="0.01" id="ipr"></div>
          <div class="pid-row"><label>D</label><input type="number" step="1.0" id="dpr"></div>
        </div>
        <div class="pid-section">
          <div class="pid-title">Yaw</div>
          <div class="pid-row"><label>P</label><input type="number" step="0.1" id="py"></div>
          <div class="pid-row"><label>I</label><input type="number" step="0.01" id="iy"></div>
          <div class="pid-row"><label>D</label><input type="number" step="1.0" id="dy"></div>
        </div>
        <div class="pid-section">
          <div class="pid-title">Auto Level</div>
          <div class="pid-row"><label>Level P</label><input type="number" step="1.0" id="pl"></div>
        </div>
        <button class="btn btn-primary" onclick="sendPID()">Update PID</button>
      </div>

      <!-- Motor Test Card -->
      <div class="card">
        <div class="card-header">
          <div class="icon icon-red">🔧</div>
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
        <button class="btn btn-danger" onclick="stopAll()">EMERGENCY STOP</button>
      </div>
    </div>
  </div>

<script>
function loadPID() {
  fetch('/get_pid').then(res => res.json()).then(data => {
    document.getElementById('ppr').value = data.ppr;
    document.getElementById('ipr').value = data.ipr;
    document.getElementById('dpr').value = data.dpr;
    document.getElementById('py').value = data.py;
    document.getElementById('iy').value = data.iy;
    document.getElementById('dy').value = data.dy;
    document.getElementById('pl').value = data.pl;
  });
}

function sendPID() {
  let params = ['ppr','ipr','dpr','py','iy','dy','pl'].map(id => `${id}=${document.getElementById(id).value}`).join('&');
  fetch(`/set_pid?${params}`).then(() => {
    let btn = event.target;
    btn.textContent = '✓ Updated!';
    setTimeout(() => btn.textContent = 'Update PID', 1500);
  });
}

setInterval(() => {
  fetch('/data').then(res => res.json()).then(data => {
    document.getElementById("ar").innerText = data.ar.toFixed(1);
    document.getElementById("ap").innerText = data.ap.toFixed(1);

    // Loop time with color + bar
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

let activeMotor = 0;
function test(m) {
  activeMotor = m;
  document.querySelectorAll('.btn-motor').forEach(b => b.classList.remove('active'));
  document.getElementById('btn' + m).classList.add('active');
  document.getElementById('slider').value = 1000;
  updateVal(1000);
}

function updateVal(val) {
  document.getElementById('sliderVal').innerText = val;
  if (activeMotor > 0) fetch(`/motor?m=${activeMotor}&val=${val}`);
}

function stopAll() {
  activeMotor = 0;
  document.querySelectorAll('.btn-motor').forEach(b => b.classList.remove('active'));
  document.getElementById('slider').value = 1000;
  document.getElementById('sliderVal').innerText = 1000;
  fetch('/stop');
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
        json += "\"r1\":" + String(drone_data->channel_1) + ",";
        json += "\"r2\":" + String(drone_data->channel_2) + ",";
        json += "\"r3\":" + String(drone_data->channel_3) + ",";
        json += "\"r4\":" + String(drone_data->channel_4) + ",";
        json += "\"lt\":" + String(drone_data->loop_time) + ",";
        json += "\"mr\":" + String(drone_data->max_time_radio) + ",";
        json += "\"mi\":" + String(drone_data->max_time_imu) + ",";
        json += "\"ci\":" + String(drone_data->current_time_imu) + ",";
        json += "\"mp\":" + String(drone_data->max_time_pid);
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