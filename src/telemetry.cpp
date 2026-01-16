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

    /* 3D Cube CSS */
    .cube-scene {
      width: 150px;
      height: 150px;
      margin: 0 auto;
      perspective: 400px;
    }
    .cube {
      width: 100%;
      height: 100%;
      position: relative;
      transform-style: preserve-3d;
      transition: transform 0.1s ease-out;
    }
    .cube-face {
      position: absolute;
      width: 150px;
      height: 150px;
      border: 2px solid var(--primary);
      display: flex;
      align-items: center;
      justify-content: center;
      font-weight: bold;
      font-size: 0.7rem;
      color: var(--text);
      backface-visibility: visible;
    }
    .cube-face.front  { background: rgba(59, 130, 246, 0.3); transform: rotateY(0deg) translateZ(75px); }
    .cube-face.back   { background: rgba(59, 130, 246, 0.2); transform: rotateY(180deg) translateZ(75px); }
    .cube-face.right  { background: rgba(34, 197, 94, 0.3); transform: rotateY(90deg) translateZ(75px); }
    .cube-face.left   { background: rgba(34, 197, 94, 0.2); transform: rotateY(-90deg) translateZ(75px); }
    .cube-face.top    { background: rgba(239, 68, 68, 0.3); transform: rotateX(90deg) translateZ(75px); }
    .cube-face.bottom { background: rgba(239, 68, 68, 0.2); transform: rotateX(-90deg) translateZ(75px); }

    /* Drift Map */
    .drift-map-container {
      background: var(--input-bg);
      border-radius: 8px;
      padding: 12px;
      border: 1px solid var(--card-border);
    }
    .drift-map-container canvas {
      width: 100%;
      height: auto;
      display: block;
    }
    .drift-map-legend {
      display: flex;
      justify-content: center;
      gap: 20px;
      margin-top: 8px;
      font-size: 0.75rem;
    }

    /* Accel Graph */
    #accelGraph {
      width: 100%;
      height: auto;
      background: var(--input-bg);
      border-radius: 8px;
      border: 1px solid var(--card-border);
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
      <!-- Inertial Drift Proof Card - FULL WIDTH -->
      <div class="card" style="grid-column: 1 / -1;">
        <div class="card-header">
          <div class="icon">🎯</div>
          <h3>Inertial Drift Proof (PoC) — Why GPS/VIO is Mandatory</h3>
        </div>
        <div style="display: flex; gap: 24px; flex-wrap: wrap;">
          <!-- 3D Cube Visualization -->
          <div style="flex: 1; min-width: 200px;">
            <div style="text-align: center; margin-bottom: 12px;">
              <span class="stat-label">3D Attitude (Live)</span>
            </div>
            <div class="cube-scene">
              <div class="cube" id="cube3d">
                <div class="cube-face front">FRONT</div>
                <div class="cube-face back">BACK</div>
                <div class="cube-face right">RIGHT</div>
                <div class="cube-face left">LEFT</div>
                <div class="cube-face top">TOP</div>
                <div class="cube-face bottom">BOT</div>
              </div>
            </div>
          </div>
          <!-- Integration Simulator -->
          <div style="flex: 1; min-width: 280px;">
            <div style="text-align: center; margin-bottom: 12px;">
              <span class="stat-label">Double Integration Simulator</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Accel X (G)</span>
              <span class="stat-value" id="drift_ax">0.000</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Accel Y (G)</span>
              <span class="stat-value" id="drift_ay">0.000</span>
            </div>
            <div class="stat-row" style="border-top: 2px solid var(--warning);">
              <span class="stat-label">Velocity X (m/s)</span>
              <span class="stat-value warning" id="drift_vx">0.00</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Velocity Y (m/s)</span>
              <span class="stat-value warning" id="drift_vy">0.00</span>
            </div>
            <div class="stat-row" style="border-top: 2px solid var(--danger);">
              <span class="stat-label">Position X (m)</span>
              <span class="stat-value danger" id="drift_px">0.00</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Position Y (m)</span>
              <span class="stat-value danger" id="drift_py">0.00</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Total Drift (m)</span>
              <span class="stat-value danger" id="drift_total" style="font-size: 1.1rem;">0.00</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Elapsed Time</span>
              <span class="stat-value" id="drift_time">0.0 s</span>
            </div>
            <div style="display: flex; gap: 10px; margin-top: 16px;">
              <button class="btn btn-primary" id="driftStartBtn" onclick="toggleDriftIntegration()">Start Integration</button>
              <button class="btn btn-danger" onclick="resetDriftIntegration()">Reset</button>
            </div>
          </div>
          <!-- 3D Drift Map -->
          <div style="flex: 2; min-width: 350px;">
            <div style="text-align: center; margin-bottom: 12px;">
              <span class="stat-label">3D Position Drift Map (XY Plane)</span>
            </div>
            <div class="drift-map-container">
              <canvas id="driftMap3D" width="400" height="300"></canvas>
              <div class="drift-map-legend">
                <span style="color: var(--success);">● Origin</span>
                <span style="color: var(--warning);">● Trail</span>
                <span style="color: var(--danger);">● Current</span>
              </div>
            </div>
            <div style="display: flex; gap: 10px; margin-top: 8px; justify-content: center;">
              <button class="btn btn-primary" style="width: auto; padding: 8px 16px;" onclick="resetDriftMap()">Clear Trail</button>
              <select id="driftMapScale" onchange="updateDriftMapScale()" style="padding: 8px; background: var(--input-bg); color: var(--text); border: 1px solid var(--card-border); border-radius: 6px;">
                <option value="1">Scale: 1m</option>
                <option value="5">Scale: 5m</option>
                <option value="10" selected>Scale: 10m</option>
                <option value="50">Scale: 50m</option>
                <option value="100">Scale: 100m</option>
                <option value="500">Scale: 500m</option>
              </select>
            </div>
          </div>
        </div>
        <p style="color: var(--text-muted); font-size: 0.7rem; margin-top: 16px; text-align: center;">
          ⚠️ Observe how position drifts to infinity even when stationary. This proves GPS/VIO is mandatory for position hold.
        </p>
      </div>

      <!-- Accelerometer Graph Card - FULL WIDTH -->
      <div class="card" style="grid-column: 1 / -1;">
        <div class="card-header">
          <div class="icon">📈</div>
          <h3>Accelerometer Real-Time Graph</h3>
        </div>
        <div style="display: flex; gap: 20px; flex-wrap: wrap;">
          <div style="flex: 3; min-width: 500px;">
            <canvas id="accelGraph" width="700" height="250"></canvas>
          </div>
          <div style="flex: 1; min-width: 150px;">
            <div class="stat-row">
              <span class="stat-label" style="color: #ef4444;">Ax (G)</span>
              <span class="stat-value" id="graph_ax">0.000</span>
            </div>
            <div class="stat-row">
              <span class="stat-label" style="color: #22c55e;">Ay (G)</span>
              <span class="stat-value" id="graph_ay">0.000</span>
            </div>
            <div class="stat-row">
              <span class="stat-label" style="color: #3b82f6;">Az (G)</span>
              <span class="stat-value" id="graph_az">0.000</span>
            </div>
            <div class="stat-row" style="margin-top: 12px;">
              <span class="stat-label">Noise (σ)</span>
              <span class="stat-value warning" id="graph_noise">0.000</span>
            </div>
            <button class="btn btn-primary" style="margin-top: 16px;" onclick="clearAccelGraph()">Clear Graph</button>
          </div>
        </div>
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
    
    // Update Yaw for cube
    lastYaw = data.ay;

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
    
    // Drift PoC - Update with accelerometer data
    if (typeof updateDriftSimulation === 'function') {
      updateDriftSimulation(data.ax, data.ay, data.az, data.ar, data.ap);
    }
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

window.onload = function() {
  loadPID();
  initAccelGraph();
  initDriftMap();
};

// ==================== ACCELEROMETER GRAPH ====================
let accelGraphCtx;
let accelDataX = [], accelDataY = [], accelDataZ = [];
const GRAPH_MAX_POINTS = 200;

function initAccelGraph() {
  const canvas = document.getElementById('accelGraph');
  if (!canvas) return;
  accelGraphCtx = canvas.getContext('2d');
}

function clearAccelGraph() {
  accelDataX = []; accelDataY = []; accelDataZ = [];
}

function updateAccelGraph(ax, ay, az) {
  if (!accelGraphCtx) return;
  
  // Store data
  accelDataX.push(ax); accelDataY.push(ay); accelDataZ.push(az);
  if (accelDataX.length > GRAPH_MAX_POINTS) {
    accelDataX.shift(); accelDataY.shift(); accelDataZ.shift();
  }
  
  // Update display values
  document.getElementById('graph_ax').innerText = ax.toFixed(4);
  document.getElementById('graph_ay').innerText = ay.toFixed(4);
  document.getElementById('graph_az').innerText = az.toFixed(4);
  
  // Calculate noise (standard deviation of last 50 samples)
  if (accelDataX.length > 10) {
    const last50 = accelDataX.slice(-50);
    const mean = last50.reduce((a,b) => a+b, 0) / last50.length;
    const variance = last50.reduce((a,b) => a + (b-mean)*(b-mean), 0) / last50.length;
    document.getElementById('graph_noise').innerText = Math.sqrt(variance).toFixed(4);
  }
  
  // Draw graph
  const canvas = accelGraphCtx.canvas;
  const w = canvas.width, h = canvas.height;
  accelGraphCtx.fillStyle = '#0f172a';
  accelGraphCtx.fillRect(0, 0, w, h);
  
  // Grid
  accelGraphCtx.strokeStyle = '#334155';
  accelGraphCtx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = h * i / 4;
    accelGraphCtx.beginPath();
    accelGraphCtx.moveTo(0, y);
    accelGraphCtx.lineTo(w, y);
    accelGraphCtx.stroke();
  }
  
  // Labels
  accelGraphCtx.fillStyle = '#94a3b8';
  accelGraphCtx.font = '10px Consolas';
  accelGraphCtx.fillText('+2G', 5, 15);
  accelGraphCtx.fillText('+1G', 5, h/4 + 5);
  accelGraphCtx.fillText(' 0G', 5, h/2 + 5);
  accelGraphCtx.fillText('-1G', 5, 3*h/4 + 5);
  accelGraphCtx.fillText('-2G', 5, h - 5);
  
  // Draw lines
  const drawLine = (data, color) => {
    if (data.length < 2) return;
    accelGraphCtx.strokeStyle = color;
    accelGraphCtx.lineWidth = 2;
    accelGraphCtx.beginPath();
    for (let i = 0; i < data.length; i++) {
      const x = (i / GRAPH_MAX_POINTS) * w;
      const y = h/2 - (data[i] / 2) * (h/2); // Scale: -2G to +2G
      if (i === 0) accelGraphCtx.moveTo(x, y);
      else accelGraphCtx.lineTo(x, y);
    }
    accelGraphCtx.stroke();
  };
  
  drawLine(accelDataX, '#ef4444'); // Red for X
  drawLine(accelDataY, '#22c55e'); // Green for Y
  drawLine(accelDataZ, '#3b82f6'); // Blue for Z
}

// ==================== 3D DRIFT MAP ====================
let driftMapCtx;
let driftTrail = [];
let driftMapScale = 10; // meters per half-width
const DRIFT_TRAIL_MAX = 500;

function initDriftMap() {
  const canvas = document.getElementById('driftMap3D');
  if (!canvas) return;
  driftMapCtx = canvas.getContext('2d');
  drawDriftMap();
}

function resetDriftMap() {
  driftTrail = [];
  drawDriftMap();
}

function updateDriftMapScale() {
  driftMapScale = parseFloat(document.getElementById('driftMapScale').value);
  drawDriftMap();
}

function updateDriftMap(px, py) {
  if (!driftMapCtx) return;
  
  driftTrail.push({x: px, y: py, t: Date.now()});
  if (driftTrail.length > DRIFT_TRAIL_MAX) driftTrail.shift();
  
  drawDriftMap();
}

function drawDriftMap() {
  if (!driftMapCtx) return;
  
  const canvas = driftMapCtx.canvas;
  const w = canvas.width, h = canvas.height;
  const cx = w / 2, cy = h / 2;
  
  // Clear with perspective gradient
  const grad = driftMapCtx.createRadialGradient(cx, cy, 0, cx, cy, Math.max(w,h)/2);
  grad.addColorStop(0, '#1e293b');
  grad.addColorStop(1, '#0f172a');
  driftMapCtx.fillStyle = grad;
  driftMapCtx.fillRect(0, 0, w, h);
  
  // Grid with perspective effect
  driftMapCtx.strokeStyle = '#334155';
  driftMapCtx.lineWidth = 1;
  
  // Concentric circles (distance rings)
  for (let r = 1; r <= 4; r++) {
    const radius = (r / 4) * Math.min(cx, cy) * 0.9;
    driftMapCtx.beginPath();
    driftMapCtx.arc(cx, cy, radius, 0, Math.PI * 2);
    driftMapCtx.stroke();
    
    // Distance label
    driftMapCtx.fillStyle = '#64748b';
    driftMapCtx.font = '10px Consolas';
    const dist = (r / 4 * driftMapScale).toFixed(0);
    driftMapCtx.fillText(dist + 'm', cx + radius + 3, cy);
  }
  
  // Cross axes
  driftMapCtx.beginPath();
  driftMapCtx.moveTo(0, cy); driftMapCtx.lineTo(w, cy);
  driftMapCtx.moveTo(cx, 0); driftMapCtx.lineTo(cx, h);
  driftMapCtx.stroke();
  
  // Axis labels
  driftMapCtx.fillStyle = '#94a3b8';
  driftMapCtx.font = '12px Consolas';
  driftMapCtx.fillText('+X', w - 20, cy - 5);
  driftMapCtx.fillText('-X', 5, cy - 5);
  driftMapCtx.fillText('+Y', cx + 5, 15);
  driftMapCtx.fillText('-Y', cx + 5, h - 5);
  
  // Origin marker
  driftMapCtx.fillStyle = '#22c55e';
  driftMapCtx.beginPath();
  driftMapCtx.arc(cx, cy, 6, 0, Math.PI * 2);
  driftMapCtx.fill();
  
  // Convert position to screen coords
  const toScreen = (px, py) => {
    const scale = Math.min(cx, cy) * 0.9 / driftMapScale;
    return {
      x: cx + px * scale,
      y: cy - py * scale // Y inverted
    };
  };
  
  // Draw trail with fade effect
  if (driftTrail.length > 1) {
    for (let i = 1; i < driftTrail.length; i++) {
      const p0 = toScreen(driftTrail[i-1].x, driftTrail[i-1].y);
      const p1 = toScreen(driftTrail[i].x, driftTrail[i].y);
      
      const alpha = 0.3 + 0.7 * (i / driftTrail.length);
      driftMapCtx.strokeStyle = `rgba(251, 191, 36, ${alpha})`;
      driftMapCtx.lineWidth = 2;
      driftMapCtx.beginPath();
      driftMapCtx.moveTo(p0.x, p0.y);
      driftMapCtx.lineTo(p1.x, p1.y);
      driftMapCtx.stroke();
    }
  }
  
  // Current position marker
  if (driftTrail.length > 0) {
    const last = driftTrail[driftTrail.length - 1];
    const p = toScreen(last.x, last.y);
    
    // Glow effect
    const glow = driftMapCtx.createRadialGradient(p.x, p.y, 0, p.x, p.y, 15);
    glow.addColorStop(0, 'rgba(239, 68, 68, 0.8)');
    glow.addColorStop(1, 'rgba(239, 68, 68, 0)');
    driftMapCtx.fillStyle = glow;
    driftMapCtx.beginPath();
    driftMapCtx.arc(p.x, p.y, 15, 0, Math.PI * 2);
    driftMapCtx.fill();
    
    // Solid center
    driftMapCtx.fillStyle = '#ef4444';
    driftMapCtx.beginPath();
    driftMapCtx.arc(p.x, p.y, 5, 0, Math.PI * 2);
    driftMapCtx.fill();
  }
}

// ==================== INERTIAL DRIFT PROOF ====================
let driftIntegrating = false;
let driftVelX = 0, driftVelY = 0;
let driftPosX = 0, driftPosY = 0;
let driftStartTime = 0;
let lastAccX = 0, lastAccY = 0, lastAccZ = 0;
let lastRoll = 0, lastPitch = 0, lastYaw = 0;
const G = 9.81; // m/s²
const DT = 0.2; // 200ms polling interval

function toggleDriftIntegration() {
  driftIntegrating = !driftIntegrating;
  const btn = document.getElementById('driftStartBtn');
  if (driftIntegrating) {
    driftStartTime = Date.now();
    btn.textContent = 'Stop Integration';
    btn.classList.remove('btn-primary');
    btn.classList.add('btn-danger');
  } else {
    btn.textContent = 'Start Integration';
    btn.classList.remove('btn-danger');
    btn.classList.add('btn-primary');
  }
}

function resetDriftIntegration() {
  driftIntegrating = false;
  driftVelX = 0; driftVelY = 0;
  driftPosX = 0; driftPosY = 0;
  driftStartTime = 0;
  document.getElementById('drift_vx').innerText = '0.00';
  document.getElementById('drift_vy').innerText = '0.00';
  document.getElementById('drift_px').innerText = '0.00';
  document.getElementById('drift_py').innerText = '0.00';
  document.getElementById('drift_total').innerText = '0.00';
  document.getElementById('drift_time').innerText = '0.0 s';
  const btn = document.getElementById('driftStartBtn');
  btn.textContent = 'Start Integration';
  btn.classList.remove('btn-danger');
  btn.classList.add('btn-primary');
  resetDriftMap();
}

function updateDriftSimulation(ax, ay, az, roll, pitch) {
  // Store raw values for display
  lastAccX = ax; lastAccY = ay; lastAccZ = az;
  lastRoll = roll; lastPitch = pitch;
  
  document.getElementById('drift_ax').innerText = ax.toFixed(4);
  document.getElementById('drift_ay').innerText = ay.toFixed(4);
  
  // Update accelerometer graph
  updateAccelGraph(ax, ay, az);
  
  // Update 3D cube rotation
  const cube = document.getElementById('cube3d');
  if (cube) {
    cube.style.transform = `rotateX(${-pitch}deg) rotateY(${lastYaw}deg) rotateZ(${-roll}deg)`;
  }
  
  if (!driftIntegrating) {
    // Still update drift map with current position even if not integrating
    if (driftTrail.length === 0) drawDriftMap();
    return;
  }
  
  // Naive gravity compensation based on angles
  // This is intentionally imperfect to show drift
  const rollRad = roll * Math.PI / 180;
  const pitchRad = pitch * Math.PI / 180;
  
  // Remove estimated gravity component (simplified)
  // Real gravity projects: gx = -sin(pitch), gy = sin(roll)*cos(pitch), gz = cos(roll)*cos(pitch)
  const gx_est = -Math.sin(pitchRad);
  const gy_est = Math.sin(rollRad) * Math.cos(pitchRad);
  
  // Linear acceleration (with noise and bias - this is the problem!)
  const linAccX = (ax - gx_est) * G;
  const linAccY = (ay - gy_est) * G;
  
  // First integration: Acceleration -> Velocity
  driftVelX += linAccX * DT;
  driftVelY += linAccY * DT;
  
  // Second integration: Velocity -> Position
  driftPosX += driftVelX * DT;
  driftPosY += driftVelY * DT;
  
  // Update display
  document.getElementById('drift_vx').innerText = driftVelX.toFixed(2);
  document.getElementById('drift_vy').innerText = driftVelY.toFixed(2);
  document.getElementById('drift_px').innerText = driftPosX.toFixed(2);
  document.getElementById('drift_py').innerText = driftPosY.toFixed(2);
  
  const totalDrift = Math.sqrt(driftPosX*driftPosX + driftPosY*driftPosY);
  document.getElementById('drift_total').innerText = totalDrift.toFixed(2);
  
  const elapsed = (Date.now() - driftStartTime) / 1000;
  document.getElementById('drift_time').innerText = elapsed.toFixed(1) + ' s';
  
  // Update 3D drift map
  updateDriftMap(driftPosX, driftPosY);
  
  // Auto-scale if drift exceeds current scale
  if (totalDrift > driftMapScale * 0.8) {
    const scaleSelect = document.getElementById('driftMapScale');
    const options = [1, 5, 10, 50, 100, 500];
    for (const opt of options) {
      if (totalDrift < opt * 0.8) {
        scaleSelect.value = opt;
        driftMapScale = opt;
        break;
      }
    }
  }
}
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
        json += "\"poy\":" + String(drone_data->pid_output_yaw) + ",";    // AJOUT: sortie PID yaw
        json += "\"ax\":" + String(drone_data->acc_x, 4) + ",";           // Accélération X (G)
        json += "\"ay\":" + String(drone_data->acc_y, 4) + ",";           // Accélération Y (G)
        json += "\"az\":" + String(drone_data->acc_z, 4);                 // Accélération Z (G)
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