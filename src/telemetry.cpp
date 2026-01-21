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

    /* ========== ESPACE 3D COMPLET ========== */
    .world-3d-container {
      background: linear-gradient(180deg, #0c1929 0%, #1a2744 50%, #243b5a 100%);
      border-radius: 12px;
      padding: 20px;
      border: 1px solid var(--card-border);
      position: relative;
      overflow: hidden;
    }
    .world-3d {
      width: 100%;
      height: 350px;
      position: relative;
      perspective: 800px;
      transform-style: preserve-3d;
    }
    .world-3d-info {
      display: flex;
      justify-content: space-between;
      margin-top: 12px;
      font-family: Consolas, monospace;
      font-size: 0.8rem;
      color: var(--text-muted);
    }
    /* Grille au sol */
    .ground-grid {
      position: absolute;
      width: 600px;
      height: 600px;
      left: 50%;
      top: 60%;
      transform: translateX(-50%) rotateX(70deg);
      background-image: 
        linear-gradient(rgba(59, 130, 246, 0.3) 1px, transparent 1px),
        linear-gradient(90deg, rgba(59, 130, 246, 0.3) 1px, transparent 1px);
      background-size: 40px 40px;
      border: 2px solid rgba(59, 130, 246, 0.5);
      transform-style: preserve-3d;
      transition: transform 0.3s ease;
    }
    /* Origine */
    .origin-marker {
      position: absolute;
      left: 50%;
      top: 60%;
      transform: translateX(-50%) translateY(-50%);
      z-index: 10;
    }
    .origin-pole {
      width: 4px;
      height: 80px;
      background: linear-gradient(to top, #22c55e, #4ade80);
      margin: 0 auto;
      border-radius: 2px;
      box-shadow: 0 0 10px #22c55e;
    }
    .origin-label {
      background: #22c55e;
      color: #000;
      padding: 2px 8px;
      border-radius: 4px;
      font-size: 0.65rem;
      font-weight: bold;
      text-align: center;
      margin-top: 4px;
    }
    /* Container drone qui se déplace */
    .drone-container {
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      transform-style: preserve-3d;
      transition: left 0.15s ease-out, top 0.15s ease-out;
      z-index: 20;
    }
    .drone-container .cube {
      width: 60px;
      height: 60px;
      transform-style: preserve-3d;
    }
    .drone-container .cube-face {
      width: 60px;
      height: 60px;
      font-size: 0.5rem;
    }
    .drone-container .cube-face.front  { transform: rotateY(0deg) translateZ(30px); }
    .drone-container .cube-face.back   { transform: rotateY(180deg) translateZ(30px); }
    .drone-container .cube-face.right  { transform: rotateY(90deg) translateZ(30px); }
    .drone-container .cube-face.left   { transform: rotateY(-90deg) translateZ(30px); }
    .drone-container .cube-face.top    { transform: rotateX(90deg) translateZ(30px); }
    .drone-container .cube-face.bottom { transform: rotateX(-90deg) translateZ(30px); }
    /* Trail 3D */
    .trail-point {
      position: absolute;
      width: 6px;
      height: 6px;
      background: #fbbf24;
      border-radius: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 0 8px #fbbf24;
    }
    /* Axes indicateurs */
    .axis {
      position: absolute;
      font-size: 0.7rem;
      font-weight: bold;
      padding: 2px 6px;
      border-radius: 4px;
    }
    .axis-x {
      right: 20px;
      top: 50%;
      background: rgba(239, 68, 68, 0.8);
      color: white;
    }
    .axis-y {
      left: 50%;
      top: 20px;
      background: rgba(34, 197, 94, 0.8);
      color: white;
      transform: translateX(-50%);
    }
    /* Glow effect drone */
    .drone-container::after {
      content: '';
      position: absolute;
      width: 80px;
      height: 80px;
      background: radial-gradient(circle, rgba(239, 68, 68, 0.4) 0%, transparent 70%);
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      border-radius: 50%;
      z-index: -1;
    }

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
      <h1><span class="status-dot"></span>Tableau de Bord Drone</h1>
      <p class="subtitle">Contrôleur de Vol ESP32 — Télémétrie Temps Réel</p>
    </header>

    <div class="dashboard">
      <!-- Carte Attitude -->
      <div class="card">
        <div class="card-header">
          <div class="icon">📐</div>
          <h3>Attitude</h3>
        </div>
        <div class="attitude-container">
          <div class="attitude-box">
            <div class="attitude-value" id="ar">0.0</div>
            <div class="attitude-label">Roulis °</div>
          </div>
          <div class="attitude-box">
            <div class="attitude-value" id="ap">0.0</div>
            <div class="attitude-label">Tangage °</div>
          </div>
          <div class="attitude-box">
            <div class="attitude-value" id="ay">0.0</div>
            <div class="attitude-label">Lacet °</div>
          </div>
        </div>
        <div class="loop-indicator">
          <span class="stat-label">Boucle</span>
          <div class="loop-bar">
            <div class="loop-fill" id="loop_bar"></div>
          </div>
          <span class="stat-value" id="lt">0</span><span style="color:var(--text-muted);font-size:0.75rem;margin-left:2px">µs</span>
        </div>
      </div>

      <!-- Carte Diagnostics -->
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
          <span class="stat-label">IMU Actuel</span>
          <span class="stat-value ok" id="cur_imu">0</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">IMU Max</span>
          <span class="stat-value" id="max_imu">0</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">PID / Moteurs</span>
          <span class="stat-value" id="max_pid">0</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">Batterie</span>
          <span class="stat-value" id="disp_bat" style="font-size:1.2rem;">0.0 V</span>
        </div>
        <button class="btn btn-primary" style="margin-top:16px" onclick="resetCounters(this)">Réinitialiser</button>
      </div>

      <!-- Carte Moniteur Radio -->
      <div class="card">
        <div class="card-header">
          <div class="icon">📡</div>
          <h3>Moniteur Radio</h3>
        </div>
        <div class="radio-channel">
          <div class="radio-header">
            <span class="radio-label">Gaz</span>
            <span class="radio-value" id="val_t">1000</span>
          </div>
          <input type="range" min="1000" max="2000" id="rx_t" disabled>
        </div>
        <div class="radio-channel">
          <div class="radio-header">
            <span class="radio-label">Lacet</span>
            <span class="radio-value" id="val_y">1500</span>
          </div>
          <input type="range" min="1000" max="2000" id="rx_y" disabled>
        </div>
        <div class="radio-channel">
          <div class="radio-header">
            <span class="radio-label">Tangage</span>
            <span class="radio-value" id="val_p">1500</span>
          </div>
          <input type="range" min="1000" max="2000" id="rx_p" disabled>
        </div>
        <div class="radio-channel">
          <div class="radio-header">
            <span class="radio-label">Roulis</span>
            <span class="radio-value" id="val_r">1500</span>
          </div>
          <input type="range" min="1000" max="2000" id="rx_r" disabled>
        </div>
      </div>

      <!-- Carte Réglage PID -->
      <div class="card">
        <div class="card-header">
          <div class="icon">🎚️</div>
          <h3>Réglage PID</h3>
        </div>
        <div class="pid-section">
          <div class="pid-title">Tangage / Roulis</div>
          <div class="pid-row"><label>P</label><input type="number" step="0.1" id="ppr"></div>
          <div class="pid-row"><label>I</label><input type="number" step="0.01" id="ipr"></div>
          <div class="pid-row"><label>D</label><input type="number" step="1.0" id="dpr"></div>
          <div class="pid-row"><label>FF</label><input type="number" step="0.01" id="ffpr"></div>

        </div>
        <div class="pid-section">
          <div class="pid-title">Lacet</div>
          <div class="pid-row"><label>P</label><input type="number" step="0.1" id="py"></div>
          <div class="pid-row"><label>I</label><input type="number" step="0.01" id="iy"></div>
          <div class="pid-row"><label>D</label><input type="number" step="1.0" id="dy"></div>
          <div class="pid-row"><label>FF</label><input type="number" step="0.01" id="ffy"></div>

        </div>
        <div class="pid-section">
          <div class="pid-title">Stabilisation Auto</div>
          <div class="pid-row"><label>Niveau P</label><input type="number" step="1.0" id="pl"></div>
          <div class="pid-row"><label>Heading P</label><input type="number" step="0.1" id="ph"></div>
        </div>
        <button class="btn btn-primary" id="pidBtn" onclick="sendPID()">Appliquer PID</button>
      </div>

      <!-- Carte Test Moteurs -->
      <div class="card">
        <div class="card-header">
          <div class="icon">🔧</div>
          <h3>Test Moteurs</h3>
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
        <button class="btn btn-danger" id="stopBtn" onclick="stopAll()">Arrêt d'Urgence</button>
      </div>

      <!-- Carte Comparaison IMU -->
      <div class="card">
        <div class="card-header">
          <div class="icon">🔬</div>
          <h3>Comparaison IMU</h3>
        </div>
        <div class="pid-title" style="margin-bottom: 8px;">MPU6050 (Principal)</div>
        <div class="stat-row">
          <span class="stat-label">Roll</span>
          <span class="stat-value" id="mpu_roll" style="color: var(--primary);">0.0°</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">Pitch</span>
          <span class="stat-value" id="mpu_pitch" style="color: var(--primary);">0.0°</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">Gyro Yaw</span>
          <span class="stat-value" id="mpu_gyaw" style="color: var(--primary);">0.0°/s</span>
        </div>
        <div class="pid-title" style="margin-top: 16px; margin-bottom: 8px; border-top: 1px solid var(--card-border); padding-top: 12px;">AltIMU-10 v2 (Secondaire)</div>
        <div class="stat-row">
          <span class="stat-label">Roll</span>
          <span class="stat-value" id="alt_roll" style="color: var(--success);">0.0°</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">Pitch</span>
          <span class="stat-value" id="alt_pitch" style="color: var(--success);">0.0°</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">Yaw (Mag)</span>
          <span class="stat-value" id="alt_yaw" style="color: var(--success);">0.0°</span>
        </div>
        <div class="pid-title" style="margin-top: 16px; margin-bottom: 8px; border-top: 1px solid var(--warning); padding-top: 12px;">Écart (MPU - Alt)</div>
        <div class="stat-row">
          <span class="stat-label">ΔRoll</span>
          <span class="stat-value warning" id="diff_roll">0.0°</span>
        </div>
        <div class="stat-row">
          <span class="stat-label">ΔPitch</span>
          <span class="stat-value warning" id="diff_pitch">0.0°</span>
        </div>
      </div>

      <!-- Carte Preuve de Dérive Inertielle - PLEINE LARGEUR -->
      <div class="card" style="grid-column: 1 / -1;">
        <div class="card-header">
          <div class="icon">🎯</div>
          <h3>Preuve de Dérive Inertielle (PoC) — Pourquoi le GPS/VIO est Obligatoire</h3>
        </div>
        <div style="display: flex; gap: 24px; flex-wrap: wrap;">
          <!-- ESPACE 3D COMPLET avec cube qui dérive -->
          <div style="flex: 2; min-width: 450px;">
            <div style="text-align: center; margin-bottom: 12px;">
              <span class="stat-label">Espace 3D — Visualisation de la Dérive</span>
            </div>
            <div class="world-3d-container">
              <div class="world-3d" id="world3d">
                <!-- Grille au sol -->
                <div class="ground-grid" id="groundGrid"></div>
                <!-- Origine marker -->
                <div class="origin-marker" id="originMarker">
                  <div class="origin-pole"></div>
                  <div class="origin-label">ORIGINE</div>
                </div>
                <!-- Cube drone qui se déplace -->
                <div class="drone-container" id="droneContainer">
                  <div class="cube" id="cube3d">
                    <div class="cube-face front">AVANT</div>
                    <div class="cube-face back">ARRIÈRE</div>
                    <div class="cube-face right">DROITE</div>
                    <div class="cube-face left">GAUCHE</div>
                    <div class="cube-face top">HAUT</div>
                    <div class="cube-face bottom">BAS</div>
                  </div>
                  <!-- Trail 3D -->
                  <div class="drone-trail" id="droneTrail"></div>
                </div>
                <!-- Axes -->
                <div class="axis axis-x"><span>+X</span></div>
                <div class="axis axis-y"><span>+Y</span></div>
              </div>
              <div class="world-3d-info">
                <span id="world3d_pos">Position: (0.0, 0.0) m</span>
                <span id="world3d_dist">Distance: 0.0 m</span>
              </div>
            </div>
            <div style="display: flex; gap: 10px; margin-top: 12px; justify-content: center; flex-wrap: wrap;">
              <button class="btn btn-primary" style="width: auto; padding: 8px 16px;" id="driftStartBtn" onclick="toggleDriftIntegration()">Démarrer Intégration</button>
              <button class="btn btn-danger" style="width: auto; padding: 8px 16px;" onclick="resetDriftIntegration()">Réinitialiser</button>
              <select id="driftMapScale" onchange="updateDriftMapScale()" style="padding: 8px; background: var(--input-bg); color: var(--text); border: 1px solid var(--card-border); border-radius: 6px;">
                <option value="1">Échelle: 1m</option>
                <option value="5">Échelle: 5m</option>
                <option value="10" selected>Échelle: 10m</option>
                <option value="50">Échelle: 50m</option>
                <option value="100">Échelle: 100m</option>
                <option value="500">Échelle: 500m</option>
              </select>
              <select id="cameraAngle" onchange="updateCameraAngle()" style="padding: 8px; background: var(--input-bg); color: var(--text); border: 1px solid var(--card-border); border-radius: 6px;">
                <option value="iso" selected>Vue Isométrique</option>
                <option value="top">Vue Dessus</option>
                <option value="side">Vue Côté</option>
                <option value="front">Vue Face</option>
              </select>
            </div>
          </div>
          <!-- Simulateur d'Intégration -->
          <div style="flex: 1; min-width: 260px;">
            <div style="text-align: center; margin-bottom: 12px;">
              <span class="stat-label">Simulateur Double Intégration</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Accél X (G)</span>
              <span class="stat-value" id="drift_ax">0.000</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Accél Y (G)</span>
              <span class="stat-value" id="drift_ay">0.000</span>
            </div>
            <div class="stat-row" style="border-top: 2px solid var(--warning);">
              <span class="stat-label">Vitesse X (m/s)</span>
              <span class="stat-value warning" id="drift_vx">0.00</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Vitesse Y (m/s)</span>
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
              <span class="stat-label">Dérive Totale (m)</span>
              <span class="stat-value danger" id="drift_total" style="font-size: 1.1rem;">0.00</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Temps Écoulé</span>
              <span class="stat-value" id="drift_time">0.0 s</span>
            </div>
            <div style="margin-top: 16px; padding: 12px; background: var(--input-bg); border-radius: 8px;">
              <div class="stat-label" style="margin-bottom: 8px;">Attitude Actuelle</div>
              <div style="display: flex; justify-content: space-between; font-family: Consolas; font-size: 0.85rem;">
                <span>Roulis: <span id="att_roll" style="color: var(--primary);">0.0°</span></span>
                <span>Tangage: <span id="att_pitch" style="color: var(--success);">0.0°</span></span>
                <span>Lacet: <span id="att_yaw" style="color: var(--warning);">0.0°</span></span>
              </div>
            </div>
          </div>
        </div>
        <p style="color: var(--text-muted); font-size: 0.7rem; margin-top: 16px; text-align: center;">
          ⚠️ Observez comment la position dérive vers l'infini même lorsque le drone est immobile. Cela prouve que le GPS/VIO est obligatoire pour le maintien de position.
        </p>
      </div>

      <!-- Carte Graphique Accéléromètre - PLEINE LARGEUR -->
      <div class="card" style="grid-column: 1 / -1;">
        <div class="card-header">
          <div class="icon">📈</div>
          <h3>Graphique Accéléromètre Temps Réel</h3>
        </div>
        <div style="display: flex; gap: 20px; flex-wrap: wrap;">
          <div style="flex: 3; min-width: 500px;">
            <canvas id="accelGraph" width="700" height="250"></canvas>
            <div style="display: flex; gap: 10px; margin-top: 10px; justify-content: center; align-items: center; flex-wrap: wrap;">
              <span class="stat-label">Échelle:</span>
              <select id="accelGraphScale" onchange="updateAccelGraphScale()" style="padding: 8px 12px; background: var(--input-bg); color: var(--text); border: 1px solid var(--card-border); border-radius: 6px; font-family: Consolas;">
                <option value="0.01">±0.01 G (micro)</option>
                <option value="0.05">±0.05 G (fin)</option>
                <option value="0.1">±0.1 G (précis)</option>
                <option value="0.5">±0.5 G (normal)</option>
                <option value="1">±1 G (large)</option>
                <option value="2" selected>±2 G (max)</option>
              </select>
              <button class="btn btn-primary" style="width: auto; padding: 8px 16px;" onclick="autoScaleAccelGraph()">Auto-Échelle</button>
              <button class="btn btn-danger" style="width: auto; padding: 8px 16px;" onclick="clearAccelGraph()">Effacer</button>
            </div>
          </div>
          <div style="flex: 1; min-width: 150px;">
            <div class="stat-row">
              <span class="stat-label" style="color: #ef4444;">Ax (G)</span>
              <span class="stat-value" id="graph_ax">0.0000</span>
            </div>
            <div class="stat-row">
              <span class="stat-label" style="color: #22c55e;">Ay (G)</span>
              <span class="stat-value" id="graph_ay">0.0000</span>
            </div>
            <div class="stat-row">
              <span class="stat-label" style="color: #3b82f6;">Az (G)</span>
              <span class="stat-value" id="graph_az">0.0000</span>
            </div>
            <div class="stat-row" style="margin-top: 12px; border-top: 2px solid var(--warning);">
              <span class="stat-label">Bruit X (σ)</span>
              <span class="stat-value warning" id="graph_noise_x">0.0000</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Bruit Y (σ)</span>
              <span class="stat-value warning" id="graph_noise_y">0.0000</span>
            </div>
            <div class="stat-row">
              <span class="stat-label">Bruit Z (σ)</span>
              <span class="stat-value warning" id="graph_noise_z">0.0000</span>
            </div>
            <div class="stat-row" style="margin-top: 12px;">
              <span class="stat-label">Plage actuelle</span>
              <span class="stat-value" id="graph_range">±2.000 G</span>
            </div>
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
    document.getElementById('ph').value = data.ph;
    document.getElementById('ffpr').value = data.ffpr;
    document.getElementById('ffy').value = data.ffy;

  });
}

// Send PID with feedback
function sendPID() {
  const btn = document.getElementById('pidBtn');
  const originalText = btn.textContent;
  btn.disabled = true;
  btn.textContent = 'Mise à jour...';
  
  let params = ['ppr','ipr','dpr','py','iy','dy','pl','ph','ffpr','ffy'].map(id => `${id}=${document.getElementById(id).value}`).join('&');
  
  fetch(`/set_pid?${params}`)
    .then(res => {
      if (res.ok) {
        btn.textContent = '✓ Appliqué';
        btn.classList.add('success');
        showToast('Paramètres PID mis à jour avec succès', 'success');
      } else {
        throw new Error('Failed');
      }
    })
    .catch(() => {
      showToast('Échec de la mise à jour PID', 'danger');
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
  btn.textContent = 'Réinit...';
  
  fetch('/reset_max')
    .then(res => {
      if (res.ok) {
        btn.textContent = '✓ Réinit';
        btn.classList.add('success');
        showToast('Compteurs réinitialisés', 'success');
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
    document.getElementById("ay").innerText = data.alt_ayw.toFixed(1);  // Yaw fusionné AltIMU
    
    // Update Yaw for cube (utilise le yaw fusionné AltIMU)
    lastYaw = data.alt_ayw;

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

    // Batterie
    let batEl = document.getElementById("disp_bat");
    batEl.innerText = data.vb.toFixed(1) + " V";
    if (data.vb < 10.5) { batEl.style.color = "var(--danger)"; }
    else if (data.vb < 11.1) { batEl.style.color = "var(--warning)"; }
    else { batEl.style.color = "var(--success)"; }

    // Radio
    document.getElementById("rx_t").value = data.r3; document.getElementById("val_t").innerText = data.r3;
    document.getElementById("rx_y").value = data.r4; document.getElementById("val_y").innerText = data.r4;
    document.getElementById("rx_p").value = data.r2; document.getElementById("val_p").innerText = data.r2;
    document.getElementById("rx_r").value = data.r1; document.getElementById("val_r").innerText = data.r1;
    
    // Comparaison IMU - MPU6050 vs AltIMU-10
    document.getElementById("mpu_roll").innerText = data.ar.toFixed(1) + "°";
    document.getElementById("mpu_pitch").innerText = data.ap.toFixed(1) + "°";
    document.getElementById("mpu_gyaw").innerText = data.gy.toFixed(1) + "°/s";
    
    document.getElementById("alt_roll").innerText = (data.alt_ar !== undefined ? data.alt_ar.toFixed(1) : "N/A") + "°";
    document.getElementById("alt_pitch").innerText = (data.alt_ap !== undefined ? data.alt_ap.toFixed(1) : "N/A") + "°";
    document.getElementById("alt_yaw").innerText = (data.alt_ayw !== undefined ? data.alt_ayw.toFixed(1) : "N/A") + "°";
    
    // Écart entre les deux IMU
    if (data.alt_ar !== undefined && data.alt_ap !== undefined) {
      let diffRoll = data.ar - data.alt_ar;
      let diffPitch = data.ap - data.alt_ap;
      document.getElementById("diff_roll").innerText = diffRoll.toFixed(1) + "°";
      document.getElementById("diff_pitch").innerText = diffPitch.toFixed(1) + "°";
      // Colorer en rouge si écart > 5°
      document.getElementById("diff_roll").style.color = Math.abs(diffRoll) > 5 ? "var(--danger)" : "var(--warning)";
      document.getElementById("diff_pitch").style.color = Math.abs(diffPitch) > 5 ? "var(--danger)" : "var(--warning)";
    }

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
  showToast('Moteur ' + m + ' sélectionné', 'warning');
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
        showToast('⚠ ARRÊT D\'URGENCE ACTIVÉ', 'danger');
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
let accelGraphScale = 2.0; // Default ±2G

function initAccelGraph() {
  const canvas = document.getElementById('accelGraph');
  if (!canvas) return;
  accelGraphCtx = canvas.getContext('2d');
  updateAccelGraphScale();
}

function clearAccelGraph() {
  accelDataX = []; accelDataY = []; accelDataZ = [];
}

function updateAccelGraphScale() {
  const select = document.getElementById('accelGraphScale');
  if (select) {
    accelGraphScale = parseFloat(select.value);
    document.getElementById('graph_range').innerText = '±' + accelGraphScale.toFixed(3) + ' G';
  }
}

function autoScaleAccelGraph() {
  // Trouver le max des données actuelles
  if (accelDataX.length < 10) return;
  
  const allData = [...accelDataX, ...accelDataY, ...accelDataZ];
  const maxVal = Math.max(...allData.map(Math.abs));
  
  // Choisir l'échelle appropriée
  const scales = [0.01, 0.05, 0.1, 0.5, 1, 2];
  let bestScale = 2;
  for (const s of scales) {
    if (maxVal < s * 0.9) {
      bestScale = s;
      break;
    }
  }
  
  accelGraphScale = bestScale;
  document.getElementById('accelGraphScale').value = bestScale;
  document.getElementById('graph_range').innerText = '±' + bestScale.toFixed(3) + ' G';
}

function calcStdDev(arr) {
  if (arr.length < 2) return 0;
  const last50 = arr.slice(-50);
  const mean = last50.reduce((a,b) => a+b, 0) / last50.length;
  const variance = last50.reduce((a,b) => a + (b-mean)*(b-mean), 0) / last50.length;
  return Math.sqrt(variance);
}

function updateAccelGraph(ax, ay, az) {
  if (!accelGraphCtx) return;
  
  // Store data
  accelDataX.push(ax); accelDataY.push(ay); accelDataZ.push(az);
  if (accelDataX.length > GRAPH_MAX_POINTS) {
    accelDataX.shift(); accelDataY.shift(); accelDataZ.shift();
  }
  
  // Update display values (plus de décimales pour micro-variations)
  document.getElementById('graph_ax').innerText = ax.toFixed(4);
  document.getElementById('graph_ay').innerText = ay.toFixed(4);
  document.getElementById('graph_az').innerText = az.toFixed(4);
  
  // Calculate noise (standard deviation) pour chaque axe
  if (accelDataX.length > 10) {
    document.getElementById('graph_noise_x').innerText = calcStdDev(accelDataX).toFixed(4);
    document.getElementById('graph_noise_y').innerText = calcStdDev(accelDataY).toFixed(4);
    document.getElementById('graph_noise_z').innerText = calcStdDev(accelDataZ).toFixed(4);
  }
  
  // Draw graph
  const canvas = accelGraphCtx.canvas;
  const w = canvas.width, h = canvas.height;
  accelGraphCtx.fillStyle = '#0f172a';
  accelGraphCtx.fillRect(0, 0, w, h);
  
  // Grid - 5 lignes horizontales
  accelGraphCtx.strokeStyle = '#334155';
  accelGraphCtx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = h * i / 4;
    accelGraphCtx.beginPath();
    accelGraphCtx.moveTo(0, y);
    accelGraphCtx.lineTo(w, y);
    accelGraphCtx.stroke();
  }
  
  // Ligne centrale plus visible
  accelGraphCtx.strokeStyle = '#475569';
  accelGraphCtx.lineWidth = 2;
  accelGraphCtx.beginPath();
  accelGraphCtx.moveTo(0, h/2);
  accelGraphCtx.lineTo(w, h/2);
  accelGraphCtx.stroke();
  
  // Labels dynamiques selon l'échelle
  accelGraphCtx.fillStyle = '#94a3b8';
  accelGraphCtx.font = '10px Consolas';
  const s = accelGraphScale;
  const fmt = s < 0.1 ? 3 : (s < 1 ? 2 : 1);
  accelGraphCtx.fillText('+' + s.toFixed(fmt) + 'G', 5, 15);
  accelGraphCtx.fillText('+' + (s/2).toFixed(fmt) + 'G', 5, h/4 + 5);
  accelGraphCtx.fillText(' 0G', 5, h/2 + 5);
  accelGraphCtx.fillText('-' + (s/2).toFixed(fmt) + 'G', 5, 3*h/4 + 5);
  accelGraphCtx.fillText('-' + s.toFixed(fmt) + 'G', 5, h - 5);
  
  // Draw lines avec échelle dynamique
  const drawLine = (data, color) => {
    if (data.length < 2) return;
    accelGraphCtx.strokeStyle = color;
    accelGraphCtx.lineWidth = 2;
    accelGraphCtx.beginPath();
    for (let i = 0; i < data.length; i++) {
      const x = (i / GRAPH_MAX_POINTS) * w;
      // Échelle dynamique: -accelGraphScale à +accelGraphScale
      const normalized = data[i] / accelGraphScale; // -1 à +1
      const y = h/2 - normalized * (h/2);
      // Clamper pour éviter de sortir du canvas
      const yc = Math.max(0, Math.min(h, y));
      if (i === 0) accelGraphCtx.moveTo(x, yc);
      else accelGraphCtx.lineTo(x, yc);
    }
    accelGraphCtx.stroke();
  };
  
  drawLine(accelDataX, '#ef4444'); // Red for X
  drawLine(accelDataY, '#22c55e'); // Green for Y
  drawLine(accelDataZ, '#3b82f6'); // Blue for Z
  
  // Légende en bas à droite
  accelGraphCtx.font = '11px Consolas';
  accelGraphCtx.fillStyle = '#ef4444';
  accelGraphCtx.fillText('■ X', w - 80, h - 25);
  accelGraphCtx.fillStyle = '#22c55e';
  accelGraphCtx.fillText('■ Y', w - 55, h - 25);
  accelGraphCtx.fillStyle = '#3b82f6';
  accelGraphCtx.fillText('■ Z', w - 30, h - 25);
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
    btn.textContent = 'Arrêter Intégration';
    btn.classList.remove('btn-primary');
    btn.classList.add('btn-danger');
  } else {
    btn.textContent = 'Démarrer Intégration';
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
  btn.textContent = 'Démarrer Intégration';
  btn.classList.remove('btn-danger');
  btn.classList.add('btn-primary');
  resetDriftMap();
  // Reset world 3D trail
  world3dTrail = [];
  const trailEl = document.getElementById('droneTrail');
  if (trailEl) trailEl.innerHTML = '';
  // Reset drone position
  const container = document.getElementById('droneContainer');
  if (container) {
    container.style.left = '50%';
    container.style.top = '50%';
  }
}

// Trail 3D pour l'espace monde
let world3dTrail = [];
const MAX_WORLD_TRAIL = 100;
let currentCameraAngle = 'iso';

function updateCameraAngle() {
  const angle = document.getElementById('cameraAngle').value;
  currentCameraAngle = angle;
  const grid = document.getElementById('groundGrid');
  const world = document.getElementById('world3d');
  
  if (angle === 'iso') {
    grid.style.transform = 'translateX(-50%) rotateX(70deg)';
  } else if (angle === 'top') {
    grid.style.transform = 'translateX(-50%) rotateX(90deg)';
  } else if (angle === 'side') {
    grid.style.transform = 'translateX(-50%) rotateX(70deg) rotateZ(45deg)';
  } else if (angle === 'front') {
    grid.style.transform = 'translateX(-50%) rotateX(80deg) rotateZ(90deg)';
  }
}

function updateWorld3D(posX, posY, roll, pitch, yaw) {
  const container = document.getElementById('droneContainer');
  const worldCube = document.getElementById('cube3d');
  const trailEl = document.getElementById('droneTrail');
  
  if (!container || !worldCube) return;
  
  // Calculer la position sur l'écran (pixels) - échelle adaptative
  const scale = Math.max(20, 200 / Math.max(Math.abs(posX), Math.abs(posY), 1));
  const screenX = 50 + (posX * scale / 6);  // % depuis le centre
  const screenY = 50 - (posY * scale / 6);  // Y inversé pour l'écran
  
  // Limiter aux bords
  const clampedX = Math.max(10, Math.min(90, screenX));
  const clampedY = Math.max(15, Math.min(85, screenY));
  
  container.style.left = clampedX + '%';
  container.style.top = clampedY + '%';
  
  // Rotation du cube selon attitude
  worldCube.style.transform = `rotateX(${-pitch}deg) rotateY(${yaw}deg) rotateZ(${-roll}deg)`;
  
  // Ajouter au trail
  if (driftIntegrating) {
    world3dTrail.push({x: clampedX, y: clampedY, t: Date.now()});
    if (world3dTrail.length > MAX_WORLD_TRAIL) world3dTrail.shift();
  }
  
  // Dessiner le trail
  trailEl.innerHTML = '';
  for (let i = 0; i < world3dTrail.length; i++) {
    const pt = world3dTrail[i];
    const age = (Date.now() - pt.t) / 5000; // fade sur 5s
    const opacity = Math.max(0.1, 1 - age);
    const size = Math.max(3, 6 * (1 - age * 0.5));
    
    const dot = document.createElement('div');
    dot.className = 'trail-point';
    dot.style.left = pt.x + '%';
    dot.style.top = pt.y + '%';
    dot.style.width = size + 'px';
    dot.style.height = size + 'px';
    dot.style.opacity = opacity;
    trailEl.appendChild(dot);
  }
  
  // Mettre à jour les infos
  const posEl = document.getElementById('world3d_pos');
  const distEl = document.getElementById('world3d_dist');
  if (posEl) posEl.innerText = `X: ${posX.toFixed(2)}m, Y: ${posY.toFixed(2)}m`;
  if (distEl) {
    const dist = Math.sqrt(posX*posX + posY*posY);
    distEl.innerText = `Distance: ${dist.toFixed(2)}m`;
  }
  
  // Attitude display
  const attRoll = document.getElementById('att_roll');
  const attPitch = document.getElementById('att_pitch');
  const attYaw = document.getElementById('att_yaw');
  if (attRoll) attRoll.innerText = roll.toFixed(1) + '°';
  if (attPitch) attPitch.innerText = pitch.toFixed(1) + '°';
  if (attYaw) attYaw.innerText = yaw.toFixed(1) + '°';
}

function updateDriftSimulation(ax, ay, az, roll, pitch) {
  // Store raw values for display
  lastAccX = ax; lastAccY = ay; lastAccZ = az;
  lastRoll = roll; lastPitch = pitch;
  
  document.getElementById('drift_ax').innerText = ax.toFixed(4);
  document.getElementById('drift_ay').innerText = ay.toFixed(4);
  
  // Update accelerometer graph
  updateAccelGraph(ax, ay, az);
  
  // Mise à jour de l'espace 3D même si pas en intégration (position 0,0)
  updateWorld3D(driftPosX, driftPosY, roll, pitch, lastYaw);
  
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
  
  // Mise à jour de l'espace 3D avec nouvelle position
  updateWorld3D(driftPosX, driftPosY, roll, pitch, lastYaw);
  
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
        // Buffer statique pour éviter les allocations dynamiques String (cause de lag)
        static char json_buffer[512];
        
        snprintf(json_buffer, sizeof(json_buffer),
            "{\"ar\":%.2f,\"ap\":%.2f,\"ay\":%.2f,"
            "\"r1\":%d,\"r2\":%d,\"r3\":%d,\"r4\":%d,"
            "\"lt\":%lu,\"mr\":%lu,\"mi\":%lu,\"ci\":%lu,\"mp\":%lu,"
            "\"gy\":%.2f,\"poy\":%.2f,"
            "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
            "\"vb\":%.1f,"
            "\"alt_ar\":%.2f,\"alt_ap\":%.2f,"
            "\"alt_ax\":%.4f,\"alt_ay\":%.4f,\"alt_az\":%.4f,"
            "\"alt_gr\":%.2f,\"alt_gp\":%.2f,\"alt_gy\":%.2f,"
            "\"alt_ayw\":%.1f}",
            drone_data->angle_roll, drone_data->angle_pitch, drone_data->angle_yaw,
            drone_data->channel_1, drone_data->channel_2, drone_data->channel_3, drone_data->channel_4,
            drone_data->loop_time, drone_data->max_time_radio, drone_data->max_time_imu,
            drone_data->current_time_imu, drone_data->max_time_pid,
            drone_data->gyro_yaw_input, drone_data->pid_output_yaw,
            drone_data->acc_x, drone_data->acc_y, drone_data->acc_z,
            drone_data->voltage_bat,
            drone_data->alt_angle_roll, drone_data->alt_angle_pitch,
            drone_data->alt_acc_x, drone_data->alt_acc_y, drone_data->alt_acc_z,
            drone_data->alt_gyro_roll, drone_data->alt_gyro_pitch, drone_data->alt_gyro_yaw,
            drone_data->alt_angle_yaw
        );
        request->send(200, "application/json", json_buffer);
    });
    
    server.on("/reset_max", HTTP_GET, [](AsyncWebServerRequest *request){
        drone_data->max_time_radio = 0;
        drone_data->max_time_imu = 0;
        drone_data->max_time_pid = 0;
        request->send(200, "text/plain", "OK");
    });

    server.on("/get_pid", HTTP_GET, [](AsyncWebServerRequest *request){
        static char json_buffer[320];
        snprintf(json_buffer, sizeof(json_buffer),
            "{\"ppr\":%.4f,\"ipr\":%.5f,\"dpr\":%.4f,"
            "\"py\":%.4f,\"iy\":%.5f,\"dy\":%.4f,"
            "\"ffpr\":%.4f,\"ffy\":%.4f,\"pl\":%.4f,\"ph\":%.4f}",
            drone_data->p_pitch_roll, drone_data->i_pitch_roll, drone_data->d_pitch_roll,
            drone_data->p_yaw, drone_data->i_yaw, drone_data->d_yaw,
            drone_data->ff_pitch_roll, drone_data->ff_yaw, drone_data->p_level, drone_data->p_heading
        );
        request->send(200, "application/json", json_buffer);
    });

    server.on("/set_pid", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasParam("ppr")) drone_data->p_pitch_roll = request->getParam("ppr")->value().toFloat();
        if(request->hasParam("ipr")) drone_data->i_pitch_roll = request->getParam("ipr")->value().toFloat();
        if(request->hasParam("dpr")) drone_data->d_pitch_roll = request->getParam("dpr")->value().toFloat();
        if(request->hasParam("py")) drone_data->p_yaw = request->getParam("py")->value().toFloat();
        if(request->hasParam("iy")) drone_data->i_yaw = request->getParam("iy")->value().toFloat();
        if(request->hasParam("dy")) drone_data->d_yaw = request->getParam("dy")->value().toFloat();
        if(request->hasParam("pl")) drone_data->p_level = request->getParam("pl")->value().toFloat();
        if(request->hasParam("ph")) drone_data->p_heading = request->getParam("ph")->value().toFloat();
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
    // Déplacer sur Core 1 pour ne pas interférer avec l'IMU principal sur Core 0
    // Stack augmentée pour le WiFi qui est gourmand en mémoire
    xTaskCreatePinnedToCore(telemetryTask, "WifiTask", 8192, NULL, 1, NULL, 1);
}