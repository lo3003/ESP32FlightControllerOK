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
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drone Dashboard</title>
  <style>
    body { font-family: sans-serif; background: #1a1a1a; color: #eee; text-align: center; padding: 10px; }
    .card { background: #333; padding: 15px; margin: 10px auto; max-width: 400px; border-radius: 8px; border: 1px solid #444; }
    h2 { color: #00d2ff; margin: 0 0 10px 0; }
    h3 { border-bottom: 1px solid #555; padding-bottom: 5px; margin-top:0; }
    .val-box { display: flex; justify-content: space-between; margin: 5px 0; font-family: monospace;}
    
    input[type=number] { width: 60px; padding: 5px; border-radius: 4px; border: none; text-align: center; }
    .btn-update { background: #2ecc71; color: white; border: none; padding: 10px; width: 100%; border-radius: 5px; font-weight: bold; cursor: pointer; margin-top: 10px;}
    .pid-row { display: flex; justify-content: space-between; align-items: center; margin: 5px 0; }
    label { width: 50px; text-align: left; }
    
    /* Styles Test Moteur */
    .grid-2 { display:grid; grid-template-columns: 1fr 1fr; gap:10px; margin-bottom:10px; }
    .btn-test { background: #444; color: white; border: 1px solid #666; padding: 10px; border-radius: 5px; cursor: pointer; }
    .active-btn { background: #00d2ff; color: black; font-weight: bold; }
  </style>
</head>
<body>
  <h2>MonDrone Tuning</h2>

  <div class="card">
    <h3>PID Réglages</h3>
    
    <div style="text-align:left; font-weight:bold; color:#aaa; margin-bottom:5px;">Pitch / Roll</div>
    <div class="pid-row"><label>P</label> <input type="number" step="0.1" id="ppr"></div>
    <div class="pid-row"><label>I</label> <input type="number" step="0.01" id="ipr"></div>
    <div class="pid-row"><label>D</label> <input type="number" step="1.0" id="dpr"></div>

    <div style="text-align:left; font-weight:bold; color:#aaa; margin:10px 0 5px 0;">Yaw</div>
    <div class="pid-row"><label>P</label> <input type="number" step="0.1" id="py"></div>
    <div class="pid-row"><label>I</label> <input type="number" step="0.01" id="iy"></div>
    <div class="pid-row"><label>D</label> <input type="number" step="1.0" id="dy"></div>

    <div style="text-align:left; font-weight:bold; color:#aaa; margin:10px 0 5px 0;">Auto Level</div>
    <div class="pid-row"><label>Level P</label> <input type="number" step="1.0" id="pl"></div>

    <button class="btn-update" onclick="sendPID()">METTRE A JOUR PID</button>
  </div>

  <div class="card">
    <h3>État</h3>
    <div class="val-box"><span>Roll:</span> <span id="ar">0.0</span>°</div>
    <div class="val-box"><span>Pitch:</span> <span id="ap">0.0</span>°</div>
    <div class="val-box"><span>Thr:</span> <span id="th">0</span></div>
  </div>

  <div class="card">
    <h3>Test Moteurs (NO PROPS)</h3>
    <div class="grid-2">
      <button id="btn1" class="btn-test" onclick="test(1)">M1 (AvD)</button>
      <button id="btn2" class="btn-test" onclick="test(2)">M2 (ArD)</button>
      <button id="btn3" class="btn-test" onclick="test(3)">M3 (ArG)</button>
      <button id="btn4" class="btn-test" onclick="test(4)">M4 (AvG)</button>
    </div>
    <input type="range" min="1000" max="1300" value="1000" id="slider" oninput="updateVal(this.value)" style="width:100%">
    <div id="sliderVal">1000</div>
    <button style="background:#e74c3c; width:100%; margin-top:10px; padding:10px; border:none; color:white; font-weight:bold; border-radius:5px;" onclick="stopAll()">STOP</button>
  </div>

<script>
// --- PID LOGIC ---
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
    let ppr = document.getElementById('ppr').value;
    let ipr = document.getElementById('ipr').value;
    let dpr = document.getElementById('dpr').value;
    let py = document.getElementById('py').value;
    let iy = document.getElementById('iy').value;
    let dy = document.getElementById('dy').value;
    let pl = document.getElementById('pl').value;
    
    fetch(`/set_pid?ppr=${ppr}&ipr=${ipr}&dpr=${dpr}&py=${py}&iy=${iy}&dy=${dy}&pl=${pl}`)
    .then(res => alert("PID Mis à jour !"));
}

// --- TELEMETRY ---
setInterval(() => {
  fetch('/data').then(res => res.json()).then(data => {
    document.getElementById("ar").innerText = data.ar.toFixed(1);
    document.getElementById("ap").innerText = data.ap.toFixed(1);
    document.getElementById("th").innerText = data.r3;
  });
}, 200);

// --- MOTOR TEST ---
let activeMotor = 0;
function test(m) {
    activeMotor = m;
    for(let i=1; i<=4; i++) document.getElementById("btn"+i).className = "btn-test";
    document.getElementById("btn"+m).className = "btn-test active-btn";
    document.getElementById("slider").value = 1000;
    updateVal(1000);
}
function updateVal(val) {
    document.getElementById("sliderVal").innerText = val;
    if(activeMotor > 0) fetch(`/motor?m=${activeMotor}&val=${val}`);
}
function stopAll() {
    activeMotor = 0;
    for(let i=1; i<=4; i++) document.getElementById("btn"+i).className = "btn-test";
    document.getElementById("slider").value = 1000;
    fetch('/stop');
}

// Init
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

    // Envoi des données JSON pour l'affichage temps réel
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"r3\":" + String(drone_data->channel_3) + ",";
        json += "\"ar\":" + String(drone_data->angle_roll) + ",";
        json += "\"ap\":" + String(drone_data->angle_pitch);
        json += "}";
        request->send(200, "application/json", json);
    });

    // --- API PID ---
    
    // 1. Récupérer les PID actuels (au chargement de la page)
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

    // 2. Mettre à jour les PID
    server.on("/set_pid", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasParam("ppr")) drone_data->p_pitch_roll = request->getParam("ppr")->value().toFloat();
        if(request->hasParam("ipr")) drone_data->i_pitch_roll = request->getParam("ipr")->value().toFloat();
        if(request->hasParam("dpr")) drone_data->d_pitch_roll = request->getParam("dpr")->value().toFloat();
        
        if(request->hasParam("py")) drone_data->p_yaw = request->getParam("py")->value().toFloat();
        if(request->hasParam("iy")) drone_data->i_yaw = request->getParam("iy")->value().toFloat();
        if(request->hasParam("dy")) drone_data->d_yaw = request->getParam("dy")->value().toFloat();

        if(request->hasParam("pl")) drone_data->p_level = request->getParam("pl")->value().toFloat();

        // On reset l'intégrale pour éviter les sauts brutaux si on change I en vol
        // pid_reset_integral(); // Décommenter si vous voulez reset I au changement
        
        request->send(200, "text/plain", "OK");
    });

    // --- MOTEURS ---
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