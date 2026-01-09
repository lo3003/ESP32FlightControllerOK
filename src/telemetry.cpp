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
    
    .grid-2 { display:grid; grid-template-columns: 1fr 1fr; gap:10px; margin-bottom:10px; }
    .btn-test { background: #444; color: white; border: 1px solid #666; padding: 10px; border-radius: 5px; cursor: pointer; }
    .active-btn { background: #00d2ff; color: black; font-weight: bold; }

    input[type=range].monitor {
        width: 100%; -webkit-appearance: none; height: 8px; background: #555; border-radius: 5px; outline: none; margin-top:5px;
        opacity: 0.8;
    }
    input[type=range].monitor::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 15px; height: 15px; border-radius: 50%; background: #f1c40f; }
  </style>
</head>
<body>
  <h2>MonDrone Tuning</h2>

  <div class="card">
    <h3>État & Debug</h3>
    <div class="val-box"><span>Roll:</span> <span id="ar">0.0</span>°</div>
    <div class="val-box"><span>Pitch:</span> <span id="ap">0.0</span>°</div>
    <div class="val-box" style="border-top:1px solid #555; margin-top:5px; padding-top:5px;">
        <span>Loop Time (us):</span> <span id="lt" style="color:#e74c3c">0</span>
    </div>
    <div style="font-size:10px; color:#aaa;">Target: < 4000us</div>
  </div>

  <div class="card">
    <h3>Diagnostic Lag (Max US)</h3>
    <div class="val-box"><span>Radio (RX):</span> <span id="max_rad" style="color:orange">0</span></div>
    <div class="val-box"><span>IMU (Gyro):</span> <span id="max_imu" style="color:orange">0</span></div>
    <div class="val-box"><span>PID/Mot:</span> <span id="max_pid" style="color:orange">0</span></div>
    <button class="btn-update" onclick="fetch('/reset_max')">RESET MAX</button>
  </div>

  <div class="card">
    <h3>Radio Monitor</h3>
    <div class="val-box"><span>Thr:</span> <span id="val_t">1000</span></div>
    <input type="range" min="1000" max="2000" class="monitor" id="rx_t" disabled>
    <div class="val-box"><span>Yaw:</span> <span id="val_y">1500</span></div>
    <input type="range" min="1000" max="2000" class="monitor" id="rx_y" disabled>
    <div class="val-box"><span>Pit:</span> <span id="val_p">1500</span></div>
    <input type="range" min="1000" max="2000" class="monitor" id="rx_p" disabled>
    <div class="val-box"><span>Rol:</span> <span id="val_r">1500</span></div>
    <input type="range" min="1000" max="2000" class="monitor" id="rx_r" disabled>
  </div>

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
    <h3>Test Moteurs</h3>
    <div class="grid-2">
      <button id="btn1" class="btn-test" onclick="test(1)">M1</button>
      <button id="btn2" class="btn-test" onclick="test(2)">M2</button>
      <button id="btn3" class="btn-test" onclick="test(3)">M3</button>
      <button id="btn4" class="btn-test" onclick="test(4)">M4</button>
    </div>
    <input type="range" min="1000" max="1300" value="1000" id="slider" oninput="updateVal(this.value)" style="width:100%">
    <div id="sliderVal">1000</div>
    <button style="background:#e74c3c; width:100%; margin-top:10px; padding:10px; border:none; color:white; font-weight:bold; border-radius:5px;" onclick="stopAll()">STOP</button>
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
    let ppr = document.getElementById('ppr').value;
    let ipr = document.getElementById('ipr').value;
    let dpr = document.getElementById('dpr').value;
    let py = document.getElementById('py').value;
    let iy = document.getElementById('iy').value;
    let dy = document.getElementById('dy').value;
    let pl = document.getElementById('pl').value;
    fetch(`/set_pid?ppr=${ppr}&ipr=${ipr}&dpr=${dpr}&py=${py}&iy=${iy}&dy=${dy}&pl=${pl}`).then(res => alert("OK"));
}

setInterval(() => {
  fetch('/data').then(res => res.json()).then(data => {
    document.getElementById("ar").innerText = data.ar.toFixed(1);
    document.getElementById("ap").innerText = data.ap.toFixed(1);
    
    // MAJ LOOP TIME
    let lt = data.lt;
    let el = document.getElementById("lt");
    el.innerText = lt;
    if(lt >= 4000) el.style.color = "red";
    else el.style.color = "#2ecc71";

    // MAJ DIAGNOSTIC LAG (Nouvelles valeurs)
    document.getElementById("max_rad").innerText = data.mr;
    document.getElementById("max_imu").innerText = data.mi;
    document.getElementById("max_pid").innerText = data.mp;

    document.getElementById("rx_t").value = data.r3; document.getElementById("val_t").innerText = data.r3;
    document.getElementById("rx_y").value = data.r4; document.getElementById("val_y").innerText = data.r4;
    document.getElementById("rx_p").value = data.r2; document.getElementById("val_p").innerText = data.r2;
    document.getElementById("rx_r").value = data.r1; document.getElementById("val_r").innerText = data.r1;
  });
}, 200); 

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
        
        // --- ENVOI DES DIAGNOSTICS DE LAG ---
        json += "\"mr\":" + String(drone_data->max_time_radio) + ",";
        json += "\"mi\":" + String(drone_data->max_time_imu) + ",";
        json += "\"mp\":" + String(drone_data->max_time_pid);
        
        json += "}";
        request->send(200, "application/json", json);
    });
    
    // --- ROUTE DE RESET DES COMPTEURS ---
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