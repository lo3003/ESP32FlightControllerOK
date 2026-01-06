#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "telemetry.h"
#include "radio.h"  // Pour lire raw_channel_x

const char* ssid = "Drone_ESP32"; // Nom du WiFi
const char* password = "password123";

DroneState* drone_data;
AsyncWebServer server(80);

// --- PAGE WEB (HTML/JS) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drone Dashboard</title>
  <style>
    body { font-family: sans-serif; background: #1a1a1a; color: #eee; text-align: center; padding: 20px; }
    .card { background: #333; padding: 15px; margin: 10px auto; max-width: 400px; border-radius: 8px; border: 1px solid #444; }
    h2 { color: #00d2ff; margin-top: 0; }
    .bar-container { display: flex; align-items: center; margin: 5px 0; }
    .label { width: 50px; text-align: left; font-weight: bold; }
    .bar-bg { flex-grow: 1; height: 10px; background: #555; margin: 0 10px; border-radius: 5px; position: relative; }
    .bar { height: 100%; background: #00d2ff; width: 50%; border-radius: 5px; transition: width 0.1s; }
    .val { width: 50px; text-align: right; font-family: monospace; }
    .big-val { font-size: 2em; color: #2ecc71; font-weight: bold; }
  </style>
</head>
<body>
  <h2>MonDrone Dashboard</h2>

  <div class="card">
    <h3>Radio (Raw Channels)</h3>
    <div id="radio_bars">Chargement...</div>
    <p style="font-size:0.8em; color:#aaa;">Utilisez ces valeurs pour calibrer radio.cpp</p>
  </div>

  <div class="card">
    <h3>Angles (IMU)</h3>
    <div>Roll: <span id="ang_roll" class="big-val">0.0</span>°</div>
    <div>Pitch: <span id="ang_pitch" class="big-val">0.0</span>°</div>
  </div>

<script>
function update() {
  fetch('/data').then(res => res.json()).then(data => {
    // Mise à jour Radio
    let html = "";
    let names = ["Roll", "Pitch", "Thr", "Yaw"];
    let vals = [data.r1, data.r2, data.r3, data.r4];
    
    for(let i=0; i<4; i++) {
      let pct = ((vals[i] - 1000) / 1000) * 100;
      html += `<div class="bar-container">
                 <span class="label">${names[i]}</span>
                 <div class="bar-bg"><div class="bar" style="width:${pct}%"></div></div>
                 <span class="val">${vals[i]}</span>
               </div>`;
    }
    document.getElementById("radio_bars").innerHTML = html;

    // Mise à jour Angles
    document.getElementById("ang_roll").innerText = data.ar.toFixed(1);
    document.getElementById("ang_pitch").innerText = data.ap.toFixed(1);
  });
}
setInterval(update, 100); // 10Hz
</script>
</body>
</html>
)rawliteral";

void telemetryTask(void * parameter) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("IP Dashboard: "); Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
        // JSON ultra-léger pour ne pas charger le CPU
        String json = "{";
        json += "\"r1\":" + String(raw_channel_1) + ",";
        json += "\"r2\":" + String(raw_channel_2) + ",";
        json += "\"r3\":" + String(raw_channel_3) + ",";
        json += "\"r4\":" + String(raw_channel_4) + ",";
        json += "\"ar\":" + String(drone_data->angle_roll) + ",";
        json += "\"ap\":" + String(drone_data->angle_pitch);
        json += "}";
        request->send(200, "application/json", json);
    });

    server.begin();
    vTaskDelete(NULL); // On tue la tâche d'init, le serveur tourne en fond via AsyncTCP
}

void start_telemetry_task(DroneState* drone_ptr) {
    drone_data = drone_ptr;
    // On lance sur le CORE 0 pour laisser le vol tranquille sur le CORE 1
    xTaskCreatePinnedToCore(telemetryTask, "WifiTask", 4096, NULL, 1, NULL, 0);
}