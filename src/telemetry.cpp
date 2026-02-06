#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "telemetry.h"
#include "radio.h"
#include "motors.h"
#include "types.h"
#include "html_page_gz.h"

const char* ssid = "Drone_ESP32";
const char* password = "password123";

DroneState* drone_data;
AsyncWebServer server(80);

void telemetryTask(void * parameter) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html_page_gz, html_page_gz_len);
        response->addHeader("Content-Encoding", "gzip");
        request->send(response);
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
        // Buffer statique pour éviter les allocations dynamiques String (cause de lag)
        static char json_buffer[2048];

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
            "\"alt_ayw\":%.1f,"
            "\"fx\":%.4f,\"fy\":%.4f,\"fq\":%d,\"li\":%.3f,"
            "\"frx\":%.4f,\"fry\":%.4f,\"ffv\":%d,"
            "\"posx\":%.4f,\"posy\":%.4f,\"dtf\":%.4f}",
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
            drone_data->alt_angle_yaw,
            drone_data->velocity_est_x, drone_data->velocity_est_y, drone_data->flow_quality, drone_data->lidar_dist_m,
            drone_data->flow_raw_rad_x, drone_data->flow_raw_rad_y, drone_data->flow_feature_valid ? 1 : 0,
            drone_data->position_x, drone_data->position_y, drone_data->dt_flow
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
        static char json_buffer[768];
        snprintf(json_buffer, sizeof(json_buffer),
            "{\"ppr\":%.4f,\"ipr\":%.5f,\"dpr\":%.4f,"
            "\"py\":%.4f,\"iy\":%.5f,\"dy\":%.4f,"
            "\"ffpr\":%.4f,\"ffy\":%.4f,\"pl\":%.4f,\"ph\":%.4f,"
            "\"fkpp\":%.4f,\"fkpv\":%.4f,\"fkiv\":%.5f,\"fkdv\":%.5f,"
            "\"fsc\":%.6f,\"fvm\":%.2f,\"fam\":%.2f,"
            "\"fsp\":%.1f,\"fsr\":%.1f,"
            "\"tr\":%.4f,\"tp\":%.4f}",
            drone_data->p_pitch_roll, drone_data->i_pitch_roll, drone_data->d_pitch_roll,
            drone_data->p_yaw, drone_data->i_yaw, drone_data->d_yaw,
            drone_data->ff_pitch_roll, drone_data->ff_yaw, drone_data->p_level, drone_data->p_heading,
            drone_data->flow_kp_pos, drone_data->flow_kp_vel, drone_data->flow_ki_vel, drone_data->flow_kd_vel,
            drone_data->flow_scale, drone_data->flow_vel_max, drone_data->flow_angle_max,
            drone_data->flow_sign_pitch, drone_data->flow_sign_roll,
            drone_data->trim_roll, drone_data->trim_pitch
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
        // Flow cascade parameters
        if(request->hasParam("fkpp")) drone_data->flow_kp_pos = request->getParam("fkpp")->value().toFloat();
        if(request->hasParam("fkpv")) drone_data->flow_kp_vel = request->getParam("fkpv")->value().toFloat();
        if(request->hasParam("fkiv")) drone_data->flow_ki_vel = request->getParam("fkiv")->value().toFloat();
        if(request->hasParam("fkdv")) drone_data->flow_kd_vel = request->getParam("fkdv")->value().toFloat();
        if(request->hasParam("fsc")) drone_data->flow_scale = request->getParam("fsc")->value().toFloat();
        if(request->hasParam("fvm")) drone_data->flow_vel_max = request->getParam("fvm")->value().toFloat();
        if(request->hasParam("fam")) drone_data->flow_angle_max = request->getParam("fam")->value().toFloat();
        if(request->hasParam("fsp")) drone_data->flow_sign_pitch = request->getParam("fsp")->value().toFloat();
        if(request->hasParam("fsr")) drone_data->flow_sign_roll = request->getParam("fsr")->value().toFloat();
        if(request->hasParam("tr")) drone_data->trim_roll = request->getParam("tr")->value().toFloat();
        if(request->hasParam("tp")) drone_data->trim_pitch = request->getParam("tp")->value().toFloat();

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
    xTaskCreatePinnedToCore(telemetryTask, "WifiTask", 8192, NULL, 1, NULL, 0);
}
