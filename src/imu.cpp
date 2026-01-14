#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "config.h"
#include "motors.h"
#include "kalman.h"

// --- FreeRTOS ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- Offsets calibration ---
static double gyro_off_x = 0;
static double gyro_off_y = 0;
static double gyro_off_z = 0;

// --- Variables brutes ---
static int16_t acc_raw[3];
static int16_t gyro_raw[3];
static int16_t temperature;

// --- Filtres PT1 pour gyro (entrées PID) ---
static float gyro_roll_filt = 0.0f;
static float gyro_pitch_filt = 0.0f;
static float gyro_yaw_filt = 0.0f;
#define GYRO_PT1_COEFF 0.5f

// --- Filtres de Kalman pour Roll et Pitch ---
static Kalman kalman_roll;
static Kalman kalman_pitch;
static float yaw_angle = 0.0f;  // AJOUT: intégration simple du Yaw
static bool kalman_initialized = false;

// ==================== SNAPSHOT IMU ====================
typedef struct {
    float gyro_roll_input;
    float gyro_pitch_input;
    float gyro_yaw_input;
    float angle_roll;
    float angle_pitch;
    float angle_yaw;          // AJOUT
    float acc_total_vector;
    unsigned long last_dur_us;
    unsigned long last_ok_ms;
    bool ok;
} ImuSnapshot;

static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static ImuSnapshot imu_snap = {0};

static volatile FlightMode imu_in_mode = MODE_SAFE;
static volatile int imu_in_ch3 = 1000;
static volatile bool imu_reset_req = false;

static DroneState imu_state;
static TaskHandle_t imu_task_handle = nullptr;

// ==================== IMU INIT ====================
void imu_init() {
    motors_write_direct(1000, 1000, 1000, 1000);

    Serial.println(F("IMU: Init Raw I2C..."));

    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();

    Serial.println(F("IMU: Calib Gyro (1000 samples)..."));
    digitalWrite(PIN_LED, HIGH);

    long gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    for (int i = 0; i < 1000; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_ADDR, 6);

        if (Wire.available() < 6) { delay(2); continue; }

        gyro_sum_x += (int16_t)(Wire.read() << 8 | Wire.read());
        gyro_sum_y += (int16_t)(Wire.read() << 8 | Wire.read());
        gyro_sum_z += (int16_t)(Wire.read() << 8 | Wire.read());

        delayMicroseconds(2000);
    }

    gyro_off_x = gyro_sum_x / 1000.0;
    gyro_off_y = gyro_sum_y / 1000.0;
    gyro_off_z = gyro_sum_z / 1000.0;

    // Configuration Kalman pour MPU6050
    kalman_roll.setQangle(0.001f);
    kalman_roll.setQbias(0.003f);
    kalman_roll.setRmeasure(0.03f);
    
    kalman_pitch.setQangle(0.001f);
    kalman_pitch.setQbias(0.003f);
    kalman_pitch.setRmeasure(0.03f);

    digitalWrite(PIN_LED, LOW);
    Serial.println(F("IMU: Calibration OK (Kalman Filter Enabled)"));
}

// ==================== IMU READ INTERNAL ====================
static void imu_read_internal(DroneState *drone) {
    static unsigned long last_us = 0;
    const unsigned long now_us = micros();
    float dt_s = 0.004f;
    if (last_us != 0) {
        dt_s = (now_us - last_us) * 1e-6f;
        if (dt_s < 0.002f) dt_s = 0.002f;
        if (dt_s > 0.010f) dt_s = 0.010f;
    }
    last_us = now_us;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();

    uint8_t count = Wire.requestFrom(MPU_ADDR, 14);

    if (count < 14) {
        drone->max_time_imu = 888888;
        return;
    }

    acc_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
    temperature = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());

    double gyro_x_cal = gyro_raw[0] - gyro_off_x;
    double gyro_y_cal = gyro_raw[1] - gyro_off_y;
    double gyro_z_cal = gyro_raw[2] - gyro_off_z;

    // Mapping axes
    double gyro_roll  = gyro_x_cal;
    long acc_roll_val = acc_raw[1];

    double gyro_pitch = -gyro_y_cal;
    long acc_pitch_val = acc_raw[0];

    // CORRECTION: Inverser le signe du gyro Yaw
       // YAW : selon l'orientation de la carte, il peut être nécessaire d'inverser le signe.
    #if IMU_INVERT_YAW
        double gyro_yaw = -gyro_z_cal;
    #else
        double gyro_yaw = gyro_z_cal;
    #endif
        long acc_yaw_val  = acc_raw[2];


    const float gyro_scale = 65.5f;

    // Gyro en degrés/seconde
    float gyro_roll_dps  = (float)(gyro_roll / gyro_scale);
    float gyro_pitch_dps = (float)(gyro_pitch / gyro_scale);
    float gyro_yaw_dps   = (float)(gyro_yaw / gyro_scale);

    // Filtre PT1 pour les entrées PID
    gyro_roll_filt  += GYRO_PT1_COEFF * (gyro_roll_dps  - gyro_roll_filt);
    gyro_pitch_filt += GYRO_PT1_COEFF * (gyro_pitch_dps - gyro_pitch_filt);
    gyro_yaw_filt   += GYRO_PT1_COEFF * (gyro_yaw_dps   - gyro_yaw_filt);

    drone->gyro_roll_input  = gyro_roll_filt;
    drone->gyro_pitch_input = gyro_pitch_filt;
    drone->gyro_yaw_input   = gyro_yaw_filt;

    // Calcul du vecteur accélération total
    drone->acc_total_vector = sqrtf((float)(acc_roll_val * acc_roll_val) +
                                    (float)(acc_pitch_val * acc_pitch_val) +
                                    (float)(acc_yaw_val * acc_yaw_val));

    // Calcul angles accéléromètre
    float angle_pitch_acc = 0.0f, angle_roll_acc = 0.0f;

    if (fabsf((float)acc_pitch_val) < drone->acc_total_vector) {
        angle_pitch_acc = asinf((float)acc_pitch_val / drone->acc_total_vector) * RAD_TO_DEG;
    }
    if (fabsf((float)acc_roll_val) < drone->acc_total_vector) {
        angle_roll_acc = asinf((float)acc_roll_val / drone->acc_total_vector) * RAD_TO_DEG;
    }

    // Trim mécanique (ajuster selon calibration physique du drone)
    angle_roll_acc  += 0.0f;
    angle_pitch_acc += 0.0f;

    // --- FILTRE DE KALMAN ---
    if (!kalman_initialized) {
        kalman_roll.setAngle(angle_roll_acc);
        kalman_pitch.setAngle(angle_pitch_acc);
        drone->angle_roll = angle_roll_acc;
        drone->angle_pitch = angle_pitch_acc;
        kalman_initialized = true;
    } else {
        // Mode adaptatif
        if (drone->current_mode == MODE_SAFE && drone->channel_3 < 1050) {
            kalman_roll.setRmeasure(0.01f);
            kalman_pitch.setRmeasure(0.01f);
        } else {
            kalman_roll.setRmeasure(0.05f);
            kalman_pitch.setRmeasure(0.05f);
        }

        drone->angle_roll  = kalman_roll.update(angle_roll_acc, gyro_roll_dps, dt_s);
        drone->angle_pitch = kalman_pitch.update(angle_pitch_acc, gyro_pitch_dps, dt_s);
    }

    // AJOUT: Intégration Yaw (pas de correction accéléromètre possible)
    yaw_angle += gyro_yaw_dps * dt_s;
    
    // Normalisation -180 à +180
    while (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    while (yaw_angle < -180.0f) yaw_angle += 360.0f;
    
    drone->angle_yaw = yaw_angle;

    // Compensation yaw - DESACTIVER POUR TEST
    #if 0  // <-- Mettre 1 pour réactiver
    const float yaw_rad = gyro_yaw_dps * dt_s * (PI / 180.0f);
    const float s = sinf(yaw_rad);

    const float roll0  = drone->angle_roll;
    const float pitch0 = drone->angle_pitch;

    drone->angle_pitch = pitch0 - roll0  * s;
    drone->angle_roll  = roll0  + pitch0 * s;
    #endif
}

// ==================== TASK IMU ====================
static void imu_task(void *parameter) {
    (void)parameter;

    memset(&imu_state, 0, sizeof(imu_state));
    imu_state.current_mode = MODE_SAFE;
    imu_state.channel_3 = 1000;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        FlightMode m;
        int ch3;
        bool do_reset = false;

        portENTER_CRITICAL(&imu_mux);
        m = imu_in_mode;
        ch3 = imu_in_ch3;
        if (imu_reset_req) { imu_reset_req = false; do_reset = true; }
        portEXIT_CRITICAL(&imu_mux);

        imu_state.current_mode = m;
        imu_state.channel_3 = ch3;

        if (do_reset) {
            imu_state.angle_pitch = 0.0f;
            imu_state.angle_roll  = 0.0f;
            imu_state.gyro_roll_input = 0.0f;
            imu_state.gyro_pitch_input = 0.0f;
            imu_state.gyro_yaw_input = 0.0f;
            
            kalman_roll.setAngle(0.0f);
            kalman_pitch.setAngle(0.0f);
            kalman_initialized = false;
            yaw_angle = 0.0f;  // AJOUT: reset angle yaw
            
            gyro_roll_filt = 0.0f;
            gyro_pitch_filt = 0.0f;
            gyro_yaw_filt = 0.0f;
        }

        unsigned long t0 = micros();
        imu_read_internal(&imu_state);
        unsigned long dur = micros() - t0;

        bool ok = (imu_state.max_time_imu != 888888);

        portENTER_CRITICAL(&imu_mux);
        imu_snap.gyro_roll_input  = imu_state.gyro_roll_input;
        imu_snap.gyro_pitch_input = imu_state.gyro_pitch_input;
        imu_snap.gyro_yaw_input   = imu_state.gyro_yaw_input;
        imu_snap.angle_roll       = imu_state.angle_roll;
        imu_snap.angle_pitch      = imu_state.angle_pitch;
        imu_snap.angle_yaw        = imu_state.angle_yaw;  // AJOUT
        imu_snap.acc_total_vector = imu_state.acc_total_vector;
        imu_snap.last_dur_us      = dur;
        imu_snap.ok               = ok;
        if (ok) imu_snap.last_ok_ms = millis();
        portEXIT_CRITICAL(&imu_mux);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(4)); // 250 Hz
    }
}

// ==================== API PUBLIQUE ====================
void imu_start_task() {
    if (imu_task_handle != nullptr) return;

    xTaskCreatePinnedToCore(
        imu_task,
        "imu_i2c",
        4096,
        nullptr,
        4,
        &imu_task_handle,
        0
    );
}

void imu_update(DroneState *drone) {
    static FlightMode last_mode = MODE_SAFE;

    ImuSnapshot s;

    portENTER_CRITICAL(&imu_mux);
    imu_in_mode = drone->current_mode;
    imu_in_ch3  = drone->channel_3;
    s = imu_snap;
    portEXIT_CRITICAL(&imu_mux);

    if ((drone->current_mode != last_mode) &&
        (drone->current_mode == MODE_SAFE || drone->current_mode == MODE_ARMED)) {
        imu_request_reset();
    }
    last_mode = drone->current_mode;

    drone->gyro_roll_input  = s.gyro_roll_input;
    drone->gyro_pitch_input = s.gyro_pitch_input;
    drone->gyro_yaw_input   = s.gyro_yaw_input;
    drone->angle_roll       = s.angle_roll;
    drone->angle_pitch      = s.angle_pitch;
    drone->angle_yaw        = s.angle_yaw;  // AJOUT
    drone->acc_total_vector = s.acc_total_vector;
    drone->current_time_imu = s.last_dur_us;
}

void imu_request_reset() {
    portENTER_CRITICAL(&imu_mux);
    imu_reset_req = true;
    portEXIT_CRITICAL(&imu_mux);
}

void imu_read(DroneState *drone) {
    imu_read_internal(drone);
}