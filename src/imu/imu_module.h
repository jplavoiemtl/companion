#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include "SensorQMI8658.hpp"
#include "calibration.h"

// Exposed IMU state (used by UI/inclinometer display)
extern bool g_isCurrentlyMoving;
extern float pitch_angle;
extern float roll_angle;
extern bool inclinometer_initialized;
extern const unsigned long IMU_PRINT_INTERVAL;
extern unsigned long lastImuPrintTime;

// IMU public API (signatures unchanged from companion.ino)
void initIMU();
void reinitializeMotionBaseline();
void updateImuData();
void updateMotionState();
void updateInclinometer(float gyro_vert, float gyro_horiz, float gyro_up);
void readImuData();
float imuGetAccelInertialVert();
float imuGetAccelInertialHoriz();

// External dependencies provided by companion.ino
extern SensorQMI8658 qmi;
extern PubSubClient mqttClient;
extern HWCDC USBSerial;
extern SemaphoreHandle_t i2c_mutex;
extern const char MOTION_TOPIC[];
extern const char IMU_TOPIC[];
extern const unsigned long MOTION_TIMEOUT;

