#include <TinyGPS++.h>
#include <Wire.h>
#include <math.h>

#include "SSD1306Wire.h"
#include "boards.h"
#include "utilities.h"
#include "vectors.h"

#define ACCELEROMETER_SENSITIVITY 16384
#define GYROSCOPE_SENSITIVITY 131

#define MPU 0x68

#define CALIBRATE_INPUT 14
#define SIMULATE_NO_GPS 13

#define BTN_CLICK_IDLE 200
#define CALIBRATION_RECORDS_COUNT 1000

#define D if (1)

struct AGValues {
    struct Vector3D Acc;
    float GyX, GyY, GyZ;
    unsigned long time;
};

TinyGPSPlus gps;
SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);

int calibrateBtnPressed = 0;
int simulateNoGpsBtnPressed = 0;

struct PolarVector orientation;
float gyXError = 0.0f;
float gyYError = 0.0f;
float gyZError = 0.0f;

unsigned long lastRecordTime = 0;

struct Vector3D toEarthCoords(struct Vector3D relativeCoords) {
    return toCartesian(rotateBy(toPolar(relativeCoords), orientation));
}

struct AGValues readAGData() {
    struct AGValues res;
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 12, true);
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();
    int16_t GyZ = Wire.read() << 8 | Wire.read();

    res.Acc.x = (float)AcX / ACCELEROMETER_SENSITIVITY;
    res.Acc.y = (float)AcY / ACCELEROMETER_SENSITIVITY;
    res.Acc.z = (float)AcZ / ACCELEROMETER_SENSITIVITY;

    res.GyX = (float)GyX * PI / (GYROSCOPE_SENSITIVITY * 90) - gyXError;
    res.GyY = (float)GyY * PI / (GYROSCOPE_SENSITIVITY * 90) - gyYError;
    res.GyZ = (float)GyZ * PI / (GYROSCOPE_SENSITIVITY * 90) - gyZError;

    res.time = millis();

    return res;
}

void updateOrientation(struct AGValues measurement) {
    unsigned long elapsedTime = measurement.time - lastRecordTime;
    orientation.alpha += measurement.GyZ * (float)elapsedTime / 1000;
    orientation.beta += measurement.GyY * (float)elapsedTime / 1000;
    orientation.gamma += measurement.GyX * (float)elapsedTime / 1000;

    while (orientation.alpha > PI) orientation.alpha -= 2 * PI;
    while (orientation.alpha < -PI) orientation.alpha += 2 * PI;
    while (orientation.beta > PI) orientation.beta -= 2 * PI;
    while (orientation.beta < -PI) orientation.beta += 2 * PI;
    while (orientation.gamma > PI) orientation.gamma -= 2 * PI;
    while (orientation.gamma < -PI) orientation.gamma += 2 * PI;
}

void calibrateDevice() {
    struct AGValues* averages =
        (struct AGValues*)malloc(sizeof(struct AGValues));

    averages->Acc = NULL_VECTOR3D;
    averages->GyX = 0;
    averages->GyY = 0;
    averages->GyZ = 0;

    int records = 0;
    while (records < CALIBRATION_RECORDS_COUNT) {
        struct AGValues agValues = readAGData();
        averages->Acc.x += agValues.Acc.x;
        averages->Acc.y += agValues.Acc.y;
        averages->Acc.z += agValues.Acc.z;
        averages->GyX += agValues.GyX;
        averages->GyY += agValues.GyY;
        averages->GyZ += agValues.GyZ;
        records++;
        delay(10);
    }
    averages->Acc.x /= records;
    averages->Acc.y /= records;
    averages->Acc.z /= records;
    averages->GyX /= records;
    averages->GyY /= records;
    averages->GyZ /= records;

    struct PolarVector expectedVector = DEFAULT_CALM_VECTOR;

    orientation = getAngleDiff(toPolar(averages->Acc), expectedVector);
    gyXError = averages->GyX;
    gyYError = averages->GyY;
    gyZError = averages->GyZ;

    D Serial.println("CALIBRATION COMPLEATED");
    D Serial.println(orientation.alpha);
    D Serial.println(orientation.beta);
    D Serial.println(orientation.gamma);

    lastRecordTime = 0;

    free(averages);
}

void handleCalibrateInputClick() {
    display.clear();
    display.drawString(0, 12, "Calibration in progress");
    display.drawString(0, 42, "Do not move with the device");
    display.display();
    calibrateDevice();
}

void handleBtnInput(int pin, int* isPressed, void (*onClickHandler)()) {
    if (digitalRead(pin) == HIGH) {
        if (*isPressed <= 0 && -1 * (*isPressed) < millis() - BTN_CLICK_IDLE) {
            *isPressed = millis();
            onClickHandler();
        }
    } else {
        if (*isPressed > 0 && *isPressed < millis() - BTN_CLICK_IDLE)
            *isPressed = -1 * millis();
    }
}

void displayGPSInfo() {
    display.clear();
    char* buffer = (char*)malloc(32 * sizeof(buffer));
    if (gps.location.isValid())
        sprintf(buffer, "G: %f, %f", gps.location.lat(), gps.location.lng());
    else
        sprintf(buffer, "Invalid location");
    display.drawString(0, 0, buffer);
    free(buffer);
}

void setup() {
    initBoard();
    delay(1500);

    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    pinMode(CALIBRATE_INPUT, INPUT);
    pinMode(SIMULATE_NO_GPS, INPUT);

    D Serial.println(F("Board initialized"));
    D Serial.println();
}

void loop() {
    handleBtnInput(CALIBRATE_INPUT, &calibrateBtnPressed,
                   &handleCalibrateInputClick);

    while (Serial1.available() > 0)
        if (gps.encode(Serial1.read())) displayGPSInfo();

    struct AGValues agData = readAGData();
    struct Vector3D accRes = toEarthCoords(agData.Acc);

    if (lastRecordTime > 0) updateOrientation(agData);
    lastRecordTime = agData.time;

    // D Serial.printf("%f,%f,%f\n", orientation.alpha, orientation.beta,
    //                 orientation.gamma);
    D Serial.printf("%f,%f,%f\n", accRes.x, accRes.y, accRes.z);

    display.display();
}
