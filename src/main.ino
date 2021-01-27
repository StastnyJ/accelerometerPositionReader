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
#define SIMULATION_LED_PIN 2

#define BTN_CLICK_IDLE 200
#define CALIBRATION_RECORDS_COUNT 1000
#define ACC_VALUES_BUFFER_SIZE 512
#define SMOOTHING_BLOCK_SIZE 16
#define GRAVITY 9.807
#define ASSUME_ZERO_LIMIT 0.03125

#define D if (0)

struct AGValues {
    struct Vector3D Acc;
    float GyX, GyY, GyZ;
    unsigned long time;
};

struct Location {
    double lon, lat;
    unsigned long time;
    int valid;
};

struct AccelerationRecord {
    struct Vector3D acceleration;
    unsigned long time;
};

TinyGPSPlus gps;
SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);

int calibrateBtnPressed = 0;
int simulateNoGpsBtnPressed = 0;

int noGPSSignal = 0;
int simulateNOGPSSignal = 0;

struct PolarVector orientation;
float gyXError = 0.0f;
float gyYError = 0.0f;
float gyZError = 0.0f;

struct Vector3D speed;
unsigned long lastCalculationTime = 0;
struct Location lastLocation;

struct AccelerationRecord* accelerationBuffer;
int accelerationValuesStored;

char* buffer128;

unsigned long lastRecordTime = 0;

float assumeAsZero(float x) {
    return (x < ASSUME_ZERO_LIMIT && x > -ASSUME_ZERO_LIMIT) ? 0 : x;
}

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

float extractX(struct AccelerationRecord r) { return r.acceleration.x; }
float extractY(struct AccelerationRecord r) { return r.acceleration.y; }
float extractZ(struct AccelerationRecord r) { return r.acceleration.z; }

void swap(struct AccelerationRecord* v1, struct AccelerationRecord* v2) {
    struct AccelerationRecord tmp;
    tmp = *v1;
    *v1 = *v2;
    *v2 = tmp;
}

void sort(struct AccelerationRecord* vs, int length,
          float (*keyExtractor)(struct AccelerationRecord v)) {
    for (int i = 0; i < length - 1; i++) {
        for (int j = 0; j < length - i - 1; j++) {
            if (keyExtractor(vs[j]) > keyExtractor(vs[j + 1]))
                swap(&vs[j], &vs[j + 1]);
        }
    }
}

float getMedian(struct AccelerationRecord* vs, int length,
                float (*keyExtractor)(struct AccelerationRecord v)) {
    sort(vs, length, keyExtractor);
    return (keyExtractor(vs[length / 2 - 1]) + keyExtractor(vs[length / 2])) /
           2.0;
}

void smoothData() {
    for (int i = 0; i < ACC_VALUES_BUFFER_SIZE; i += SMOOTHING_BLOCK_SIZE) {
        float medX =
            getMedian(accelerationBuffer + i, SMOOTHING_BLOCK_SIZE, extractX);
        float medY =
            getMedian(accelerationBuffer + i, SMOOTHING_BLOCK_SIZE, extractY);
        float medZ =
            getMedian(accelerationBuffer + i, SMOOTHING_BLOCK_SIZE, extractZ);
        for (int j = 0; j < SMOOTHING_BLOCK_SIZE; j++)
            accelerationBuffer[i + j].acceleration = {
                .x = medX, .y = medY, .z = medZ};
    }
}

float meterToLat(float x) { return x / 111111; }

float meterToLon(float x, float lat) {
    return x / (111111 * cos(lat * PI / 180.0));
}

void prepareAccelerationForIntegration() {
    for (int i = 0; i < ACC_VALUES_BUFFER_SIZE; i++) {
        accelerationBuffer[i].acceleration.x =
            GRAVITY * assumeAsZero(accelerationBuffer[i].acceleration.x);
        accelerationBuffer[i].acceleration.y =
            GRAVITY * assumeAsZero(accelerationBuffer[i].acceleration.y);

        D Serial.printf("%f,%f\n", accelerationBuffer[i].acceleration.x,
                        accelerationBuffer[i].acceleration.y);
    }
}

void integrateAccelerationBuffer(float x0, float y0) {
    unsigned long lastTime = lastCalculationTime;
    if (lastTime == 0) lastTime = accelerationBuffer[0].time;
    float lastX = x0;
    float lastY = y0;
    for (int i = 0; i < ACC_VALUES_BUFFER_SIZE; i++) {
        accelerationBuffer[i].acceleration.x =
            lastX +
            accelerationBuffer[i].acceleration.x *
                (float)((long)accelerationBuffer[i].time - (long)lastTime) /
                1000.0;
        accelerationBuffer[i].acceleration.y =
            lastY +
            accelerationBuffer[i].acceleration.y *
                (float)((long)accelerationBuffer[i].time - (long)lastTime) /
                1000.0;
        lastX = accelerationBuffer[i].acceleration.x;
        lastY = accelerationBuffer[i].acceleration.y;
        lastTime = accelerationBuffer[i].time;
    }
}

void recalculateSpeed() {
    prepareAccelerationForIntegration();
    integrateAccelerationBuffer(speed.x, speed.y);
    speed.x = accelerationBuffer[ACC_VALUES_BUFFER_SIZE - 1].acceleration.x;
    speed.y = accelerationBuffer[ACC_VALUES_BUFFER_SIZE - 1].acceleration.y;
}

void recalculatePosition() {
    integrateAccelerationBuffer(0, 0);
    lastLocation.valid = 1;
    lastLocation.time = accelerationBuffer[ACC_VALUES_BUFFER_SIZE - 1].time;
    lastLocation.lat += meterToLat(
        accelerationBuffer[ACC_VALUES_BUFFER_SIZE - 1].acceleration.y);
    lastLocation.lon += meterToLon(
        accelerationBuffer[ACC_VALUES_BUFFER_SIZE - 1].acceleration.x,
        lastLocation.lat);
}

void logAGRecord(struct AGValues agData) {
    accelerationBuffer[accelerationValuesStored].time = agData.time;
    accelerationBuffer[accelerationValuesStored++].acceleration =
        toEarthCoords(agData.Acc);
    if (accelerationValuesStored == ACC_VALUES_BUFFER_SIZE) {
        smoothData();
        recalculateSpeed();
        if (noGPSSignal) recalculatePosition();
        accelerationValuesStored = 0;
        lastCalculationTime =
            accelerationBuffer[ACC_VALUES_BUFFER_SIZE - 1].time;
    }
}

void calibrateDevice() {
    delay(1000);

    struct AGValues* averages =
        (struct AGValues*)malloc(sizeof(struct AGValues));

    averages->Acc = NULL_VECTOR3D;
    averages->GyX = 0;
    averages->GyY = 0;
    averages->GyZ = 0;

    gyXError = 0;
    gyYError = 0;
    gyZError = 0;

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
    speed = {.x = 0, .y = 0, .z = 0};
    lastCalculationTime = 0;

    free(averages);
}

void handleCalibrateInputClick() {
    display.clear();
    display.drawString(0, 12, "Calibration in progress");
    display.drawString(0, 42, "Do not move with the device");
    display.display();
    calibrateDevice();
}

void handleSimulateNOGPClick() {
    simulateNOGPSSignal = (simulateNOGPSSignal + 1) % 2;
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

struct Location readGPSData() {
    struct Location res;
    while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
            if (gps.location.isValid() && !simulateNOGPSSignal) {
                res.lat = gps.location.lat();
                res.lon = gps.location.lng();
                res.valid = 1;
                res.time = millis();
                noGPSSignal = 0;
            } else {
                noGPSSignal = 1;
            }
        }
    }
    return res;
}

void displayData() {
    display.clear();

    if (noGPSSignal)
        display.drawString(4, 4, "NO GPS, approx. loc.");
    else
        display.drawString(4, 4, "Your position");

    sprintf(buffer128, "Lat: %f°", lastLocation.lat);
    display.drawString(4, 16, buffer128);
    sprintf(buffer128, "Lon %f°", lastLocation.lon);
    display.drawString(4, 28, buffer128);

    // D Serial.printf("Speed: [%f, %f]\n", speed.x, speed.y);
    sprintf(buffer128, "Speed: %.3f m/s", absoluteValue(speed));
    display.drawString(4, 44, buffer128);

    display.display();
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
    pinMode(SIMULATION_LED_PIN, OUTPUT);

    speed = {.x = 0, .y = 0, .z = 0};
    lastLocation = {.lon = 0, .lat = 0, .time = 0, .valid = 0};
    buffer128 = (char*)malloc(sizeof(char) * 128);

    accelerationBuffer = (struct AccelerationRecord*)malloc(
        sizeof(AGValues) * ACC_VALUES_BUFFER_SIZE);

    D Serial.println(F("Board initialized"));
    D Serial.println();
}

void loop() {
    handleBtnInput(CALIBRATE_INPUT, &calibrateBtnPressed,
                   &handleCalibrateInputClick);
    handleBtnInput(SIMULATE_NO_GPS, &simulateNoGpsBtnPressed,
                   &handleSimulateNOGPClick);
    if (simulateNOGPSSignal)
        digitalWrite(SIMULATION_LED_PIN, HIGH);
    else
        digitalWrite(SIMULATION_LED_PIN, LOW);

    struct Location location = readGPSData();
    struct AGValues agData = readAGData();

    logAGRecord(agData);

    if (noGPSSignal) {
    } else {
        lastLocation = location;
        delay(1);
    }

    if (lastRecordTime > 0) updateOrientation(agData);
    lastRecordTime = agData.time;

    displayData();
}
