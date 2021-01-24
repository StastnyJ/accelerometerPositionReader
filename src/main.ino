#include <TinyGPS++.h>
#include <Wire.h>

#include "SSD1306Wire.h"
#include "boards.h"
#include "utilities.h"

#define ACCELEROMETER_SENSITIVITY 16384

const int MPU = 0x68;

TinyGPSPlus gps;
SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);

void displayGPSInfo();

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

    Serial.println(F("Board initialized"));
    Serial.println();
}

void loop() {
    while (Serial1.available() > 0)
        if (gps.encode(Serial1.read())) displayGPSInfo();

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 12, true);
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();

    float AccX = (float)AcX / ACCELEROMETER_SENSITIVITY;
    float AccY = (float)AcY / ACCELEROMETER_SENSITIVITY;
    float AccZ = (float)AcZ / ACCELEROMETER_SENSITIVITY;

    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();
    int16_t GyZ = Wire.read() << 8 | Wire.read();

    // display.clear();
    char* buffer = (char*)malloc(32 * sizeof(buffer));
    sprintf(buffer, "A: %.3f, %.3f, %f", AccX, AccY, AccZ);
    Serial.println(buffer);
    display.drawString(0, 16, buffer);
    sprintf(buffer, "GY: %.3f, %.3f, %f", (float)GyX / 131, (float)GyY / 131,
            (float)GyZ / 131);
    display.drawString(0, 32, buffer);
    display.display();
    free(buffer);

    delay(333);
}

void displayGPSInfo() {
    display.clear();
    char* buffer = (char*)malloc(32 * sizeof(buffer));
    if (gps.location.isValid())
        sprintf(buffer, "G: %f, %f", gps.location.lat(), gps.location.lng());
    else
        sprintf(buffer, "Invalid location");
    display.drawString(0, 0, buffer);
    // display.display();
    free(buffer);
}
