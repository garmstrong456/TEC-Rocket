#include "I2Cdev.h"
#include "MPU6050.h"

#include <SPI.h>
#include <SD.h>

#include <PololuLedStrip.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <Adafruit_BMP085.h>

//BMP Variables
Adafruit_BMP085 altimeter;
float temp, alt;
int32_t pressure;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//MPU6050 Variables
MPU6050 accelgyro;
int16_t ax, ay, az;   //the raw acc values for each axis
int16_t gx, gy, gz;   //the raw rotation rate values for each axis

//SD card variables
#define CS 2

//RBF Variables
#define RBF 4

//LED Variables
#define LED_PIN 3
#define NUMPIXELS 1
PololuLedStrip<LED_PIN> LED;
rgb_color LEDcolor[NUMPIXELS];

// Converts a color from HSV to RGB.
// h is hue, as a number between 0 and 360.
// s is the saturation, as a number between 0 and 255.
// v is the value, as a number between 0 and 255.
rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v)
{
    uint8_t f = (h % 60) * 255 / 60;
    uint8_t p = (255 - s) * (uint16_t)v / 255;
    uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t r = 0, g = 0, b = 0;
    switch((h / 60) % 6){
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
    return (rgb_color){r, g, b};
}

void setup()
{
  //Initialize I2C communication
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif  
    
    Serial.begin(9600);
    
    //Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    accelgyro.initialize();
    Serial.print("Testing MPU6050 connection...");
    if (accelgyro.testConnection()) {
      Serial.println("success!");
    } else {
      Serial.println("fail :(");
      while(1);
    }
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   //set the range to +/-2g
                                                            //other possible ranges:
                                                            //MPU6050_ACCEL_FS_2     +/-2g
                                                            //MPU6050_ACCEL_FS_4     +/-4g
                                                            //MPU6050_ACCEL_FS_8     +/-8g
                                                            //MPU6050_ACCEL_FS_16    +/-16g

    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   //set the range to +/-2g
                                                            //other possible ranges:
                                                            //MPU6050_GYRO_FS_250     +/-250deg/s
                                                            //MPU6050_GYRO_FS_500     +/-500deg/s
                                                            //MPU6050_GYRO_FS_1000    +/-1000deg/s
                                                            //MPU6050_GYRO_FS_2000    +/-2000deg/s


    //Initialize BMP
    Serial.println("Initializing BMP280...");
    if (altimeter.begin()) {
      Serial.println("success!");
    } else {
      Serial.println("fail :(");
      while(1);
    } 

    //Initialize SD card
    pinMode(CS, OUTPUT);
    Serial.println("Initializing SD card...");
    if (SD.begin(CS)) {
      Serial.println("success!");
    } else {
      Serial.println("fail :(");
      while(1);
    }

    //Test SD card write
    Serial.println("Testing SD card write...");
    File dataFile = SD.open("testfile.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println("test text");
      dataFile.close();
      Serial.println("success!");
    } else {
      Serial.println("fail :(");
      while(1);
    }
}

int hue = 0;

void loop()
{
  //read accelerometer and gyro values
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print(  "ax:\t"); Serial.print(ax);
  Serial.print("\tay:\t"); Serial.print(ay);
  Serial.print("\taz:\t"); Serial.print(az);
  Serial.print("\tgx:\t"); Serial.print(gx);
  Serial.print("\tgy:\t"); Serial.print(gy);
  Serial.print("\tgz:\t"); Serial.print(gz);
  Serial.print("\thue:\t"); Serial.print(hue);
  Serial.println();

  //read temperature pressure and altitude
  temp = altimeter.readTemperature();
  pressure = altimeter.readPressure();
  alt = altimeter.readAltitude();

  hue = map(ay, -16384, 16384, 0, 360);
  LEDcolor[0] = hsvToRgb(hue, 255, 255);
  LED.write(LEDcolor, NUMPIXELS);
}

