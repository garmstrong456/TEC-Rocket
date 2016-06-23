/*
TEC Rocket v2.3 flight code by Greg Armstrong <garmstrong456@gmail.com>
May 2016

License: CC BY-SA 4.0

****IMPORTANT*****
This code has not been flight tested. Use at your own risk.
******************

*/

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
File dataFile;

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

//This function runs when any of the initialization processes fail
//it will make the LED blink red indefinitely
//stopNum can be used to change the LED pattern based on where the init failed (not implemented)
void initFail(int stopNum) {
  while(1) {
    setLED(255, 0, 0);
    delay(300);
    setLED(0, 0, 0);
    delay(300);
  }
}

//Sets the LED to a particular color using RGB colors
void setLED(int red, int green, int blue) {
  //Note that red and green are reversed
  //For some reason the Pololu library doesn't take in to account that the LEDs are
  //GRB rather than RGB
  LEDcolor[0].red = green;
  LEDcolor[0].green = red;
  LEDcolor[0].blue = blue;
  LED.write(LEDcolor, NUMPIXELS);
}

void setup() {
  //Set the LED color to light blue to indicate the initialization process has started
  setLED(0, 255, 255);
  delay(500);
  
  //Set the RBF jumper to be a digital input with the internal pullup engaged
  pinMode(RBF, INPUT_PULLUP);
  
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
      setLED(0, 255, 0);            //blink the LED green briefly to indicate success
      delay(200);
      setLED(0, 255, 255);
      delay(200);
    } else {
      Serial.println("fail :(");
      initFail(0);
    }
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);   //set the range to +/-8g
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
      setLED(0, 255, 0);            //blink the LED green briefly to indicate success
      delay(200);
      setLED(0, 255, 255);
      delay(200);
    } else {
      Serial.println("fail :(");
      initFail(1);
    } 
    

    //Initialize SD card
    pinMode(CS, OUTPUT);
    Serial.println("Initializing SD card...");
    if (SD.begin(CS)) {
      Serial.println("success!");
      setLED(0, 255, 0);            //blink the LED green briefly to indicate success
      delay(200);
      setLED(0, 255, 255);
      delay(200);
    } else {
      Serial.println("fail :(");
      initFail(2);
    }

    //Opening the datafile
    Serial.println("Opening datalog.csv on the SD card...");
    dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.close();
      Serial.println("success!");
      setLED(0, 255, 0);            //blink the LED green briefly to indicate success
      delay(200);
      setLED(0, 255, 255);
      delay(200);
    } else {
      Serial.println("fail :(");
      initFail(3);
    }

  //If we make it to this point all initializations were successful!
  //Turn the LED to solid green to indicate success
  delay(200);
  setLED(0, 255, 0);

  //wait for the RBF Jumper to be pulled before continuing
  while (digitalRead(RBF) == LOW) {}  //wait while the RBF jumper is present
  dataFile.println("Starting new launch data");
  dataFile.println("time (ms), ax, ay, az, gx, gy, gz, temperature, pressure, altitude");

  //Blink the LED to indicate datalogging has started
  for(int i = 0; i < 4; i++) {
    setLED(0, 0, 255);
    delay(200);
    setLED(0, 0, 0);
    delay(200);
  }
}

//used in one of the LED routines below
int maxAcc = -32768;
int hue = 0;

void loop() {

  //RBF jumper has been pulled, start recording flight data

  String dataString = "";
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //temp = altimeter.readTemperature();
  //pressure = altimeter.readPressure();
  //alt = altimeter.readAltitude();

  dataString += String(millis()) + ",";
  dataString += String(ax) + ",";
  dataString += String(ay) + ",";
  dataString += String(az) + ",";
  dataString += String(gx) + ",";
  dataString += String(gy) + ",";
  dataString += String(gz) + ",";
  dataString += String(temp) + ",";
  dataString += String(pressure) + ",";
  dataString += String(alt) + ",";

  //This code opens and closes the datafile every time a write operation is preformed
  //This guarantees that all data will be recorded, but reduces performance (time between data readings will be larger)
  //to increase performance at the cost of reliability, open the file in setup (first line) and delete the dataFile.close() line
  dataFile = SD.open("datalog.csv", FILE_WRITE);
  dataFile.println(dataString);
  dataFile.close();
  
  //Do something fun with the LED

  //This will set the hue of the LED based on the acceleration in the y direction
  //this assumes the scale is set to +/-8g
  hue = map(ay, -4096, 4096, 0, 360);
  LEDcolor[0] = hsvToRgb(hue, 255, 255);
  LED.write(LEDcolor, NUMPIXELS);

  /*
  //Set the LED based on the maximum acceleration measured
  //The LED will fade from green to red as the max measured vertical acceleration goes up
  if (ay > maxAcc) {maxAcc = ay;}
  int red = map(maxAcc, -32768, 32767, 0, 255);
  int green = map(maxAcc, -32768, 32767, 255, 0);
  setLED(red, green, 0);
  */ 
   
}
