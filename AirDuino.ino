#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "SPI.h"
#include <Adafruit_10DOF.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Wire.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10
#define trigPin 30
#define echoPin 32

// TODO: Allow for SLP entry
float seaLevelPressure = 1028.4;

int lastAltitude = 0;
float lastTemp   = 0;
long lastDist    = 0;
int lastHeading  = 0;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

void initSensors()
{
  if(!accel.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin()) {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

void setup() {
  tft.begin();
  tft.setRotation(1);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  initSensors();
  clearDisplay();
}


void loop(void) {
  sensors_event_t accel_event;

  printAltitude(getAltitude());
  printTemp(getTemperature());
  printRange(getRange());
  printHeading(getHeading());

  delay(250);
}


/*
 * Inputs
 */

// Get altitude in feet
int getAltitude() {
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure) {
    // Convert atmospheric pressure, SLP and temp to altitude, then convert to feet
    int altitude = round(
      bmp.pressureToAltitude(
        seaLevelPressure,
        bmp_event.pressure,
        getTemperature()) * 3.2808399);

    return altitude;
  } else {
    return lastAltitude;
  }
}

// Get heading
int getHeading() {
  sensors_event_t mag_event;
  sensors_vec_t orientation;
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
    return (int) orientation.heading;
  } else {
    return lastHeading;
  }
}

// Get ranger distance
int getRange() {
  long duration;
  long distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance >= 200 || distance <= 0){
    return (int) -1;
  }
  else {
    return (int) distance;
  }
}

// Get temperature
float getTemperature() {
  float temperature;
  bmp.getTemperature(&temperature);

  return temperature;
}


/*
 * Outputs
 */

// Fill the screen black
void clearDisplay() {
  tft.fillScreen(ILI9341_BLACK);
}

void printRange(int dist) {
  if (lastDist != dist) {
    tft.setCursor(0, 0);
    tft.setTextColor(ILI9341_BLACK);    tft.setTextSize(6);
    tft.println(lastDist);
    tft.setCursor(0, 0);
    tft.setTextColor(ILI9341_RED);    tft.setTextSize(6);
    tft.println(dist);
    lastDist = dist;
  }
}

void printAltitude(int altitude) {
  if (lastAltitude != altitude) {
    tft.setCursor(0, 60);
    tft.setTextColor(ILI9341_BLACK);    tft.setTextSize(6);
    tft.println(lastAltitude);
    tft.setCursor(0, 60);
    tft.setTextColor(ILI9341_RED);    tft.setTextSize(6);
    tft.println(altitude);
    lastAltitude = altitude;
  }
}

void printHeading(int heading) {
  if (lastHeading != heading) {
    tft.setCursor(0, 180);
    tft.setTextColor(ILI9341_BLACK);    tft.setTextSize(6);
    tft.println(lastHeading);
    tft.setCursor(0, 180);
    tft.setTextColor(ILI9341_RED);    tft.setTextSize(6);
    tft.println(heading);
    lastHeading = heading;
  }
}

void printTemp(float temp) {
  if (lastTemp != temp) {
    tft.setCursor(0, 120);
    tft.setTextColor(ILI9341_BLACK);    tft.setTextSize(6);
    tft.println(lastTemp);
    tft.setCursor(0, 120);
    tft.setTextColor(ILI9341_RED);    tft.setTextSize(6);
    tft.println((int) temp);
    lastTemp = temp;
  }
}
