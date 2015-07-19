#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "SPI.h"
#include <Adafruit_10DOF.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
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

float seaLevelPressure = 1025.5;

float lastAltitude = 0;
long lastDist      = 0;

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

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
  firstText();
}


void loop(void) {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude    */
    float altitude = bmp.pressureToAltitude(seaLevelPressure,
                                            bmp_event.pressure,
                                            temperature) * 3.2808399;

    printAltitude(altitude);
    // Serial.print(F(" m; "));
    // /* Display the temperature */
    // Serial.print(F("Temp: "));
    // Serial.print(temperature);
    // Serial.print(F(" C"));
  }

  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance >= 200 || distance <= 0){
    Serial.println("Out of range");
  }
  else {
    printRange(distance);
  }
  delay(250);
}

unsigned long firstText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(120, 0);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println("Ranger");
  return micros() - start;
}

void printRange(long dist) {
  if (lastDist != dist) {
    tft.setCursor(0, 60);
    tft.setTextColor(ILI9341_BLACK);    tft.setTextSize(6);
    tft.println(lastDist);
    tft.setCursor(0, 60);
    tft.setTextColor(ILI9341_RED);    tft.setTextSize(6);
    tft.println(dist);
    lastDist = dist;
  }
}

void printAltitude(float altitude) {
  if (lastAltitude != altitude) {
    tft.setCursor(0, 120);
    tft.setTextColor(ILI9341_BLACK);    tft.setTextSize(6);
    tft.println(lastAltitude);
    tft.setCursor(0, 120);
    tft.setTextColor(ILI9341_RED);    tft.setTextSize(6);
    tft.println(altitude);
    lastAltitude = altitude;
  }
}
