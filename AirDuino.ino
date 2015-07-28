#include <Adafruit_10DOF.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Button.h>
#include <DS3232RTC.h>
#include <math.h>
#include <SD.h>
#include <SPI.h>
#include <Time.h>
#include <Ultrasonic.h>
#include <Wire.h>

// Pin declarations
#define echoPin 23
#define SD_CS   4
#define TFT_CS  10
#define TFT_DC  9
#define trigPin 22
#define viewPin 24

// Calibration values
double rangerCalib = 0.76755776;
float qnh          = 1019.2;
int echoTime       = 23529;
int rangerOffset   = 0;

// Hardware declarations
Adafruit_10DOF dof                  = Adafruit_10DOF();
Adafruit_BMP085_Unified bmp         = Adafruit_BMP085_Unified(18001);
Adafruit_ILI9341 tft                = Adafruit_ILI9341(TFT_CS, TFT_DC);
Adafruit_L3GD20_Unified gyro        = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag     = Adafruit_LSM303_Mag_Unified(30302);

Button viewBtn = Button(viewPin, BUTTON_PULLUP_INTERNAL);
Ultrasonic ultrasonic(trigPin, echoPin, echoTime);

// Theming
uint16_t blue     = 0x01A8;
uint16_t cream    = 0xEF37;
uint16_t darkblue = 0x0126;
uint16_t orange   = 0xDBC4;
uint16_t red      = 0xA8C4;

bool viewReset = true;
int activeView = -1; //Button registers a press on power up for some reason

// Previous sensor values
double lastAccelX;
double lastAccelY;
double lastAccelZ;
double lastGyroX;
double lastGyroY;
double lastGyroZ;
double lastPressure;
double lastTemp;
int lastAltitude;
int lastHeading;
int lastPitch = 999;
int lastRange = 999;
int lastRoll = 999;
long totalTime;
time_t lastTime;

// Previous strings, to mask old values on TFT
char lastAccelXStr[6];
char lastAccelYStr[6];
char lastAccelZStr[6];
char lastGyroStr[20];
char lastPressureStr[7];
char lastTempStr[6];

File logFile;
char logName[11];

// Create a log filename based on index
void createLogName(int fileCount) {
  sprintf(logName, "log%d.csv", fileCount);
}

// Test the clock by setting the system time to RTC time
void initClock() {
  setSyncProvider(RTC.get);
  if(timeStatus() != timeSet) {
    errorMsg("Error syncing time");
  }
}

// Initialise SD and make log file
void initSD() {
  pinMode(TFT_CS, OUTPUT);
  if (!SD.begin(SD_CS)) {
    errorMsg("SD initialisation failed");
  }

  // Create initial file name
  int fileCount = 0;
  createLogName(fileCount);
  bool fileExists = true;

  // Increase file name index until new file is found
  while(fileExists) {
    if (!SD.exists(logName)) {
      logFile = SD.open(logName, FILE_WRITE);
      logFile.println("Time,Pitch,Roll,Heading,Acceleration X (m/s^2),Acceleration Y (m/s^2),Acceleration Z (m/s^2),Gyro X (rad/s),Gyro Y (rad/s),Gyro Z (rad/s),Altitude (ft),Range (cm),Pressure (hPa),Temperature (C)");
      logFile.close();

      fileExists = false;
    } else {
      fileCount++;
      createLogName(fileCount);
    }
  }

  if (!SD.exists(logName)) {
    errorMsg("Log file not created");
  }
}

// Initialise 10DOF Sensors
void init10DOF() {
  if(!accel.begin()) {
    errorMsg("No LSM303 detected");
  }
  if(!mag.begin()) {
    errorMsg("No LSM303 detected");
  }
  if(!bmp.begin()) {
    errorMsg("No BMP180 detected");
  }
  gyro.enableAutoRange(true);
  if(!gyro.begin()) {
    errorMsg("No L3GD20 detected");
  }
}

// Initialise Ranger
void initRanger() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// Initialise screen
void initScreen() {
  tft.begin();
  tft.setRotation(1);
  clearDisplay();
}

void setup() {
  initScreen();
  init10DOF();
  initClock();
  initRanger();
  initSD();
}


void loop(void) {
  // Get readings
  double rawTemperature = getTemperature();
  double rawPressure    = getPressure();
  double rawAccelX      = getAccel('x');
  double rawAccelY      = getAccel('y');
  double rawAccelZ      = getAccel('z');
  double rawGyroX       = getGyro('x');
  double rawGyroY       = getGyro('y');
  double rawGyroZ       = getGyro('z');
  int altitude          = getAltitude();
  int heading           = getHeading();
  int pitch             = getPitch();
  int range             = getRange();
  int roll              = getRoll();
  time_t timeNow        = RTC.get();

  // Format pressure to string
  char strPressure[7];
  dtostrf(rawPressure, 6, 1, strPressure);
  char *pressure = deblank(strPressure);

  // Format QNH to string
  char strQnh[7];
  dtostrf(qnh, 6, 1, strQnh);
  char *qnhStr = deblank(strQnh);

  // Format temperature to string
  char strTemperature[6];
  dtostrf(rawTemperature, 5, 1, strTemperature);
  char *temperature = deblank(strTemperature);

  // Format Gyro X to string
  char strGyroX[6];
  dtostrf(rawGyroX, 5, 2, strGyroX);
  char *gyroX = deblank(strGyroX);

  // Format Gyro Y to string
  char strGyroY[6];
  dtostrf(rawGyroY, 5, 2, strGyroY);
  char *gyroY = deblank(strGyroY);

  // Format Gyro Z to string
  char strGyroZ[6];
  dtostrf(rawGyroZ, 5, 2, strGyroZ);
  char *gyroZ = deblank(strGyroZ);

  // Format Acceleration X to string
  char strAccelX[7];
  dtostrf(rawAccelX, 6, 2, strAccelX);
  char *accelX = deblank(strAccelX);

  // Format Acceleration X to string
  char strAccelY[7];
  dtostrf(rawAccelY, 6, 2, strAccelY);
  char *accelY = deblank(strAccelY);

  // Format Acceleration X to string
  char strAccelZ[7];
  dtostrf(rawAccelZ, 6, 2, strAccelZ);
  char *accelZ = deblank(strAccelZ);


  // Log sensor data to SD card
  char logMsg[120];
  sprintf(
    logMsg, 
    "%02d/%02d/%d %02d:%02d:%02d,%d,%d,%d,%s,%s,%s,%s,%s,%s,%d,%d,%s,%s", 
    day(timeNow), 
    month(timeNow), 
    year(timeNow), 
    hour(timeNow), 
    minute(timeNow), 
    second(timeNow), 
    pitch, 
    roll, 
    heading, 
    accelX, 
    accelY, 
    accelZ, 
    gyroX, 
    gyroY, 
    gyroZ, 
    altitude, 
    range, 
    pressure, 
    temperature
  );

  logToCard(logMsg);

  // Check for view button press
  if (viewBtn.uniquePress()) {
    viewReset = true;
    if (activeView == 4) {
      activeView = 0;
    } else {
      activeView++;
    }
  } else {
    // Update/run the active view
    switch (activeView) {
      case 1:
        attitude(heading, qnhStr, altitude, pitch, roll);
        break;
      case 2:
        atmosphere(pressure, temperature, rawPressure, rawTemperature);
        break;
      case 3:
        accelerations(accelX, accelY, accelZ);
        break;
      case 4:
        clock(timeNow);
        break;
      default:
        ranger(range);
    }
  }

  // Update last values
  double lastTemp     = rawTemperature;
  double lastPressure = rawPressure;
  double lastAccelX   = rawAccelX;
  double lastAccelY   = rawAccelY;
  double lastAccelZ   = rawAccelZ;
  double lastGyroX    = rawGyroX;
  double lastGyroY    = rawGyroY;
  double lastGyroZ    = rawGyroZ;
  int lastAltitude    = altitude;
  int lastHeading     = heading;
  int lastPitch       = pitch;
  int lastRange       = range;
  int lastRoll        = roll;
}


/*
 * Sensor getters
 */

// Get altitude in feet
int getAltitude() {
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure) {
    // Convert atmospheric pressure, QNH and temp to altitude, then convert to feet
    int altitude = round(
      bmp.pressureToAltitude(
        qnh,
        bmp_event.pressure,
        getTemperature()) * 3.2808399);

    return altitude;
  } else {
    return lastAltitude;
  }
}

// Get heading
int getHeading() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t orientation;

  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  dof.magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
    return (int) orientation.heading;
  } else {
    return lastHeading;
  }
}

// Get ranger distance
int getRange() {
  return (int) ultrasonic.Ranging(CM) * rangerCalib + rangerOffset;
}

// Get roll
int getRoll() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t orientation;

  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
    return (int) orientation.pitch;
  } else {
    return lastRoll;
  }
}

// Get pitch
int getPitch() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t orientation;

  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
    return (int) orientation.roll;
  } else {
    return lastPitch;
  }
}

// Get acceleration
double getAccel(char axis) {
  sensors_event_t event; 
  accel.getEvent(&event);

  switch (axis) {
    case 'x':
      return (double) event.acceleration.x;
      break;
    case 'y':
      return (double) event.acceleration.y;
      break;
    default:
      return (double) event.acceleration.z;
  }
}

// Get rotational velocity
double getGyro(char axis) {
  sensors_event_t event; 
  gyro.getEvent(&event);

  switch (axis) {
    case 'x':
      return (double) event.gyro.x;
      break;
    case 'y':
      return (double) event.gyro.y;
      break;
    default:
      return (double) event.gyro.z;
  }
}

// Get pressure
double getPressure() {
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    return (double) event.pressure;
  } else {
    return lastPressure;
  }
}

// Get temperature
double getTemperature() {
  float temperature;
  bmp.getTemperature(&temperature);

  return (double) temperature;
}

/*
 * Views
 */

// Pressure and Temp
void atmosphere(char pressure[], char temperature[], double pressureVal, double temperatureVal) {
  // Setup
  if (viewReset) {
    newView(2);
    printData("Pressure", "", 10, 10, ILI9341_RED, 3);
    printData("Temperature", "", 10, 110, ILI9341_RED, 3);
    printData(pressure, "", 10, 40, ILI9341_RED, 4);
    printData(temperature, "", 10, 140, ILI9341_RED, 4);
    viewReset = false;
  }

  // Loop
  if ((int) round(lastPressure * 10) != (int) round(pressureVal * 10)) {
    printData(pressure, lastPressureStr, 10, 40, ILI9341_RED, 4);
    strcpy(lastPressureStr, pressure);
    lastPressure = pressureVal;
  }
  
  if ((int) round(lastTemp * 10) != (int) round(temperatureVal * 10)) {
    printData(temperature, lastTempStr, 10, 140, ILI9341_RED, 4);
    strcpy(lastTempStr, temperature);
    lastTemp = temperatureVal;
  }
}

// Heading, Pitch, Roll & Altitude
void attitude(int heading, char qnhStr[], int altitude, int pitch, int roll) {
  // Setup
  if (viewReset) {
    newView(1);
    printData("Pitch", "", 40, 100, ILI9341_RED, 3);
    printData("Roll", "", 40, 130, ILI9341_RED, 3);
    viewReset = false;
  }

  // Loop

  printInt(heading, &lastHeading, 125, 10, ILI9341_RED, 4);

  printData(qnhStr, "", 40, 70, ILI9341_RED, 3);
  printInt(altitude, &lastAltitude, 190, 70, ILI9341_RED, 3);
  printInt(pitch, &lastPitch, 190, 100, ILI9341_RED, 3);
  printInt(roll, &lastRoll, 190, 130, ILI9341_RED, 3);
}

// Accelerations
void accelerations(char accelX[], char accelY[], char accelZ[]) {
  // Setup
  if (viewReset) {
    newView(3);
    printData("X", "", 10, 10, ILI9341_RED, 2);
    printData("Y", "", 10, 70, ILI9341_RED, 2);
    printData("Z", "", 10, 130, ILI9341_RED, 2);
    viewReset = false;
  }
  printData(accelX, lastAccelXStr, 10, 30, ILI9341_RED, 3);
  printData(accelY, lastAccelYStr, 10, 90, ILI9341_RED, 3);
  printData(accelZ, lastAccelZStr, 10, 150, ILI9341_RED, 3);
  strcpy(lastAccelXStr, accelX);
  strcpy(lastAccelYStr, accelY);
  strcpy(lastAccelZStr, accelZ);
}

// Ultrasonic Range
void ranger(int range) {
  // Setup
  if (viewReset) {
    newView(0);
    printData("cm", "", 280, 170, ILI9341_RED, 3);
    printInt(range, 0, 10, 30, ILI9341_RED, 15);
    viewReset = false;
  }

  // Loop
  if (lastRange != range) {
    printInt(range, &lastRange, 10, 30, ILI9341_RED, 15);
  }
}

// Ultrasonic Range
void clock(time_t current) {
  // Setup
  if (viewReset) {
    newView(4);
    viewReset = false;
  }

  // Loop
  if (second(current) != second(lastTime)) {
    char clearDate[11];
    sprintf(clearDate, "%02d/%02d/%d", day(lastTime), month(lastTime), year(lastTime));
    char date[11];
    sprintf(date, "%02d/%02d/%d", day(current), month(current), year(current));
    printData(date, clearDate, 10, 10, ILI9341_RED, 3);

    char clearTime[11];
    sprintf(clearTime, "%02d:%02d:%02d", hour(lastTime), minute(lastTime), second(lastTime));
    char timeNow[11];
    sprintf(timeNow, "%02d:%02d:%02d", hour(current), minute(current), second(current));
    printData(timeNow, clearTime, 10, 50, ILI9341_RED, 3);

    lastTime = current;
  }
}

/*
 * TFT functions
 */

 // Fill the screen black
void clearDisplay() {
  tft.fillRect(0, 0, 320, 198, ILI9341_BLACK);
}

// Fill navbar
void navbar() {
  tft.fillRect(0, 198, 320, 42, darkblue);
}

// Blank display for new view
void newView(int menu) {
  clearDisplay();
  tft.fillRect(0, 198, 320, 42, darkblue);
  tft.fillRect(menu * 64, 198, 64, 42, orange);
  viewReset = false;
}

// Print integer
void printInt(int msg, int *lastMsg, int x, int y, uint16_t color, int textSize) {
  if (*lastMsg != msg) {
    char lastMsgStr[6];
    sprintf(lastMsgStr, "%d", *lastMsg);
    char msgStr[6];
    sprintf(msgStr, "%d", msg);
    printData(msgStr, lastMsgStr, x, y, ILI9341_RED, textSize);
    *lastMsg = msg;
  }
}

// Print Data
void printData(char msg[], char lastMsg[], int x, int y,  uint16_t color, int textSize) {
  tft.setCursor(x, y);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(textSize);
  tft.println(lastMsg);
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  tft.println(msg);
}

// Report error message
void errorMsg(char msg[]) {
  clearDisplay();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_RED);                  
  tft.setTextSize(2);
  tft.println(msg);

  while(1);
}

/*
 * SD functions
 */

// Log to SD
void logToCard(char msg[]) {
  if (millis() >= totalTime + 1000) {
    logFile = SD.open(logName, FILE_WRITE);

    if (logFile) {
      logFile.println(msg);
      logFile.close();
    }
    totalTime = millis();
  }
}

/*
 * Formatting functions
 */

// Remove spaces from string
char *deblank(char *str) {
  char *out = str, *put = str;

  for(; *str != '\0'; ++str) {
    if(*str != ' ')
      *put++ = *str;
  }
  *put = '\0';

  return out;
}