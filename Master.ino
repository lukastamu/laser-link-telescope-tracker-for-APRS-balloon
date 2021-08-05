// ----- Libraries
#include <Wire.h>
#include <NMEAGPS.h>
#include <SdFat.h>
#include <BMP180I2C.h>
#include <basicMPU6050.h>
#include <avr/wdt.h>

// ----- SD card
SdFat SD;
File logfile;

// ----- Configure GNSS
NMEAGPS gnss;
gps_fix fix;

// ----- IMU
basicMPU6050<> imu;
float Gxyz[3] = {0};
float Axyz[3] = {0};
//calibration data

// ----- Barometer
#define I2C_ADDRESS 0x77
BMP180I2C bmp180(I2C_ADDRESS);

// ----- LightAPRS
#define LightAPRS_ADDRESS 8

// ----- Information from this device
// GNSS
float GNSS_lat = 0;
float GNSS_lon = 0;
float GNSS_altitude = 0;
float GNSS_speed = 0;
int GNSS_heading = 0;
// Barometer
float baroTemp = 0;
long baroPressure = 0;
float pressureAltitude = 0;
const float seaLevelPressure = 1016.25;
// Temperature sensors
#define temp1pin A1
#define temp2pin A8
#define temp3pin A9
#define temp4pin A10
#define temp5pin A11
#define temp6pin A12
float tempSensor[6] = {0};
// Power
float battVoltage = 0;
float R1 = 0;
float R2 = 0;
// Time
int time_year = 0, time_mon = 0, time_day = 0;
int time_hour = 0, time_min = 0, time_sec = 0;
int timezone = 3;

// ----- Information from slave device
float rotorHeading = 0;
float rotorBattVoltage = 0;
float rotorGyroZ = 0;

// ----- MISC
#define aref_voltage 3.3
int loopcounter1 = 0;
int loopcounter2 = 0;

// ----- Startup sequence
void setup() {
  // Start debug link
  Serial.begin(9600);
  // Initialize two LED pins
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  // Start I2C
  Wire.begin();
  // Start GNSS
  Serial1.begin(38400);
  // Start Bluetooth
  Serial3.begin(9600);
  // Start SD card
  if (!SD.begin(53))
  {
    Serial.println("SD card init failed!");
    for (int i = 1; i <= 2; i++) {
      digitalWrite(4, HIGH);
      delay(100);
      digitalWrite(4, LOW);
      delay(100);
    }
    delay(2000);
  }
  initializeSD();
  // Start barometer
  if (!bmp180.begin())
  {
    Serial.println("BMP sensor init failed!");
    for (int i = 1; i <= 4; i++) {
      digitalWrite(4, HIGH);
      delay(100);
      digitalWrite(4, LOW);
      delay(100);
    }
    delay(2000);
  }
  bmp180.resetToDefaults();
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);
  // initialize IMU
  imu.setup();
  imu.setBias();
  // Set analog reference
  analogReference(EXTERNAL);

  wdt_enable(WDTO_8S);

  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  Serial.println("System ready!");
}

// ----- Main loop
void loop() {
  wdt_reset();
  updateGNSS();

  if (loopcounter1 > 100) {
    getMPU();
    getBTData();
    readTemperatures();
    loopcounter1 = 0;
  }
  if (loopcounter2 > 1000) {
    Serial.print("Gyro");
    for (int i = 0; i <= 2; i++) {
      Serial.print("  ");
      Serial.print(Gxyz[i]);
    }
    Serial.println();
    Serial.print("Accel");
    for (int i = 0; i <= 2; i++) {
      Serial.print("  ");
      Serial.print(Axyz[i]);
    }
    Serial.println();
    digitalWrite(5, HIGH);
    loopcounter2 = 0;
    writeToSD();
    updateBaroData();
    sendDataToAPRS();
    digitalWrite(5, LOW);
    if (rotorBattVoltage != 0) {
      delay(50);
      digitalWrite(4, HIGH);
      delay(50);
      digitalWrite(4, LOW);
    }
  }
  loopcounter1++;
  loopcounter2++;
  delay(1);
}

// ----- Read new data from sensor
void getMPU() {
  imu.updateBias();
  Axyz[0] = imu.ax();
  Axyz[1] = imu.ay();
  Axyz[2] = imu.az();
  Gxyz[0] = imu.gx();
  Gxyz[1] = imu.gy();
  Gxyz[2] = imu.gz();
}

// ----- Get new data from GNSS module
void updateGNSS() {
  while (gnss.available(Serial1)) {
    fix = gnss.read();
    if (fix.valid.date) {
      time_year = 2000 + fix.dateTime.year;
      time_mon = fix.dateTime.month;
      time_day = fix.dateTime.date;
    }
    if (fix.valid.time) {
      time_hour = fix.dateTime.hours + timezone;
      if (time_hour > 23) time_hour -= 24;
      else if (time_hour < 0) time_hour += 24;
      time_min = fix.dateTime.minutes;
      time_sec = fix.dateTime.seconds;
    }
    if (fix.valid.location) {
      GNSS_lat = fix.latitude();
      GNSS_lon = fix.longitude();
    }
    if (fix.valid.altitude) GNSS_altitude = fix.altitude();
    if (fix.valid.speed) GNSS_speed = fix.speed_kph();
    if (fix.valid.heading) GNSS_heading = fix.heading();
  }
}

// ----- Read new data from BMP180
void updateBaroData() {
  if (!bmp180.measureTemperature()) return;
  do {
    delay(10);
  } while (!bmp180.hasTemperatureValue());
  baroTemp = bmp180.getTemperature() - 3;

  if (!bmp180.measurePressure()) return;

  do {
    delay(10);
  } while (!bmp180.hasPressureValue());
  baroPressure = bmp180.getPressure();

  pressureAltitude = 44330 * (1.0 - pow(baroPressure / 100 / seaLevelPressure, 0.1903));
}

// ----- Mark the start of logging
void initializeSD() {
  logfile = SD.open("log.txt", FILE_WRITE);
  logfile.println("Logging started");
  logfile.println("| Date | Time | GNSS lat, lon, alt(m), speed(kph), heading | Gyro X , Y , Z (deg/s) | Accel X , Y , Z (m/s^2) |"
                  " Baro temp, pressure(Pa), pressure alt(m) | Temp 1, 2, 3, 4, 5, 6 | Batt | Rotor: heading, batt, gyro Z |");
  logfile.close();
}

// ----- Write data to SD card
void writeToSD() {
  char DateTimeString[20];
  sprintf(DateTimeString, "%04d-%02d-%02d,%02d:%02d:%02d", time_year, time_mon, time_day, time_hour, time_min, time_sec);
  logfile = SD.open("log.txt", FILE_WRITE);
  logfile.print(DateTimeString);
  logfile.print(",");
  logfile.print(GNSS_lat, 6);
  logfile.print(",");
  logfile.print(GNSS_lon, 6);
  logfile.print(",");
  logfile.print(GNSS_altitude);
  logfile.print(",");
  logfile.print(GNSS_speed);
  logfile.print(",");
  logfile.print(GNSS_heading);
  logfile.print(",");
  for (int i = 0; i <= 2; i++) {
    logfile.print(Gxyz[i]);
    logfile.print(",");
  }
  for (int i = 0; i <= 2; i++) {
    logfile.print(Axyz[i]);
    logfile.print(",");
  }
  logfile.print(baroTemp);
  logfile.print(",");
  logfile.print(baroPressure);
  logfile.print(",");
  logfile.print(pressureAltitude);
  logfile.print(",");
  for (int i = 0; i <= 5; i++) {
    logfile.print(tempSensor[i]);
    logfile.print(",");
  }
  logfile.print(battVoltage);
  logfile.print(",");
  logfile.print(rotorHeading);
  logfile.print(",");
  logfile.print(rotorBattVoltage);
  logfile.print(",");
  logfile.print(rotorGyroZ);
  logfile.println();
  logfile.close();
}

// ----- Send data to LightAPRS module via I2C
void sendDataToAPRS() {
  char c_GNSS_lat[15];
  char c_GNSS_lon[15];
  char c_GNSS_altitude[20];
  char c_pressureAltitude[20];
  char c_baroPressure[20];
  char c_baroTemp[10];
  char c_battVoltage[10];
  char c_temp1[10];
  char c_temp2[10];
  char c_temp3[10];
  char c_temp4[10];
  char c_temp5[10];
  char c_temp6[10];
  char c_Gx[10];
  char c_Gy[10];
  char c_Gz[10];
  char c_Ax[10];
  char c_Ay[10];
  char c_Az[10];
  char cr_heading[10];
  char cr_battVoltage[10];
  char cr_gyroZ[10];

  dtostrf(GNSS_lat, 3, 6, c_GNSS_lat);
  dtostrf(GNSS_lon, 3, 6, c_GNSS_lon);
  dtostrf(GNSS_altitude, 3, 2, c_GNSS_altitude);
  dtostrf(pressureAltitude, 3, 2, c_pressureAltitude);
  dtostrf(baroTemp, 3, 2, c_baroTemp);
  dtostrf(baroPressure, 3, 0, c_baroPressure);
  dtostrf(tempSensor[0], 3, 1, c_temp1);
  dtostrf(tempSensor[1], 3, 1, c_temp2);
  dtostrf(tempSensor[2], 3, 1, c_temp3);
  dtostrf(tempSensor[3], 3, 1, c_temp4);
  dtostrf(tempSensor[4], 3, 1, c_temp5);
  dtostrf(tempSensor[5], 3, 1, c_temp6);
  dtostrf(Gxyz[0], 3, 2, c_Gx);
  dtostrf(Gxyz[1], 3, 2, c_Gy);
  dtostrf(Gxyz[2], 3, 2, c_Gz);
  dtostrf(Axyz[0], 3, 2, c_Ax);
  dtostrf(Axyz[1], 3, 2, c_Ay);
  dtostrf(Axyz[2], 3, 2, c_Az);
  dtostrf(battVoltage, 3, 2, c_battVoltage);
  dtostrf(rotorHeading, 3, 2, cr_heading);
  dtostrf(rotorBattVoltage, 3, 2, cr_battVoltage);
  dtostrf(rotorGyroZ, 3, 2, cr_gyroZ);

  Wire.beginTransmission(LightAPRS_ADDRESS);
  Wire.write("<");
  Wire.write(c_GNSS_lat);
  Wire.write(",");
  Wire.write(c_GNSS_lon);
  Wire.write(",");
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(LightAPRS_ADDRESS);
  Wire.write(c_GNSS_altitude);
  Wire.write(",");
  Wire.write(c_pressureAltitude);
  Wire.write(",");
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(LightAPRS_ADDRESS);
  Wire.write(c_baroPressure);
  Wire.write(",");
  Wire.write(c_baroTemp);
  Wire.write(",");
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(LightAPRS_ADDRESS);
  Wire.write(c_temp1);
  Wire.write(",");
  Wire.write(c_temp2);
  Wire.write(",");
  Wire.write(c_temp3);
  Wire.write(",");
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(LightAPRS_ADDRESS);
  Wire.write(c_temp4);
  Wire.write(",");
  Wire.write(c_temp5);
  Wire.write(",");
  Wire.write(c_temp6);
  Wire.write(",");
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(LightAPRS_ADDRESS);
  Wire.write(c_Gx);
  Wire.write(",");
  Wire.write(c_Gy);
  Wire.write(",");
  Wire.write(c_Gz);
  Wire.write(",");
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(LightAPRS_ADDRESS);
  Wire.write(c_Ax);
  Wire.write(",");
  Wire.write(c_Ay);
  Wire.write(",");
  Wire.write(c_Az);
  Wire.write(",");
  Wire.write(c_battVoltage);
  Wire.write(",");
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(LightAPRS_ADDRESS);
  Wire.write(cr_heading);
  Wire.write(",");
  Wire.write(cr_battVoltage);
  Wire.write(",");
  Wire.write(cr_gyroZ);
  Wire.write(">");
  Wire.endTransmission();
}

// ----- Parse new data from Bluetooth
void getBTData() {
  char * strtokIndex;
  char readDataCopy[50];
  char readData[50];
  char rch;
  int index = 0;
  bool reading = false;
  bool newData = false;

  while (Serial3.available() > 0) {
    rch = Serial3.read();
    if (newData == false) {
      if (rch == '<') reading = true;
      else if (reading) {
        if (rch != '>') {
          readData[index] = rch;
          index++;
        }
        else {
          readData[index] = '\0';
          index = 0;
          reading = false;
          newData = true;
        }
      }
    }
  }
  if (newData == true) {
    strcpy(readDataCopy, readData);
    strtokIndex = strtok(readDataCopy, ",");
    if (atof(strtokIndex) != 0) rotorHeading = atof(strtokIndex);
    strtokIndex = strtok(NULL, ",");
    if (atof(strtokIndex) != 0) rotorBattVoltage = atof(strtokIndex);
    strtokIndex = strtok(NULL, ">");
    if (atof(strtokIndex) != 0) rotorGyroZ = atof(strtokIndex);
    newData = false;
  }
  newData = false;
}

// ----- Read temperatures from TMP36 analog sensors
void readTemperatures() {
  tempSensor[0] = (analogRead(temp1pin) * aref_voltage / 1024.0 - 0.5) * 100;
  tempSensor[1] = (analogRead(temp2pin) * aref_voltage / 1024.0 - 0.5) * 100;
  tempSensor[2] = (analogRead(temp3pin) * aref_voltage / 1024.0 - 0.5) * 100;
  tempSensor[3] = (analogRead(temp4pin) * aref_voltage / 1024.0 - 0.5) * 100;
  tempSensor[4] = (analogRead(temp5pin) * aref_voltage / 1024.0 - 0.5) * 100;
  tempSensor[5] = (analogRead(temp6pin) * aref_voltage / 1024.0 - 0.5) * 100;
}

// ----- Read battery voltage
void measureBatteryVoltage() {
  float raw_voltage = analogRead(A13) * aref_voltage / 1024.0;
  battVoltage = battVoltage * 0.95 + raw_voltage * 0.05;
}
