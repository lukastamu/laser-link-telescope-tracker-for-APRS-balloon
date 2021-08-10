#include <LibAPRS.h>        //Modified version of https://github.com/markqvist/LibAPRS
#include <Wire.h>
#include <SoftwareSerial.h>
#include "GEOFENCE.h"       // Modified version of https://github.com/TomasTT7/TT7F-Float-Tracker/blob/master/Software/ARM_GEOFENCE.c
#include <avr/wdt.h>
#include <EEPROM.h>

#define I2C_ADDRESS 8

#define RfPDPin     19
#define RfPwrHLPin  21
#define RfPttPin    20
#define BattPin     A2
#define PIN_DRA_RX  22
#define PIN_DRA_TX  23

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH false

#define RfON          digitalWrite(RfPDPin, HIGH)
#define RfOFF         digitalWrite(RfPDPin, LOW)
#define RfPwrHigh     pinMode(RfPwrHLPin, INPUT)
#define RfPwrLow      pinMode(RfPwrHLPin, OUTPUT);digitalWrite(RfPwrHLPin, LOW)
#define RfPttON       digitalWrite(RfPttPin, HIGH)//NPN
#define RfPttOFF      digitalWrite(RfPttPin, LOW)
#define AprsPinInput  pinMode(12,INPUT);pinMode(13,INPUT);pinMode(14,INPUT);pinMode(15,INPUT)
#define AprsPinOutput pinMode(12,OUTPUT);pinMode(13,OUTPUT);pinMode(14,OUTPUT);pinMode(15,OUTPUT)

//#define DEVMODE // Development mode. Uncomment to enable for debugging.

//https://github.com/lightaprs/LightAPRS-1.0/wiki/Tips-&-Tricks-for-Pico-Balloons

//****************************************************************************
char  CallSign[7] = "LY5N"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
int   CallNumber = 11; //SSID http://www.aprs.org/aprs11/SSIDs.txt
char  Symbol = 'O'; // '/O' for balloon, '/>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool  alternateSymbolTable = false ; //false = '/' , true = '\'

char StatusMessage[50] = "Laser balloon";
//*****************************************************************************


unsigned int   BeaconWait = 5; //seconds sleep for next beacon (TX).
unsigned int   BattWait = 60;  //seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel)
float BattMin = 4.5;      // min Volts to wake up.
float DraHighVolt = 9;  // min Volts for radio module (DRA818V) to transmit (TX) 1 Watt, below this transmit 0.5 Watt.

boolean aliveStatus = true; //for tx status message on first wake-up just once.

//do not change WIDE path settings below if you don't know what you are doing :)
byte  Wide1 = 1; // 1 for WIDE1-1 path
byte  Wide2 = 1; // 1 for WIDE2-1 path

/**
  Airborne stations above a few thousand feet should ideally use NO path at all, or at the maximum just WIDE2-1 alone.
  Due to their extended transmit range due to elevation, multiple digipeater hops are not required by airborne stations.
  Multi-hop paths just add needless congestion on the shared APRS channel in areas hundreds of miles away from the aircraft's own location.
  NEVER use WIDE1-1 in an airborne path, since this can potentially trigger hundreds of home stations simultaneously over a radius of 150-200 miles.
*/
int pathSize = 2; // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N
boolean autoPathSizeHighAlt = true; //force path to WIDE2-N only for high altitude (airborne) beaconing (over 1.000 meters (3.280 feet))

boolean beaconViaARISS = false; //there are no iGates in some regions (such as North Africa,  Oceans, etc) so try to beacon via ARISS (International Space Station) https://www.amsat.org/amateur-radio-on-the-iss/

// GEOFENCE
uint32_t GEOFENCE_APRS_frequency      = 144800000; //default frequency before geofencing. This variable will be updated based on GPS location.
uint32_t GEOFENCE_no_tx               = 0;

boolean radioSetup = false;
boolean GpsFirstFix = false;

static char telemetry_buff[200];// telemetry buffer
uint16_t TxCount = 1;

float GNSS_lat = 0;
float GNSS_lon = 0;
float GNSS_altitude = 0;
float pressureAltitude = 0;
float baroTemp = 0;
long baroPressure = 0;
float battVoltage = 0;
float tempSensor[6] = {0};
float Gxyz[3] = {0};
float Axyz[3] = {0};
float gimbalHeading = 0;
float gimbalBattVoltage = 0;
float gimbalGyroZ = 0;

char I2Cmsg[200];
int pos = 0;
bool reading = false;
bool newData = false;

void setup() {
  wdt_enable(WDTO_8S);
  analogReference(INTERNAL2V56);
  pinMode(RfPDPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(RfPttPin, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PIN_DRA_TX, INPUT);

  RfOFF;
  RfPwrLow;
  RfPttOFF;

  Serial.begin(9600);
#if defined(DEVMODE)
  Serial.println(F("Start"));
#endif

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(CallSign, CallNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setMessageDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", Wide1);
  APRS_setPath2("WIDE2", Wide2);
  APRS_useAlternateSymbolTable(alternateSymbolTable);
  APRS_setSymbol(Symbol);
  //increase following value (for example to 500UL) if you experience packet loss/decode issues.
  APRS_setPreamble(350UL);
  APRS_setPathSize(pathSize);
  AprsPinInput;

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
}

void loop() {
  wdt_reset();
#if defined(DEVMODE)
  Serial.println();
  Serial.print("Lat: ");
  Serial.print(GNSS_lat, 6);
  Serial.print(" Lon: ");
  Serial.print(GNSS_lon, 6);
  Serial.print(" GNSS altitude: ");
  Serial.print(GNSS_altitude);
  Serial.print(" Pressure altitude: ");
  Serial.println(pressureAltitude);
  Serial.print("Pressure: ");
  Serial.print(baroPressure);
  Serial.print(" Baro temp: ");
  Serial.print(baroTemp);
  Serial.print(" Battery: ");
  Serial.println(battVoltage);
  Serial.print("Temperatures: ");
  for (int i = 0; i <= 5; i++) {
    Serial.print(tempSensor[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Gyro: ");
  for (int i = 0; i <= 2; i++) {
    Serial.print(Gxyz[i]);
    Serial.print("   ");
  }
  Serial.print("Accel: ");
  for (int i = 0; i <= 2; i++) {
    Serial.print(Axyz[i]);
    Serial.print("   ");
  }
  Serial.println();
  Serial.print("Gimbal Heading: ");
  Serial.print(gimbalHeading);
  Serial.print(" Gimbal battery: ");
  Serial.print(gimbalBattVoltage);
  Serial.print(" Gimbal Gyro Z: ");
  Serial.println(gimbalGyroZ);
#endif

  if (aliveStatus) {
    //send status tx on startup once (before gps fix)
#if defined(DEVMODE)
    Serial.println(F("Sending"));
#endif
    sendStatus();
    aliveStatus = false;
    sleepSeconds(BeaconWait);
  }

  if (GNSS_lat != 0 && GNSS_lon != 0) {
    updatePosition();
    updateTelemetry();

    GpsFirstFix = true;

    if (autoPathSizeHighAlt && GNSS_altitude > 1000) {
      //force to use high altitude settings (WIDE2-n)
      APRS_setPathSize(1);
    }
    else {
      //use default settings
      APRS_setPathSize(pathSize);
    }
    //APRS frequency isn't the same for the whole world. (for pico balloon only)
    if (!radioSetup) {
      configureFreqbyLocation();
    }
    //send status message every 60 minutes
    /*if (gps.time.minute() == 0) {
      sendStatus();
      }*/

    sendLocation();
    Serial.flush();
    sleepSeconds(BeaconWait);
  }
  else {
#if defined(DEVMODE)
    Serial.println(F("No GPS fix"));
#endif
    sleepSeconds(BeaconWait);
  }
}

void receiveEvent(int howMany) {
  newData = false;
  //Serial.println();
  while (Wire.available()) {
    char databyte = (char)Wire.read();
    //Serial.print(databyte);
    if (newData == false) {
      if (databyte == '<') reading = true;
      else if (reading) {
        if (databyte != '>') {
          I2Cmsg[pos] = databyte;
          pos++;
        }
        else {
          I2Cmsg[pos] = '\0';
          pos = 0;
          reading = false;
          newData = true;
          char readDataCopy[200];
          char * strtokIndex;
          strcpy(readDataCopy, I2Cmsg);
          strtokIndex = strtok(readDataCopy, ",");
          if (atof(strtokIndex) != 0) GNSS_lat = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) GNSS_lon = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) GNSS_altitude = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) pressureAltitude = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) baroPressure = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) baroTemp = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) tempSensor[0] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) tempSensor[1] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) tempSensor[2] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) tempSensor[3] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) tempSensor[4] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) tempSensor[5] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) Gxyz[0] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) Gxyz[1] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) Gxyz[2] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) Axyz[0] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) Axyz[1] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) Axyz[2] = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) battVoltage = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) gimbalHeading = atof(strtokIndex);
          strtokIndex = strtok(NULL, ",");
          if (atof(strtokIndex) != 0) gimbalBattVoltage = atof(strtokIndex);
          strtokIndex = strtok(NULL, ">");
          if (atof(strtokIndex) != 0) gimbalGyroZ = atof(strtokIndex);
        }
      }
    }
  }
}

void aprs_msg_callback(struct AX25Msg *msg) {
  //do not remove this function, necessary for LibAPRS
}

void sleepSeconds(int sec) {
  RfOFF;
  RfPttOFF;
  wdt_disable();
  for (int i = 0; i < sec; i++) {
    delay(1000);
  }
  wdt_enable(WDTO_8S);
}


boolean isAirborneAPRSAllowed() {
  GEOFENCE_position(GNSS_lat, GNSS_lon);
  boolean airborne = true;
  if (GEOFENCE_no_tx == 1) {
    airborne = false;
  }
  return airborne;
}


boolean inARISSGeoFence(float tempLat, float tempLong) {
  boolean ariss = false;
  //North Africa
  if (tempLat > 0 && tempLat < 32 && tempLong > 0 && tempLong < 32) {
    ariss = true;
  }
  //North Pacific
  if (tempLat > 28 && tempLat < 50 && tempLong > -180 && tempLong < -130) {
    ariss = true;
  }
  //North Atlantic
  if (tempLat > 25 && tempLat < 42 && tempLong > -60 && tempLong < -33) {
    ariss = true;
  }

  return ariss;
}

void configureFreqbyLocation() {
  if (beaconViaARISS && inARISSGeoFence(GNSS_lat, GNSS_lon)) {
    APRS_setPath1("ARISS", Wide1);
    APRS_setPath2("WIDE2", Wide2);
    APRS_setPathSize(2);
    configDra818("145.8250");
  }
  else {
    GEOFENCE_position(GNSS_lat, GNSS_lon);
    float dividedFreq = GEOFENCE_APRS_frequency / 1000000.f;
    char aprsFreq_buff[8];
    dtostrf(dividedFreq, 8, 4, aprsFreq_buff);
    configDra818(aprsFreq_buff);
  }
  radioSetup = true;
}

byte configDra818(char *freq) {
  SoftwareSerial Serial_dra(PIN_DRA_RX, PIN_DRA_TX);
  Serial_dra.begin(9600);
  RfON;
  char ack[3];
  int n;
  delay(2000);
  char cmd[50];
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial_dra.println(cmd);
  ack[2] = 0;
  while (ack[2] != 0xa) {
    if (Serial_dra.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial_dra.read();
    }
  }
  Serial_dra.end();
  RfOFF;
  pinMode(PIN_DRA_TX, INPUT);
#if defined(DEVMODE)
  if (ack[0] == 0x30) Serial.println(F("Frequency updated...")); else Serial.println(F("Frequency update error!"));
#endif
  return (ack[0] == 0x30) ? 1 : 0;
}

void updatePosition() {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  int temp = 0;

  double d_lat = GNSS_lat;
  double dm_lat = 0.0;

  if (d_lat < 0.0) {
    temp = -(int)d_lat;
    dm_lat = temp * 100.0 - (d_lat + temp) * 60.0;
  }
  else {
    temp = (int)d_lat;
    dm_lat = temp * 100 + (d_lat - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, latStr);

  if (dm_lat < 1000) {
    latStr[0] = '0';
  }

  if (d_lat >= 0.0) {
    latStr[7] = 'N';
  }
  else {
    latStr[7] = 'S';
  }

  APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  double d_lon = GNSS_lon;
  double dm_lon = 0.0;

  if (d_lon < 0.0) {
    temp = -(int)d_lon;
    dm_lon = temp * 100.0 - (d_lon + temp) * 60.0;
  }
  else {
    temp = (int)d_lon;
    dm_lon = temp * 100 + (d_lon - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, lonStr);

  if (dm_lon < 10000) {
    lonStr[0] = '0';
  }
  if (dm_lon < 1000) {
    lonStr[1] = '0';
  }

  if (d_lon >= 0.0) {
    lonStr[8] = 'E';
  }
  else {
    lonStr[8] = 'W';
  }

  APRS_setLon(lonStr);
}

void updateTelemetry() {
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
  char cg_heading[10];
  char cg_battVoltage[10];
  char cg_gyroZ[10];

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
  dtostrf(gimbalHeading, 3, 2, cg_heading);
  dtostrf(gimbalBattVoltage, 3, 2, cg_battVoltage);
  dtostrf(gimbalGyroZ, 3, 2, cg_gyroZ);

  sprintf(telemetry_buff, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", c_battVoltage, c_GNSS_altitude,
          c_pressureAltitude, c_baroPressure, c_baroTemp, c_temp1, c_temp2, c_temp3, c_temp4, c_temp5, c_temp6,
          c_Gx, c_Gy, c_Gz, c_Ax, c_Ay, c_Az, cg_heading, cg_battVoltage, cg_gyroZ);

  //dtostrf(readBatt(), 5, 2, telemetry_buff + 43);

#if defined(DEVMODE)
  Serial.println("Telemetry:");
  Serial.println(telemetry_buff);
  Serial.println();
#endif
}

void sendLocation() {
#if defined(DEVMODE)
  Serial.println(F("Sending location"));
#endif
  if ((readBatt() > DraHighVolt) && (readBatt() < 10)) RfPwrHigh; //DRA Power 1 Watt
  else RfPwrLow; //DRA Power 0.5 Watt

  int hh = 0;
  int mm = 0;
  int ss = 0;

  char timestamp_buff[7];

  sprintf(timestamp_buff, "%02d", hh);
  sprintf(timestamp_buff + 2, "%02d", mm);
  sprintf(timestamp_buff + 4, "%02d", ss);
  timestamp_buff[6] = 'h';
  AprsPinOutput;
  RfON;
  delay(2000);
  RfPttON;
  delay(1000);

  APRS_sendLoc(telemetry_buff, strlen(telemetry_buff)); //beacon without timestamp
  //APRS_sendLocWtTmStmp(telemetry_buff, strlen(telemetry_buff), timestamp_buff); //beacon with timestamp
  delay(50);
  while (digitalRead(1)) {
    ; //LibAprs TX Led pin PB1
  }
  delay(50);
  RfPttOFF;
  RfOFF;
  AprsPinInput;
#if defined(DEVMODE)
  Serial.println(F("Location sent"));
#endif
  TxCount++;
}

void sendStatus() {
  if ((readBatt() > DraHighVolt) && (readBatt() < 10)) RfPwrHigh; //DRA Power 1 Watt
  else RfPwrLow; //DRA Power 0.5 Watt

  char status_buff[60];
  sprintf(status_buff, "%s", StatusMessage);

  AprsPinOutput;
  RfON;
  delay(2000);
  RfPttON;
  delay(1000);

  APRS_sendStatus(status_buff, strlen(status_buff));
  delay(50);
  while (digitalRead(1)) {
    ; //LibAprs TX Led pin PB1
  }
  delay(50);
  RfPttOFF;
  RfOFF;
  AprsPinInput;
#if defined(DEVMODE)
  Serial.println(F("Status sent"));
#endif
  TxCount++;
}

float readBatt() {
  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0;
  do {
    value = analogRead(BattPin);
    delay(5);
    value = analogRead(BattPin);
    value = value - 8;
    value = (value * 2.56) / 1024.0;
    value = value / (R2 / (R1 + R2));
  } while (value > 16.0);
  return value ;
}
