// ----- Libraries
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include "libs/I2Cdev.cpp"
#include "libs/MPU9250.cpp"
#include <avr/wdt.h>

// ----- Static telescope coordinates
float tele_lat = 54.810128;
float tele_lon = 25.416873;

// ----- IMU
MPU9250 myIMU;
//calibration data
float A_cal[6] = {316.05, 104.00, -346.28, 6.137e-5, 5.9879e-5, 6.0326e-5}; // 0..2 offset xyz, 3..5 scale xyz
float M_cal[6] = {11.88, 63.05, -67.96, 0.0138307, 0.0139207, 0.0130670};
float G_off[3] = { -190.0, 178.2, 48.4};
//raw data and scaled as vector
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
//magnetic declination
float declination = +8.5;
//general vars
int heading = 0;
float smoothing = 0.82 ;
//orientation vector
float p[] = {1, 0, 0};
/*
  This tilt-compensated code assumes that the sensor is oriented with Mag X pointing
  to the North, Y pointing East, and Z pointing down (toward the ground).
  With the MPU9250, the accelerometer is aligned differently, so the accelerometer axes are swapped.

  The code compensates for tilts of up to 90 degrees away from horizontal.
  Facing vector p is the direction of travel and allows reassigning these directions.
  It should be defined as pointing forward,
  parallel to the ground, with coordinates {X, Y, Z} (in magnetometer frame of reference).
*/

// ----- GNSS
NeoSWSerial gnssPort(3, 2); // RX TX
NMEAGPS  gnss; // This parses the GPS characters
float GNSS_lat = 0;
float GNSS_lon = 0;

// ----- Servo
Servo servo;

// ----- PID
double setpoint = 0;
double input = 0;
double output = 0;
//aggresive values
double ap = 0.8;
double ai = 0.01;
double ad = 0.002;
//concervative values
double cp = 0.5;
double ci = 0.02;
double cd = 0.002;
PID myPID(&input, &output, &setpoint, ap, ai, ad, DIRECT);

// ----- MISC
#define ledON          digitalWrite(13, HIGH)
#define ledOFF         digitalWrite(13, LOW)
float battVoltage = 0;
float toDeg = 180 / PI;
float toRad = PI / 180;
unsigned long time_started = 0;

// ----- Setup sequence
void setup() {
  //start watchdog
  wdt_enable(WDTO_1S);
  Wire.begin();
  //start Bluetooth
  Serial.begin(9600);
  //start GNSS port
  gnssPort.begin(9600);
  //start servo
  servo.attach(5);
  //setup PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-89, 89);
  myPID.SetSampleTime(5);
  //initialize IMU
  myIMU.initialize();
  myIMU.setFullScaleGyroRange(1); //500 LSB(d/s) scale
  myIMU.setFullScaleAccelRange(1); //4G scale
  pinMode(13, OUTPUT);
}

// ----- Main loop
void loop() {
  wdt_reset();

  getHeading_smooth();
  calculatePIDInput(calculateAzimuth(), heading);

  float delta = abs(setpoint - input);
  if (delta <= 24) myPID.SetTunings(cp, ci, cd);
  else myPID.SetTunings(ap, ai, ad);
  myPID.Compute();
  //move servo accordingly
  if (GNSS_lat != 0 && GNSS_lon != 0) {
    servo.write(output + 90);
    //indicate setpoint
    if (delta <= 5) ledON;
    else ledOFF;
  }
  if ((millis() - time_started) > 200) {
    sendBTData();
    time_started = millis();
  }
  measureBatteryVoltage();
}

// ----- Read new data from sensor
void getMPU() {
  myIMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Axyz[0] = (float) ax * 2.048;
  Axyz[1] = (float) ay * 2.048;
  Axyz[2] = (float) az * 2.048;
  for (int i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

  Mxyz[0] = (float) mx;
  Mxyz[1] = (float) my;
  Mxyz[2] = (float) mz;
  for (int i = 0; i < 3; i++) Mxyz[i] = (Mxyz[i] - M_cal[i]) * M_cal[i + 3];

  Gxyz[0] = ((float) gx - G_off[0]) * 500. / 32768;
  Gxyz[1] = ((float) gy - G_off[1]) * 500. / 32768;
  Gxyz[2] = ((float) gz - G_off[2]) * 500. / 32768;
}

// ----- Get heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int getHeading(float acc[3], float mag[3], float p[3]) {
  float E[3], N[3]; //direction vectors

  // cross "down" (acceleration vector) with magnetic vector (magnetic north + inclination) with  to produce "east"
  vector_cross(acc, mag, E);
  vector_normalize(E);

  // cross "east" with "down" to produce "north" (parallel to the ground)
  vector_cross(E, acc, N);
  vector_normalize(N);

  // compute heading
  int heading = round(atan2(vector_dot(E, p), vector_dot(N, p)) * 180 / M_PI);
  heading = -heading + 90 + declination;
  if (heading < 0)
    heading += 360;
  else if (heading > 360)
    heading -= 360;
  return heading;
}

// ----- Smoothen any fluctuations
void getHeading_smooth() {
  getMPU();
  // correct for accelerometer and magnetometer alignment
  float tmp = Axyz[1];
  Axyz[1] = Axyz[0]; //swap x and y
  Axyz[0] = tmp;
  Axyz[2] = -Axyz[2]; //invert z axis

  int new_heading = getHeading(Axyz, Mxyz, p);

  if (heading - new_heading > 180) {
    heading = heading * smoothing + (360 + new_heading) * (1 - smoothing);
    if (heading > 360) heading -= 360;
  }
  else if (heading - new_heading < -180) {
    heading = heading * smoothing + (new_heading - 360) * (1 - smoothing);
    if (heading < 0) heading += 360;
  }
  else heading = heading * smoothing + new_heading * (1 - smoothing);
}

// ----- Vector functions
void vector_cross(float a[3], float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}
void vector_normalize(float a[3]) {
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}
float vector_dot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// ----- Get new data from GNSS module
void updateGNSS() {
  while (gnss.available(gnssPort)) {
    gps_fix  fix = gnss.read();
    if (fix.valid.location) {
      GNSS_lat = fix.latitude();
      GNSS_lon = fix.longitude();
    }
  }
}

// ----- Calculate azmiuth from telescope and balloon coordinates
int calculateAzimuth() {
  updateGNSS();
  int azimuth;
  float fi_1 = GNSS_lat * toRad;
  float lambda_1 = GNSS_lon * toRad;
  float fi_2 = tele_lat * toRad;
  float lambda_2 = tele_lon * toRad;
  azimuth = round(atan2(sin(lambda_2 - lambda_1) * cos(fi_2), cos(fi_1) * sin(fi_2) - sin(fi_1) * cos(fi_2) * cos(lambda_2 - lambda_1)) * toDeg);
  if (azimuth < 0) azimuth += 360;
  else if (azimuth > 360) azimuth -= 360;
  return azimuth;
}

// ----- Offset azimuth and heading correctly
void calculatePIDInput(float azimuth, float heading) {
  if (azimuth > 180) azimuth -= 360;
  if (heading > 180) heading -= 360;
  input = heading - azimuth;
  if (input > 180) input -= 360;
  else if (input < -180) input += 360;
}

// ----- Read battery voltage
void measureBatteryVoltage() {
  float raw_voltage = analogRead(A6) * 5.01 / 1024.0;
  battVoltage = battVoltage * 0.95 + raw_voltage * 0.05;
}

// ----- Send data to master device
void sendBTData() {
  Serial.print("<"); //Message starting mark
  Serial.print(heading);
  Serial.print(",");
  Serial.print(battVoltage);
  Serial.print(",");
  Serial.print(Gxyz[2]);
  Serial.println(">"); //Message ending mark
}
