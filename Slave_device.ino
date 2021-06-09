// ----- Libraries
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>

// ----- Gyro
#define MPU9250_I2C_address 0x68                                        // I2C address for MPU9250 
#define MPU9250_I2C_master_enable 0x6A                                  // USER_CTRL[5] = I2C_MST_EN
#define MPU9250_Interface_bypass_mux_enable 0x37                        // INT_PIN_CFG[1]= BYPASS_EN

#define Frequency 1000                                                  // 1 mS sample interval 
#define Sensitivity 65.5                                                // Gyro sensitivity (see data sheet)

#define Sensor_to_deg 1/(Sensitivity*Frequency)                         // Convert sensor reading to degrees
#define Sensor_to_rad Sensor_to_deg*DEG_TO_RAD                          // Convert sensor reading to radians

#define Loop_time 1000000/Frequency                                     // Loop time (uS)
long    Loop_start;                                                     // Loop start time (uS)

int     Gyro_x,     Gyro_y,     Gyro_z;
float   Gyro_pitch, Gyro_roll, Gyro_yaw;
float   Gyro_pitch_output, Gyro_roll_output;

// ----- Accelerometer
long    Accel_x,      Accel_y,      Accel_z,    Accel_total_vector;
float   Accel_pitch,  Accel_roll;

// ----- Magnetometer
#define AK8963_I2C_address 0x0C                                          // I2C address for AK8963
#define AK8963_cntrl_reg_1 0x0A                                          // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02                                         // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001                                // Data ready mask
#define AK8963_overflow_mask 0b00001000                                  // Magnetic sensor overflow mask
#define AK8963_data 0x03                                                 // Start address of XYZ data                                                                
#define AK8963_fuse_ROM 0x10                                             // X,Y,Z fuse ROM

// ----- Mgnetic declination. http://www.magnetic-declination.com/
float   Declination = +8.5;

int     Mag_x,                Mag_y,                Mag_z;
float   Mag_x_dampened,       Mag_y_dampened,       Mag_z_dampened;
float   Mag_x_hor, Mag_y_hor;
float   Mag_pitch, Mag_roll;

// ----- CALIBRATION VALUES
// ----- Magnetometer calibration data
int     Mag_x_offset = 70,      Mag_y_offset = 342,     Mag_z_offset = -365;
float   Mag_x_scale = 1.12,     Mag_y_scale = 1.09,     Mag_z_scale = 0.84;
float   ASAX = 1.18,            ASAY = 1.17,            ASAZ = 1.13;           // (A)sahi (S)ensitivity (A)djustment fuse ROM values.

// ----- Accelerometer calibration data
int     Accel_x_offset = 101,   Accel_y_offset = 470,   Accel_z_offset = 30;
float   Accel_x_scale = 0.99,  Accel_y_scale = 1.03,   Accel_z_scale = 0.98;

// ----- Gyro calibration data
long    Gyro_x_offset = -115,     Gyro_y_offset = 95,      Gyro_z_offset = -2;

// ----- Flags
bool Gyro_synchronised = false;
bool Tilt_compensation_OFF = false;

// ----- Configure GNSS ports and other
NeoSWSerial gnssPort(3, 2); // RX TX
NMEAGPS  gnss; // This parses the GPS characters

// ----- Static telescope coordinates
float tele_lat = 54.810128;
float tele_lon = 25.416873;

// ----- GNSS variables
float GNSS_lat = 0;
float GNSS_lon = 0;
float GNSS_speed = 0;

// ----- Servo
Servo servo;

// ----- PID
float diff;
double setpoint = 0;
double input = 0;
double output = 0;
//Aggresive values
double ap = 0.8;
double ai = 0;
double ad = 0.0035;
//Concervative values
double cp = 0.55;
double ci = 0.01;
double cd = 0.0035;
PID thePID(&input, &output, &setpoint, ap, ai, ad, DIRECT);

// ----- Compass dampening - what fraction of recent value to use. Value from 0 to 1
float m_damp = 0.4;

// ----- Battery voltage
float BatteryVoltage = 0;

// ----- MISC
float toDeg = 180 / PI;
float toRad = PI / 180;
long Loop_count = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  gnssPort.begin(9600);

  servo.attach(5);

  thePID.SetMode(AUTOMATIC);
  thePID.SetOutputLimits(-90, 90);
  thePID.SetSampleTime(1);

  configure_magnetometer();
  config_gyro_accel();

  Serial.begin(9600);
}

void loop() {
  //Get correct input, setpoint always stays zero
  calculateInput(calculateAzimuth(), calculateHeading());

  diff = abs(setpoint - input);
  if (diff <= 24) thePID.SetTunings(cp, ci, cd);
  else thePID.SetTunings(ap, ai, ad);
  thePID.Compute();

  //Move servo accordingly
  if (GNSS_lat != 0 && GNSS_lon != 0) servo.write(output + 92);

  if (Loop_count < 100) Loop_count++;
  else {
    //Send data to master device
    sendBTData();
    Loop_count = 0;
  }

  delay(1);
}

// ----- Get heading from compass
float calculateHeading() {
  //Component side up, X-axis facing forward
  float heading = 0;

  read_mpu_6050_data();

  //Apply gyro corrections
  Gyro_x -= Gyro_x_offset;
  Gyro_y -= Gyro_y_offset;
  Gyro_z -= Gyro_z_offset;

  //Calculate travelled angles
  Gyro_pitch += -Gyro_y * Sensor_to_deg;
  Gyro_roll += Gyro_x * Sensor_to_deg;
  Gyro_yaw += -Gyro_z * Sensor_to_deg;

  //Compensate pitch and roll for gyro yaw
  Gyro_pitch += Gyro_roll * sin(Gyro_z * Sensor_to_rad);            // Transfer the roll angle to the pitch angle if the Z-axis has yawed
  Gyro_roll -= Gyro_pitch * sin(Gyro_z * Sensor_to_rad);            // Transfer the pitch angle to the roll angle if the Z-axis has yawed

  //Accelerometer angle calculations
  Accel_total_vector = sqrt((Accel_x * Accel_x) + (Accel_y * Accel_y) + (Accel_z * Accel_z));   // Calculate the total (3D) vector
  Accel_pitch = asin((float)Accel_x / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the pitch angle
  Accel_roll = asin((float)Accel_y / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the roll angle

  //Zero any residual accelerometer readings
  Accel_pitch += 3.2;                                             //Accelerometer calibration value for pitch
  Accel_roll += 3.9;                                               //Accelerometer calibration value for roll

  //Correct for any gyro drift
  if (Gyro_synchronised) {
    //Gyro & accel have been synchronised
    Gyro_pitch = Gyro_pitch * 0.9 + Accel_pitch * 0.1;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    Gyro_roll = Gyro_roll * 0.9 + Accel_roll * 0.1;           //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else {
    //Synchronise gyro & accel
    Gyro_pitch = Accel_pitch;                                       //Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll = Accel_roll;                                         //Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_synchronised = true;                                       //Set the IMU started flag
  }

  //Dampen the pitch and roll angles
  Gyro_pitch_output = Gyro_pitch_output * 0.9 + Gyro_pitch * 0.1;
  Gyro_roll_output = Gyro_roll_output * 0.9 + Gyro_roll * 0.1;

  //Read the magnetometer
  read_magnetometer();

  //Fix the pitch, roll, & signs
  Mag_pitch = -Gyro_roll_output * DEG_TO_RAD;
  Mag_roll = Gyro_pitch_output * DEG_TO_RAD;

  //Apply the standard tilt formulas
  Mag_x_hor = Mag_x * cos(Mag_pitch) + Mag_y * sin(Mag_roll) * sin(Mag_pitch) - Mag_z * cos(Mag_roll) * sin(Mag_pitch);
  Mag_y_hor = Mag_y * cos(Mag_roll) + Mag_z * sin(Mag_roll);

  //Disable tilt stabization if Tilt_compensation_OFF closed
  if (Tilt_compensation_OFF) {
    // ---- Test equations
    Mag_x_hor = Mag_x;
    Mag_y_hor = Mag_y;
  }

  //Dampen some data fluctuations
  Mag_x_dampened = Mag_x_dampened * (1 - m_damp) + Mag_x_hor * m_damp;
  Mag_y_dampened = Mag_y_dampened * (1 - m_damp) + Mag_y_hor * m_damp;

  //Calculate the heading
  heading = atan2(Mag_x_dampened, Mag_y_dampened) * RAD_TO_DEG;  // Magnetic North
  heading += Declination;
  if (heading > 360.0) heading -= 360.0;
  if (heading < 0.0) heading += 360.0;

  //Allow for under/overflow
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;

  return heading;
}

// ----- Cofigure MPU6050
void config_gyro_accel() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x6B);                                     //Point to power management register
  Wire.write(0x00);                                     //Use internal 20MHz clock
  Wire.endTransmission();                               //End the transmission

  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1C);                                     //Point to accelerometer configuration reg
  Wire.write(0x10);                                     //Select +/-8g full-scale
  Wire.endTransmission();                               //End the transmission

  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1B);                                     //Point to gyroscope configuration
  Wire.write(0x08);                                     //Select 500dps full-scale
  Wire.endTransmission();                               //End the transmission
}

// ----- Read gyro and accelerometer data
void read_mpu_6050_data() {
  //Locals
  int temperature;                                  // Needed when reading the MPU-6050 data ... not used
  long accel_x, accel_y, accel_z;

  //Point to data
  Wire.beginTransmission(0x68);                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                     // Point to start of data
  Wire.endTransmission();                               // End the transmission

  //Read the data
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  accel_x = Wire.read() << 8 | Wire.read();
  accel_y = Wire.read() << 8 | Wire.read();
  accel_z = Wire.read() << 8 | Wire.read();
  //Apply accelerometer corrections
  Accel_x = (accel_x - Accel_x_offset) * Accel_x_scale;
  Accel_y = (accel_y - Accel_y_offset) * Accel_y_scale;
  Accel_z = (accel_z - Accel_z_offset) * Accel_z_scale;

  temperature = Wire.read() << 8 | Wire.read();

  Gyro_x = Wire.read() << 8 | Wire.read();
  Gyro_y = Wire.read() << 8 | Wire.read();
  Gyro_z = Wire.read() << 8 | Wire.read();
}

// ----- Configure AK8963
void configure_magnetometer() {
  /*
     The MPU-9250 contains an AK8963 magnetometer and an
     MPU-6050 gyro/accelerometer within the same package.

     To access the AK8963 magnetometer chip The MPU-9250 I2C bus
     must be changed to pass-though mode. To do this we must:
      - disable the MPU-9250 slave I2C and
      - enable the MPU-9250 interface bypass mux
  */
  //Disable MPU9250 I2C master interface
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_I2C_master_enable);                            // Point USER_CTRL[5] = I2C_MST_EN
  Wire.write(0x00);                                                 // Disable the I2C master interface
  Wire.endTransmission();

  //Enable MPU9250 interface bypass mux
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_Interface_bypass_mux_enable);                  // Point to INT_PIN_CFG[1] = BYPASS_EN
  Wire.write(0x02);                                                 // Enable the bypass mux
  Wire.endTransmission();

  //Access AK8963 fuse ROM
  /* The factory sensitivity readings for the XYZ axes are stored in a fuse ROM.
     To access this data we must change the AK9863 operating mode.
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // CNTL[3:0] mode bits
  Wire.write(0b00011111);                                           // Output data=16-bits; Access fuse ROM
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  //Get factory XYZ sensitivity adjustment values from fuse ROM
  /* There is a formula on page 53 of "MPU-9250, Register Map and Decriptions, Revision 1.4":
      Hadj = H*(((ASA-128)*0.5)/128)+1 where
      H    = measurement data output from data register
      ASA  = sensitivity adjustment value (from fuse ROM)
      Hadj = adjusted measurement data (after applying)
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_fuse_ROM);                                      // Point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 3);                          // Request 3 bytes of data
  while (Wire.available() < 3);                                     // Wait for the data
  ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;                       // Adjust data
  ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;

  //Power down AK8963 while the mode is changed
  /*
     This wasn't necessary for the first mode change as the chip was already powered down
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00000000);                                           // Set mode to power down
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  //Set output to mode 2 (16-bit, 100Hz continuous)
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00010110);                                           // Output=16-bits; Measurements = 100Hz continuous
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change
}

// ----- Read magnetometer data
void read_magnetometer() {
  //Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;

  //Point to status register 1
  Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
  Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
  while (Wire.available() < 1);                                 // Wait for the data
  if (Wire.read() & AK8963_data_ready_mask) {
    //Read data from each axis (LSB,MSB)
    Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
    while (Wire.available() < 7);                               // Wait for the data
    mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
    mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
    mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
    status_reg_2 = Wire.read();                                 // Read status and signal data read

    //Validate data
    if (!(status_reg_2 & AK8963_overflow_mask)) {
      Mag_x = (mag_x - Mag_x_offset) * Mag_x_scale;
      Mag_y = (mag_y - Mag_y_offset) * Mag_y_scale;
      Mag_z = (mag_z - Mag_z_offset) * Mag_z_scale;
    }
  }
}

// ----- Request new data from GNSS module
void updateGNSS() {
  while (gnss.available(gnssPort)) {
    gps_fix  fix = gnss.read();
    if (fix.valid.location) {
      GNSS_lat = fix.latitude();
      GNSS_lon = fix.longitude();
    }
    if (fix.valid.speed) GNSS_speed = fix.speed_kph();
  }
}

// ----- Calculate azmiuth from telescope and balloon coordinates
float calculateAzimuth() {
  updateGNSS();
  float azimuth;
  float fi_1 = GNSS_lat * toRad;
  float lambda_1 = GNSS_lon * toRad;
  float fi_2 = tele_lat * toRad;
  float lambda_2 = tele_lon * toRad;
  //Calculate gps based azimuth
  azimuth = atan2(sin(lambda_2 - lambda_1) * cos(fi_2), cos(fi_1) * sin(fi_2) - sin(fi_1) * cos(fi_2) * cos(lambda_2 - lambda_1)) * toDeg;
  if (azimuth < 0) azimuth += 360;
  return azimuth;
}

// ----- Offset azimuth and heading correctly
void calculateInput(float azimuth, float heading) {
  if (azimuth > 180) azimuth -= 360;
  if (heading > 180) heading -= 360;
  input = heading - azimuth;
  if (input > 180) input -= 360;
  else if (input < -180) input += 360;
}

// ----- Get battery voltage reading
void measureBatteryVoltage() {
  float raw_voltage = analogRead(A6) * 5.15 / 1024;
  BatteryVoltage = BatteryVoltage * 0.99 + raw_voltage * 0.01;
}

// ----- Send data to master device
void sendBTData() {
  measureBatteryVoltage();
  Serial.print("<"); //Message starting mark
  Serial.print(calculateHeading());
  Serial.print(",");
  Serial.print(GNSS_lat, 6);
  Serial.print(",");
  Serial.print(GNSS_lon, 6);
  Serial.print(",");
  Serial.print(GNSS_speed);
  Serial.print(",");
  Serial.print(BatteryVoltage);
  Serial.print(">"); //Message ending mark
}
