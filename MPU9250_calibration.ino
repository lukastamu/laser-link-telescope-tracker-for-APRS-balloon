#include <Wire.h>

// ----- Gyro
#define MPU9250_I2C_address 0x68                                        // I2C address for MPU9250 
#define MPU9250_I2C_master_enable 0x6A                                  // USER_CTRL[5] = I2C_MST_EN
#define MPU9250_Interface_bypass_mux_enable 0x37                        // INT_PIN_CFG[1]= BYPASS_EN

#define Frequency 125                                                   // X mS sample interval 
#define Sensitivity 65.5                                                // Gyro sensitivity (see data sheet)

#define Sensor_to_deg 1/(Sensitivity*Frequency)                         // Convert sensor reading to degrees
#define Sensor_to_rad Sensor_to_deg*DEG_TO_RAD                          // Convert sensor reading to radians

#define Loop_time 1000000/Frequency                                     // Loop time (uS)
long    Loop_start;                                                     // Loop start time (uS)

int     Gyro_x,     Gyro_y,     Gyro_z;
float   Gyro_pitch, Gyro_roll, Gyro_yaw;
float   Gyro_pitch_output, Gyro_roll_output;

// ----- Gyro calibration data
long    Gyro_x_offset = 0,     Gyro_y_offset = 0,      Gyro_z_offset = 0;

// ----- Accelerometer
long    Accel_x,      Accel_y,      Accel_z,    Accel_total_vector;
float   Accel_pitch,  Accel_roll;

// ----- Accelerometer calibration data
int     Accel_x_offset = 0,   Accel_y_offset = 0,   Accel_z_offset = 0;
float   Accel_x_scale = 0,    Accel_y_scale = 0,    Accel_z_scale = 0;

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

// ----- Magnetometer calibration data
int     Mag_x_offset = 0,      Mag_y_offset = 0,     Mag_z_offset = 0;
float   Mag_x_scale = 0,       Mag_y_scale = 0,      Mag_z_scale = 0;
float   ASAX = 0,              ASAY = 0,             ASAZ = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  configure_magnetometer();
  config_gyro_accel();

  //Calibrate sensors
  Serial.begin(115200);
  delay(500);
  Serial.println(F("Gyro calibration will start in 5 seconds..."));
  delay(5000);
  calibrate_gyro();
  calibrate_magnetometer();
  calibrate_accelerometer();
  while (1); // Wheelspin ... program halt
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

// ----- Calibrate accelerometer, get offsets and scale factors
void calibrate_accelerometer() {
  //Locals
  long accel_x, accel_y, accel_z;
  long accel_x_min =  32767;
  long accel_y_min =  32767;
  long accel_z_min =  32767;
  long accel_x_max = -32768;
  long accel_y_max = -32768;
  long accel_z_max = -32768;

  float chord_x,  chord_y,  chord_z;
  float chord_average;

  //Display calibration message
  Serial.println(F("\nAccelerometer calibration will start in 5 seconds..."));
  delay(5000);
  Serial.println(F("Rotate sensor carefully and slowly in 3D till done"));

  //Record min/max XYZ compass readings
  for (int counter = 0; counter < 16000 ; counter ++) {
    Loop_start = micros();
    if (counter % 1000 == 0)Serial.println(F("."));

    //Point to data
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();

    //Read the data
    Wire.requestFrom(0x68, 14);
    while (Wire.available() < 14);
    accel_x = Wire.read() << 8 | Wire.read();
    accel_y = Wire.read() << 8 | Wire.read();
    accel_z = Wire.read() << 8 | Wire.read();

    //Find max/min xyz values
    accel_x_min = min(accel_x, accel_x_min);
    accel_x_max = max(accel_x, accel_x_max);
    accel_y_min = min(accel_y, accel_y_min);
    accel_y_max = max(accel_y, accel_y_max);
    accel_z_min = min(accel_z, accel_z_min);
    accel_z_max = max(accel_z, accel_z_max);

    delay(4);
  }

  //Calculate hard-iron offsets
  Accel_x_offset = (accel_x_max + accel_x_min) / 2;
  Accel_y_offset = (accel_y_max + accel_y_min) / 2;
  Accel_z_offset = (accel_z_max + accel_z_min) / 2;

  //Calculate soft-iron scale factors
  chord_x = ((float)(accel_x_max - accel_x_min)) / 2;
  chord_y = ((float)(accel_y_max - accel_y_min)) / 2;
  chord_z = ((float)(accel_z_max - accel_z_min)) / 2;

  chord_average = (chord_x + chord_y + chord_z) / 3;

  Accel_x_scale = chord_average / chord_x;
  Accel_y_scale = chord_average / chord_y;
  Accel_z_scale = chord_average / chord_z;

  //Record accelerometer offsets
  //Display data extremes
  Serial.print(F("XYZ Max/Min: "));
  Serial.print(accel_x_min); Serial.print(F("\t"));
  Serial.print(accel_x_max); Serial.print(F("\t"));
  Serial.print(accel_y_min); Serial.print(F("\t"));
  Serial.print(accel_y_max); Serial.print(F("\t"));
  Serial.print(accel_z_min); Serial.print(F("\t"));
  Serial.println(accel_z_max);
  Serial.println("");

  //Display hard-iron offsets
  Serial.print(F("Offsets: "));
  Serial.print(Accel_x_offset); Serial.print(F("\t"));
  Serial.print(Accel_y_offset); Serial.print(F("\t"));
  Serial.println(Accel_z_offset);
  Serial.println("");

  //Display soft-iron scale factors
  Serial.print(F("Scale fators: "));
  Serial.print(Accel_x_scale); Serial.print(F("\t"));
  Serial.print(Accel_y_scale); Serial.print(F("\t"));
  Serial.println(Accel_z_scale);
  Serial.println("");
}

// ----- Calibrate gyro, get offsets
void calibrate_gyro() {
  // ----- Display calibration message
  Serial.println(F("Calibrating gyro"));

  // ----- Calibrate gyro
  for (int counter = 0; counter < 1000 ; counter ++)    //Run this code 1000 times
  {
    Loop_start = micros();
    if (counter % 125 == 0)Serial.println(F("."));
    read_mpu_6050_data();                               //Read the raw acc and gyro data from the MPU-6050
    Gyro_x_offset += Gyro_x;                               //Add the gyro x-axis offset to the gyro_x_offset variable
    Gyro_y_offset += Gyro_y;                               //Add the gyro y-axis offset to the gyro_y_offset variable
    Gyro_z_offset += Gyro_z;                               //Add the gyro z-axis offset to the gyro_z_offset variable
    while (micros() - Loop_start < Loop_time);           // Wait until "Loop_time" microseconds have elapsed
  }
  Gyro_x_offset /= 1000;                                   //Divide the gyro_x_offset variable by 2000 to get the average offset
  Gyro_y_offset /= 1000;                                   //Divide the gyro_y_offset variable by 2000 to get the average offset
  Gyro_z_offset /= 1000;                                   //Divide the gyro_z_offset variable by 2000 to get the average offset
  //Display hard-iron offsets
  Serial.print(F("Offsets: "));
  Serial.print(Gyro_x_offset); Serial.print(F("\t"));
  Serial.print(Gyro_y_offset); Serial.print(F("\t"));
  Serial.println(Gyro_z_offset);
  Serial.println("");
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

// ----- Calibrate magnetometer, get offsets and scale factors
void calibrate_magnetometer() {
  //Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;
  int mag_x_min =  32767;
  int mag_y_min =  32767;
  int mag_z_min =  32767;
  int mag_x_max = -32768;
  int mag_y_max = -32768;
  int mag_z_max = -32768;
  float chord_x,  chord_y,  chord_z;
  float chord_average;

  //Display calibration message
  Serial.println("\nMagnetometer calibration will start in 5 seconds...");
  delay(5000);
  Serial.println("Rotate sensor in 8 figure till done");

  //Record min/max XYZ compass readings
  for (int counter = 0; counter < 16000 ; counter ++) {
    Loop_start = micros();
    if (counter % 1000 == 0)Serial.println(".");

    //Point to status register 1
    Wire.beginTransmission(AK8963_I2C_address);
    Wire.write(AK8963_status_reg_1);
    Wire.endTransmission();
    Wire.requestFrom(AK8963_I2C_address, 1);
    while (Wire.available() < 1);
    if (Wire.read() & AK8963_data_ready_mask) {
      //Read data from each axis (LSB,MSB)
      Wire.requestFrom(AK8963_I2C_address, 7);
      while (Wire.available() < 7);
      mag_x = (Wire.read() | Wire.read() << 8) * ASAX;
      mag_y = (Wire.read() | Wire.read() << 8) * ASAY;
      mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;
      status_reg_2 = Wire.read();

      //Validate data
      if (!(status_reg_2 & AK8963_overflow_mask)) {
        //Find max/min xyz values
        mag_x_min = min(mag_x, mag_x_min);
        mag_x_max = max(mag_x, mag_x_max);
        mag_y_min = min(mag_y, mag_y_min);
        mag_y_max = max(mag_y, mag_y_max);
        mag_z_min = min(mag_z, mag_z_min);
        mag_z_max = max(mag_z, mag_z_max);
      }
    }
    delay(4);
  }

  //Calculate hard-iron offsets
  Mag_x_offset = (mag_x_max + mag_x_min) / 2;
  Mag_y_offset = (mag_y_max + mag_y_min) / 2;
  Mag_z_offset = (mag_z_max + mag_z_min) / 2;

  //Calculate soft-iron scale factors
  chord_x = ((float)(mag_x_max - mag_x_min)) / 2;
  chord_y = ((float)(mag_y_max - mag_y_min)) / 2;
  chord_z = ((float)(mag_z_max - mag_z_min)) / 2;

  chord_average = (chord_x + chord_y + chord_z) / 3;

  Mag_x_scale = chord_average / chord_x;
  Mag_y_scale = chord_average / chord_y;
  Mag_z_scale = chord_average / chord_z;

  //Record magnetometer offsets
  //Display data extremes
  Serial.print(F("XYZ Max/Min: "));
  Serial.print(mag_x_min); Serial.print(F("\t"));
  Serial.print(mag_x_max); Serial.print(F("\t"));
  Serial.print(mag_y_min); Serial.print(F("\t"));
  Serial.print(mag_y_max); Serial.print(F("\t"));
  Serial.print(mag_z_min); Serial.print(F("\t"));
  Serial.println(mag_z_max);
  Serial.println("");

  //Display hard-iron offsets
  Serial.print(F("Offsets: "));
  Serial.print(Mag_x_offset); Serial.print(F("\t"));
  Serial.print(Mag_y_offset); Serial.print(F("\t"));
  Serial.println(Mag_z_offset);
  Serial.println("");

  //Display soft-iron scale factors
  Serial.print(F("Scale fators: "));
  Serial.print(Mag_x_scale); Serial.print(F("\t"));
  Serial.print(Mag_y_scale); Serial.print(F("\t"));
  Serial.println(Mag_z_scale);
  Serial.println("");

  //Display fuse ROM values
  Serial.print(F("ASA: "));
  Serial.print(ASAX); Serial.print(F("\t"));
  Serial.print(ASAY); Serial.print(F("\t"));
  Serial.println(ASAZ);
}
