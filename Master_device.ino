// ----- Libraries
#include <NeoSWSerial.h>
#include <BMP180I2C.h>

// ----- Configure Bluetooth
NeoSWSerial BTPort(3, 2);

// ----- Barometer
BMP180I2C bmp180(0x77);
long sealevelPressure = 101325;
float altitude, bmp_pressure, bmp_temp;
bool bmp_started = false;

// ----- Information from slave device
float Heading = 0;
float GNSS_lat = 0;
float GNSS_lon = 0;
float GNSS_speed = 0;
float BatteryVoltage = 0;
int BatteryPercentage = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("\nREADY\n");

  //Start Bluetooth
  BTPort.begin(9600);

  //Start Bbrometer
  if (!bmp180.begin()) {
    Serial.println("BMP180 init failed.");
    while (1); //Halt
  }
  bmp180.resetToDefaults();
  bmp180.setSamplingMode(BMP180MI::MODE_UHR); //Ultra high resolution mode for pressure measurements
}

void loop() {
  //Get information from slave device
  getBTData();

  //Get BMP barometer readings
  getBMP(bmp_temp, bmp_pressure);
  altitude = 44330 * (1.0 - pow(bmp_pressure / sealevelPressure, 0.1903));

  //Calculate percentage
  BatteryPercentage = ((BatteryVoltage - 2.5) / 1.7) * 100;
  if (BatteryPercentage > 100) BatteryPercentage = 100;
  else if (BatteryPercentage < 0) BatteryPercentage = 0;

  Serial.print(F("Temp: "));
  Serial.print(bmp_temp);
  Serial.print(F(" | Pressure: "));
  Serial.print(bmp_pressure);
  Serial.print(F(" | Alt: "));
  Serial.print(altitude);
  Serial.print(F(" | Heading: "));
  Serial.print(Heading);
  Serial.print(F(" | Lat: "));
  Serial.print(GNSS_lat, 6);
  Serial.print(F(" | Long: "));
  Serial.print(GNSS_lon, 6);
  Serial.print(F(" | Speed: "));
  Serial.print(GNSS_speed);
  Serial.print(F(" | Batt(V): "));
  Serial.print(BatteryVoltage);
  Serial.print(F(" | Batt(%): "));
  Serial.println(BatteryPercentage);
  delay(50);
}

// ----- Read new data from slave device
void getBTData() {
  char * strtokIndex;
  char readDataCopy[80];
  char readData[80];
  char rch;
  int index = 0;
  bool reading = false;
  bool newData = false;

  while (BTPort.available() > 0) {
    rch = BTPort.read();
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
    if (atof(strtokIndex) != 0) Heading = atof(strtokIndex);
    strtokIndex = strtok(NULL, ",");
    if (atof(strtokIndex) != 0) GNSS_lat = atof(strtokIndex);
    strtokIndex = strtok(NULL, ",");
    if (atof(strtokIndex) != 0) GNSS_lon = atof(strtokIndex);
    strtokIndex = strtok(NULL, ",");
    if (atof(strtokIndex) != 0) GNSS_speed = atof(strtokIndex);
    strtokIndex = strtok(NULL, ",");
    if (atof(strtokIndex) != 0) BatteryVoltage = atof(strtokIndex);
    newData = false;
  }
  newData = false;
}

// ----- Request new data from BMP-180 barometer
void getBMP(float &temp, float &pressure) {
  if (!bmp_started) {
    bmp_started = true;
    bmp180.measureTemperature();
  }
  else if (bmp180.hasTemperatureValue()) {
    temp = bmp180.getTemperature() - 3;
    //Start measuring pressure ONLY AFTER temperature measurement has been finished
    bmp180.measurePressure();
  }
  else if (bmp180.hasPressureValue()) {
    bmp_started = false;
    pressure = bmp180.getPressure();
  }
}
