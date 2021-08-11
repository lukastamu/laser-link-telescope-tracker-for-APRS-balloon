# laser-link-telescope-tracker-for-APRS-balloon
Arduino based system created to rotate retroreflector prism on one axis.

# aim
Project's aim is to test laser link using high altitude weather balloon and retroreflector prism. Mirror telescope is used to track balloon and point laser straight to it.

# files
**libs** folder is necessary for `Gimbal.ino`. It contains MPU9250 libraries.

`Gimbal.ino` is Arduino Nano code for rotating part of weather balloon. Its task is to aim retroreflector prism to ground station (telescope).

`Master.ino` is Arduino Mega 2560 code for stationary part. It collects data from various sensors, gimbal and GNSS module, logs everything to SD card and sends some data to LightAPRS.

`APRS.ino` is LightAPRS code. It is modified code to receive telemetry data from I2C (including GNSS data) and transmit it to ground station. 
