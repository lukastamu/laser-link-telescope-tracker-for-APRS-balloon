# laser-link-telescope-tracker-for-APRS-balloon
Arduino based system created to rotate retroreflector prism on one axis.

# aim
Project's aim is to test laser link using high altitude weather balloon and retroreflector prism. Mirror telescope is used to track balloon and point laser straight to it.

# files
File `Slave_device.ino` is written for Arduino nano rotary platform. Platform includes MPU9250 9-axis accel/gyro/mag sensor, u-blox M8 GNSS module, HC-05 Bluetooth module, continuous servo.
Platform's task is to compensate for weather balloon's horizontal rotation (due to winds) and keep retroreflector prism facing telescope and laser.
It works by calculating angle using GNSS coordinates, static telescope coordinates and azimuth from magnetometer.

File `Master_device.ino` is intended for balloon's stacionary part where Arduino Uno is constantly collecting information from rotary platform through Bluetooth.
It also measures temperature and barometric altitude. Redundancy GNSS module and motion sensor will be added in future.

File `MPU9250_calibration` is used to calibrate MPU9250 sensor. It prints out offsets and scale factors for gyro, accelerometer and magnetometer.
Calibration must be done with platform fully assembled.
