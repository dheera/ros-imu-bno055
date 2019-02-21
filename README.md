# ROS driver for IMU Bosch BNO055 (I2C)

This is a ROS node for the BNO055 IMU that communicates via I2C and without any dependencies besides libi2c-dev. It does **not** require RTIMULib, RTIMULib2, RTIMULib3 or whatever the latest sequel is. It is specifically targeted at using a BNO055 with NVIDIA boards such as the TX1, TX2, and Xavier, or any other board that has native I2C.

The BNO055 supports I2C and UART communication. This driver supports I2C only. If you are looking for a UART driver, see [this driver](https://github.com/mdrwiega/bosch_imu_driver) by [mdrwiega](https://github.com/mdrwiega) instead.

For use on a Raspberry Pi, you may need to [slow down the I2C clock](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching), since the Pi does not support clock stretching. but I have not tested this.

Note that you may need to add your user to the i2c group, e.g. `sudo usermod -aG i2c nvidia` if you are on a Jetson TX2.

## Parameters:

* **device** -- the path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** -- the i2c address of the IMU. Default is 0x28.

## Outputs topics:
* **/data** (sensor\_msgs/Imu) -- fused IMU data
* **/raw** (sensor\_msgs/Imu) -- raw accelerometer data
* **/mag** (sensor\_msgs/MagneticField) -- raw magnetic field data
* **/temp** (sensor\_msgs/Temperature) -- temperature data
* **/status** (diagnostic\_msgs/DiagnosticStatus) -- a DiagnosticStatus object showing the current calibration, interrupt, system status of the IMU

## Service calls:
* **/reset** (std\_srvs/Trigger) -- resets the IMU
* **/calibrate** (std\_srvs/Trigger) -- not yet implemented
