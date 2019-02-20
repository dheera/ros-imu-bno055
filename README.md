# imu\_bno055\_i2c

This is a ROS node for the BNO055 IMU that communicates via I2C and without any dependencies besides libi2c-dev. It does **not** require RTIMULib, RTIMULib2, RTIMULib3 or whatever the latest sequel is. It is specifically targeted at using a BNO055 with NVIDIA boards such as the TX1, TX2, and Xavier, or any other board that has native I2C.

It is unlikely to work with a Raspberry Pi as-is since the Pi does not support clock stretching. You may have luck by [slowing down the I2C clock](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching), but I have not tested this.
