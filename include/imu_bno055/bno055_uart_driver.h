/* bno055_uart_driver.h
 * Author: Dheera Venkatraman <dheera@dheera.net>
 */

#ifndef IMU_BNO055_UART_DRIVER_H
#define IMU_BNO055_UART_DRIVER_H

#include <string>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include "imu_bno055/imu_record.h"

namespace imu_bno055 {

class BNO055UARTDriver {
public:
    BNO055UARTDriver(std::string device_, int baud_rate_);
    void init();
    bool reset();
    IMURecord read();

private:
    void configureUART();
    void writeByte(uint8_t reg, uint8_t value);
    uint8_t readByte(uint8_t reg);
    uint16_t readWord(uint8_t reg);
    int readBlock(uint8_t start_reg, int length, uint8_t* buffer);

    std::string device;
    int baud_rate;
    int file;
};

}

#endif // IMU_BNO055_UART_DRIVER_H

