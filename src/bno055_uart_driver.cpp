/* bno055_uart_driver.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 */

#include "imu_bno055/bno055_uart_driver.h"

namespace imu_bno055 {

BNO055UARTDriver::BNO055UARTDriver(std::string device_, int baud_rate_) {
    device = device_;
    baud_rate = baud_rate_;
}

bool BNO055UARTDriver::reset() {
    int i = 0;

    // Send the reset command via UART
    writeByte(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // reset
    writeByte(BNO055_SYS_TRIGGER_ADDR, 0x20);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // wait for chip to come back online
    while(readByte(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(i++ > 500) {
            throw std::runtime_error("chip did not come back online within 5 seconds of reset");
            return false;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // normal power mode
    writeByte(BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    writeByte(BNO055_PAGE_ID_ADDR, 0);
    writeByte(BNO055_SYS_TRIGGER_ADDR, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    writeByte(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    return true;
}

void BNO055UARTDriver::init() {

    file = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (file < 0) {
        throw std::runtime_error("UART device open failed");
    }

    configureUART();

    if (readByte(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        throw std::runtime_error("incorrect chip ID");
    }

    std::cerr << "rev ids:"
      << " accel:" << readByte(BNO055_ACCEL_REV_ID_ADDR)
      << " mag:" << readByte(BNO055_MAG_REV_ID_ADDR)
      << " gyro:" << readByte(BNO055_GYRO_REV_ID_ADDR)
      << " sw:" << readWord(BNO055_SW_REV_ID_LSB_ADDR)
      << " bl:" << readByte(BNO055_BL_REV_ID_ADDR) << std::endl;

    if (!reset()) {
        throw std::runtime_error("chip init failed");
    }
}

IMURecord BNO055UARTDriver::read() {
    IMURecord record;
    unsigned char c = 0;

    // read the data block via UART
    if (readBlock(BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
        throw std::runtime_error("read error");
    }
    if (readBlock(BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
        throw std::runtime_error("read error");
    }

    return record;
}

void BNO055UARTDriver::configureUART() {
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(file, &tty) != 0) {
        throw std::runtime_error("error from tcgetattr");
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(file, TCSANOW, &tty) != 0) {
        throw std::runtime_error("error from tcsetattr");
    }
}

void BNO055UARTDriver::writeByte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    if (write(file, buf, 2) != 2) {
        throw std::runtime_error("UART write failed");
    }
}

uint8_t BNO055UARTDriver::readByte(uint8_t reg) {
    write(file, &reg, 1);
    uint8_t value;
    if (read(file, &value, 1) != 1) {
        throw std::runtime_error("UART read failed");
    }
    return value;
}

uint16_t BNO055UARTDriver::readWord(uint8_t reg) {
    write(file, &reg, 1);
    uint8_t buf[2];
    if (read(file, buf, 2) != 2) {
        throw std::runtime_error("UART read failed");
    }
    return (buf[1] << 8) | buf[0];
}

int BNO055UARTDriver::readBlock(uint8_t start_reg, int length, uint8_t* buffer) {
    write(file, &start_reg, 1);
    return read(file, buffer, length);
}

}

