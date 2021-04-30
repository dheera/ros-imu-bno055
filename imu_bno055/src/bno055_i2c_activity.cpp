/* bno055_i2c_activity.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Defines a BNO055I2C Activity class, constructed with node handles
 * and which handles all ROS duties.
 */

#include "imu_bno055/bno055_i2c_activity.h"
#include <linux/i2c.h>

namespace imu_bno055 {

// ******** constructors ******** //

BNO055I2CActivity::BNO055I2CActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
  nh(_nh), nh_priv(_nh_priv) {
    ROS_INFO("initializing");
    nh_priv.param("device", param_device, (std::string)"/dev/i2c-1");
    nh_priv.param("address", param_address, (int)BNO055_ADDRESS_A);
    nh_priv.param("frame_id", param_frame_id, (std::string)"imu");
    nh_priv.param("operation_mode", param_operation_mode, (std::string)"NDOF");
    nh_priv.param("acc_bandwidth", param_acc_bandwidth, 62.5);
    nh_priv.param("acc_range", param_acc_range, 4);
    nh_priv.param("gyro_bandwidth", param_gyro_bandwidth, 32.0);
    nh_priv.param("gyro_range", param_gyro_range, 2000);

    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);
}

// ******** private methods ******** //

int BNO055I2CActivity::operation_mode() {
    std::map<std::string, int> mode_map {
        { "ACCONLY", BNO055_OPERATION_MODE_ACCONLY },
        { "MAGONLY", BNO055_OPERATION_MODE_MAGONLY },
        { "GYRONLY", BNO055_OPERATION_MODE_GYRONLY },
        { "ACCMAG", BNO055_OPERATION_MODE_ACCMAG },
        { "ACCGYRO", BNO055_OPERATION_MODE_ACCGYRO },
        { "MAGGYRO", BNO055_OPERATION_MODE_MAGGYRO },
        { "AMG", BNO055_OPERATION_MODE_AMG },
        { "IMU", BNO055_OPERATION_MODE_IMUPLUS },
        { "COMPASS", BNO055_OPERATION_MODE_COMPASS },
        { "M4G", BNO055_OPERATION_MODE_M4G },
        { "NDOF_FMC_OFF", BNO055_OPERATION_MODE_NDOF_FMC_OFF },
        { "NDOF", BNO055_OPERATION_MODE_NDOF },
    };

    if (mode_map.count(param_operation_mode)) {
        ROS_INFO_STREAM("BNO055 operation mode: " << param_operation_mode);
        return mode_map.at(param_operation_mode);
    }

    return -1;
}

bool BNO055I2CActivity::configure_sensors() {
    std::map<double, uint8_t> acc_bw {
        { 7.81, 0 },
        { 15.63, 1 },
        { 31.25, 2 },
        { 62.5, 3 },
        { 125.0, 4 },
        { 250.0, 5 },
        { 500.0, 6 },
        { 1000.0, 7 },
    };

    std::map<int, uint8_t> acc_range {
        { 2, 0 },
        { 4, 1 },
        { 8, 2 },
        { 16, 3 },
    };

    std::map<double, uint8_t> gyro_bw {
        { 523.0, 0 },
        { 230.0, 1 },
        { 116.0, 2 },
        { 47.0, 3 },
        { 23.0, 4 },
        { 12.0, 5 },
        { 64.0, 6 },
        { 32.0, 7 },
    };

    std::map<int, uint8_t> gyro_range {
        { 2000, 0 },
        { 1000, 1 },
        { 500, 2 },
        { 250, 3 },
        { 125, 4 },
    };

    if (!acc_bw.count(param_acc_bandwidth)) {
        ROS_ERROR_STREAM("Unsupported acc_bandwidth: " << param_acc_bandwidth);
        return false;
    }

    if (!acc_range.count(param_acc_range)) {
        ROS_ERROR_STREAM("Unsupported acc_range: " << param_acc_range);
        return false;
    }

    if (!gyro_bw.count(param_gyro_bandwidth)) {
        ROS_ERROR_STREAM("Unsupported gyro_bandwidth: " << param_gyro_bandwidth);
        return false;
    }

    if (!gyro_range.count(param_gyro_range)) {
        ROS_ERROR_STREAM("Unsupported gyro_range: " << param_gyro_range);
        return false;
    }

    ROS_INFO_STREAM("Accelerometer range: +/-" << param_acc_range << "G, bandwidth: " << param_acc_bandwidth << "Hz");
    ROS_INFO_STREAM("Gyroscope range: +/-" << param_gyro_range << "dps, bandwidth: " << param_gyro_bandwidth << "Hz");

    uint8_t acc = acc_range.at(param_acc_range) | (acc_bw.at(param_acc_bandwidth) << 2);
    uint8_t gyro = gyro_range.at(param_gyro_range) | (gyro_bw.at(param_gyro_bandwidth) << 3);

    _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 1);

    _i2c_smbus_write_byte_data(file, BNO055_ACC_CONFIG, acc);
    _i2c_smbus_write_byte_data(file, BNO055_GYR_CONFIG_0, gyro);

    _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);

    return true;
}

bool BNO055I2CActivity::reset() {
    int i = 0;

    int opr_mode = operation_mode();
    if (opr_mode < 0) {
        ROS_ERROR_STREAM("Unknown operation_mode: " << param_operation_mode);
        return false;
    }

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    ros::Duration(0.025).sleep();

    // reset
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
    ros::Duration(0.025).sleep();

    // wait for chip to come back online
    while(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        ros::Duration(0.010).sleep();
        if(i++ > 500) {
            ROS_ERROR_STREAM("chip did not come back online in 5 seconds after reset");
            return false;
        }
    }
    ros::Duration(0.100).sleep();

    // normal power mode
    _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    ros::Duration(0.010).sleep();

    _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, (uint8_t)opr_mode);
    ros::Duration(0.025).sleep();

    if (opr_mode < BNO055_OPERATION_MODE_IMUPLUS) {
        if (!configure_sensors()) {
            return false;
        }
    }

    return true;
}

// ******** public methods ******** //

bool BNO055I2CActivity::start() {
    ROS_INFO("starting");

    if(!pub_data) pub_data = nh.advertise<sensor_msgs::Imu>("data", 1);
    if(!pub_raw) pub_raw = nh.advertise<sensor_msgs::Imu>("raw", 1);
    if(!pub_mag) pub_mag = nh.advertise<sensor_msgs::MagneticField>("mag", 1);
    if(!pub_temp) pub_temp = nh.advertise<sensor_msgs::Temperature>("temp", 1);
    if(!pub_status) pub_status = nh.advertise<diagnostic_msgs::DiagnosticStatus>("status", 1);

    if(!service_calibrate) service_calibrate = nh.advertiseService(
        "calibrate",
        &BNO055I2CActivity::onServiceCalibrate,
        this
    );

    if(!service_reset) service_reset = nh.advertiseService(
        "reset",
        &BNO055I2CActivity::onServiceReset,
        this
    );

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        ROS_ERROR("i2c device open failed");
        return false;
    }

    if(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        ROS_ERROR("incorrect chip ID");
        return false;
    }

    ROS_INFO_STREAM("rev ids:"
      << " accel:" << _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
      << " mag:" << _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
      << " gyro:" << _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
      << " sw:" << _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
      << " bl:" << _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR));

    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }

    return true;
}

bool BNO055I2CActivity::spinOnce() {
    ros::spinOnce();

    ros::Time time = ros::Time::now();

    IMURecord record;

    unsigned char c = 0;

    seq++;

    unsigned char reg = BNO055_ACCEL_DATA_X_LSB_ADDR;

    struct i2c_msg msgs[] {
        {
            .addr = (uint16_t)param_address,
            .flags = 0,
            .len = 1,
            .buf = &reg,
        },
        {
            // TODO: I2C_M_NOSTART would be nice together with I2C_M_RD but it seems to be not supported by i2c-gpio.c (I2C_FUNC_NOSTART).
            .addr = (uint16_t)param_address,
            .flags = I2C_M_RD,
            .len = sizeof(record),
            .buf = (unsigned char *)&record,
        },
    };

    struct i2c_rdwr_ioctl_data msgset {
        .msgs = msgs,
        .nmsgs = sizeof(msgs)/sizeof(*msgs),
    };

    if (ioctl(file, I2C_RDWR, &msgset) < 0)
        return false;

    sensor_msgs::Imu msg_raw;
    msg_raw.header.stamp = time;
    msg_raw.header.frame_id = param_frame_id;
    msg_raw.header.seq = seq;
    msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
    msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
    msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
    msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::MagneticField msg_mag;
    msg_mag.header.stamp = time;
    msg_mag.header.frame_id = param_frame_id;
    msg_mag.header.seq = seq;
    msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
    msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
    msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;

    sensor_msgs::Imu msg_data;
    msg_data.header.stamp = time;
    msg_data.header.frame_id = param_frame_id;
    msg_data.header.seq = seq;

    double fused_orientation_norm = std::pow(
      std::pow(record.fused_orientation_w, 2) +
      std::pow(record.fused_orientation_x, 2) +
      std::pow(record.fused_orientation_y, 2) +
      std::pow(record.fused_orientation_z, 2), 0.5);

    msg_data.orientation.w = (double)record.fused_orientation_w / fused_orientation_norm;
    msg_data.orientation.x = (double)record.fused_orientation_x / fused_orientation_norm;
    msg_data.orientation.y = (double)record.fused_orientation_y / fused_orientation_norm;
    msg_data.orientation.z = (double)record.fused_orientation_z / fused_orientation_norm;
    msg_data.linear_acceleration.x = (double)record.fused_linear_acceleration_x / 100.0;
    msg_data.linear_acceleration.y = (double)record.fused_linear_acceleration_y / 100.0;
    msg_data.linear_acceleration.z = (double)record.fused_linear_acceleration_z / 100.0;
    msg_data.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_data.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_data.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    // Source: https://github.com/Vijfendertig/rosserial_adafruit_bno055/blob/532b63db9b0e5e5e9217bd89905001fe979df3a4/src/imu_publisher/imu_publisher.cpp#L42.
    // The Bosch BNO055 datasheet is pretty useless regarding the sensor's accuracy.
    // - The accuracy of the magnetometer is +-2.5deg. Users on online forums agree on that number.
    // - The accuracy of the gyroscope is unknown. I use the +-3deg/s zero rate offset. To be tested.
    // - The accuracy of the accelerometer is unknown. Based on the typical and maximum zero-g offset (+-80mg and
    //   +-150mg) and the fact that my graphs look better than that, I use 80mg. To be tested.
    // Cross-axis errors are not (yet) taken into account. To be tested.
    for(unsigned row = 0; row < 3; ++ row) {
      for(unsigned col = 0; col < 3; ++ col) {
        msg_data.orientation_covariance[row * 3 + col] = (row == col? 0.002: 0.);  // +-2.5deg
        msg_data.angular_velocity_covariance[row * 3 + col] = (row == col? 0.003: 0.);  // +-3deg/s
        msg_data.linear_acceleration_covariance[row * 3 + col] = (row == col? 0.60: 0.);  // +-80mg
      }
    }

    sensor_msgs::Temperature msg_temp;
    msg_temp.header.stamp = time;
    msg_temp.header.frame_id = param_frame_id;
    msg_temp.header.seq = seq;
    msg_temp.temperature = (double)record.temperature;

    pub_data.publish(msg_data);
    pub_raw.publish(msg_raw);
    pub_mag.publish(msg_mag);
    pub_temp.publish(msg_temp);

    if(seq % 50 == 0) {
        current_status.values[DIAG_CALIB_STAT].value = std::to_string(record.calibration_status);
        current_status.values[DIAG_SELFTEST_RESULT].value = std::to_string(record.self_test_result);
        current_status.values[DIAG_INTR_STAT].value = std::to_string(record.interrupt_status);
        current_status.values[DIAG_SYS_CLK_STAT].value = std::to_string(record.system_clock_status);
        current_status.values[DIAG_SYS_STAT].value = std::to_string(record.system_status);
        current_status.values[DIAG_SYS_ERR].value = std::to_string(record.system_error_code);
        pub_status.publish(current_status);
    }

    return true;    
}

bool BNO055I2CActivity::stop() {
    ROS_INFO("stopping");

    if(pub_data) pub_data.shutdown();
    if(pub_raw) pub_raw.shutdown();
    if(pub_mag) pub_mag.shutdown();
    if(pub_temp) pub_temp.shutdown();
    if(pub_status) pub_status.shutdown();

    if(service_calibrate) service_calibrate.shutdown();
    if(service_reset) service_reset.shutdown();

    return true;
}

bool BNO055I2CActivity::onServiceReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }
    return true;
}

bool BNO055I2CActivity::onServiceCalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // TODO implement this
    return true;
}

}
