#include "imu_bno055/bno055_i2c_driver.h"
#include "imu_bno055/bno055_uart_driver.h"
#include "watchdog/watchdog.h"
#include <csignal>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <memory>

class BNO055Node {
public:
    BNO055Node();
    void run();
    bool readAndPublish();
    void stop();
    bool onSrvReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
private:
    enum class InterfaceType { I2C, UART };
    
    std::unique_ptr<imu_bno055::BNO055I2CDriver> imu_i2c;
    std::unique_ptr<imu_bno055::BNO055UARTDriver> imu_uart;
    InterfaceType interface_type;

    std::string param_device;
    int param_address;
    int param_baud_rate;
    double param_rate;
    std::string param_frame_id;

    diagnostic_msgs::DiagnosticStatus current_status;

    ros::NodeHandle nh;
    ros::Publisher pub_data;
    ros::Publisher pub_raw;
    ros::Publisher pub_mag;
    ros::Publisher pub_temp;
    ros::Publisher pub_status;
    ros::ServiceServer srv_reset;

    std::unique_ptr<ros::Rate> rate;

    watchdog::Watchdog watchdog;

    int seq;
};

BNO055Node::BNO055Node() : nh("~") {
    nh.param<std::string>("device", param_device, "/dev/i2c-1");
    std::string param_interface;
    nh.param<std::string>("interface", param_interface, "i2c");
    nh.param<int>("address", param_address, BNO055_ADDRESS_A);
    nh.param<int>("baud_rate", param_baud_rate, 115200);
    nh.param<std::string>("frame_id", param_frame_id, "imu");
    nh.param<double>("rate", param_rate, 100.0);

    if (param_interface == "i2c") {
        interface_type = InterfaceType::I2C;
        imu_i2c = std::make_unique<imu_bno055::BNO055I2CDriver>(param_device, param_address);
        imu_i2c->init();
    } else if (param_interface == "uart") {
        interface_type = InterfaceType::UART;
        imu_uart = std::make_unique<imu_bno055::BNO055UARTDriver>(param_device, param_baud_rate);
        imu_uart->init();
    } else {
        throw std::runtime_error("Unsupported interface type");
    }

    pub_data = nh.advertise<sensor_msgs::Imu>("data", 10);
    pub_raw = nh.advertise<sensor_msgs::Imu>("raw", 10);
    pub_mag = nh.advertise<sensor_msgs::MagneticField>("mag", 10);
    pub_temp = nh.advertise<sensor_msgs::Temperature>("temp", 10);
    pub_status = nh.advertise<diagnostic_msgs::DiagnosticStatus>("status", 10);

    srv_reset = nh.advertiseService("reset", &BNO055Node::onSrvReset, this);

    seq = 0;

    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055";

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

    rate = std::make_unique<ros::Rate>(param_rate);
}

void BNO055Node::run() {
    while (ros::ok()) {
        rate->sleep();
        if (readAndPublish()) {
            watchdog.refresh();
        }
        ros::spinOnce();
    }
}

bool BNO055Node::readAndPublish() {
    imu_bno055::IMURecord record;

    try {
        if (interface_type == InterfaceType::I2C) {
            record = imu_i2c->read();
        } else if (interface_type == InterfaceType::UART) {
            record = imu_uart->read();
        }
    } catch(const std::runtime_error& e) {
        ROS_WARN("%s", e.what());
        return false;
    }

    ros::Time time = ros::Time::now();

    sensor_msgs::Imu msg_raw;
    msg_raw.header.stamp = time;
    msg_raw.header.frame_id = param_frame_id;
    msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
    msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
    msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
    msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::MagneticField msg_mag;
    msg_mag.header.stamp = time;
    msg_mag.header.frame_id = param_frame_id;
    msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
    msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
    msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;

    sensor_msgs::Imu msg_data;
    msg_data.header.stamp = time;
    msg_data.header.frame_id = param_frame_id;

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

    sensor_msgs::Temperature msg_temp;
    msg_temp.header.stamp = time;
    msg_temp.header.frame_id = param_frame_id;
    msg_temp.temperature = (double)record.temperature;

    pub_data.publish(msg_data);
    pub_raw.publish(msg_raw);
    pub_mag.publish(msg_mag);
    pub_temp.publish(msg_temp);

    if((seq++) % 50 == 0) {
        current_status.values[0].value = std::to_string(record.calibration_status);
        current_status.values[1].value = std::to_string(record.self_test_result);
        current_status.values[2].value = std::to_string(record.interrupt_status);
        current_status.values[3].value = std::to_string(record.system_clock_status);
        current_status.values[4].value = std::to_string(record.system_status);
        current_status.values[5].value = std::to_string(record.system_error_code);
        pub_status.publish(current_status);
    }

    return true;
}

void BNO055Node::stop() {
    ROS_INFO("Stopping");
    pub_data.shutdown();
    pub_raw.shutdown();
    pub_mag.shutdown();
    pub_temp.shutdown();
    pub_status.shutdown();
    srv_reset.shutdown();
}

bool BNO055Node::onSrvReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    bool reset_successful = false;

    try {
        if (interface_type == InterfaceType::I2C) {
            reset_successful = imu_i2c->reset();
        } else if (interface_type == InterfaceType::UART) {
            reset_successful = imu_uart->reset();
        }
    } catch(const std::runtime_error& e) {
        ROS_WARN("%s", e.what());
    }

    if (!reset_successful) {
        res.success = false;
        res.message = "IMU reset failed";
        return false;
    }

    res.success = true;
    res.message = "IMU reset successful";
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bno055_node");
    BNO055Node node;
    node.run();
    node.stop();
    return 0;
}

