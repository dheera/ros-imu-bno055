#include <imu_bno055/bno055_i2c_driver.h>
#include "watchdog/watchdog.h"
#include <csignal>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <memory>

class BNO055I2CNode : public rclcpp::Node {
public:
    BNO055I2CNode();
    void run();
    bool readAndPublish();
    void stop();
    bool onSrvReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
private:
    std::unique_ptr<imu_bno055::BNO055I2CDriver> imu;

    std::string param_device;
    int param_address;
    double param_rate;
    std::string param_frame_id;

    diagnostic_msgs::msg::DiagnosticStatus current_status;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_data;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_raw;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_status;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset;

    std::unique_ptr<rclcpp::Rate> rate;

    watchdog::Watchdog watchdog;

    int seq;
};

BNO055I2CNode::BNO055I2CNode()
    : Node("bno055_node") {  // Initialize rate here
    this->declare_parameter<std::string>("device", "/dev/i2c-1");
    this->declare_parameter<int>("address", BNO055_ADDRESS_A);
    this->declare_parameter<std::string>("frame_id", "imu");
    this->declare_parameter<double>("rate", 100.0);

    this->get_parameter("device", param_device);
    this->get_parameter("address", param_address);
    this->get_parameter("frame_id", param_frame_id);
    this->get_parameter("rate", param_rate);

    imu = std::make_unique<imu_bno055::BNO055I2CDriver>(param_device, param_address);

    imu->init();

    pub_data = this->create_publisher<sensor_msgs::msg::Imu>("data", 10);
    pub_raw = this->create_publisher<sensor_msgs::msg::Imu>("raw", 10);
    pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
    pub_temp = this->create_publisher<sensor_msgs::msg::Temperature>("temp", 10);
    pub_status = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("status", 10);

    srv_reset = this->create_service<std_srvs::srv::Trigger>("reset", std::bind(&BNO055I2CNode::onSrvReset, this, std::placeholders::_1, std::placeholders::_2));

    seq = 0;

    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::msg::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::msg::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::msg::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::msg::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::msg::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::msg::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);

    rate = std::make_unique<rclcpp::Rate>(param_rate);
}

void BNO055I2CNode::run() {
    while (rclcpp::ok()) {
        rate->sleep();
        if (readAndPublish()) {
            watchdog.refresh();
        }
    }
}

bool BNO055I2CNode::readAndPublish() {
    imu_bno055::IMURecord record;

    try {
        record = imu->read();
    } catch(const std::runtime_error& e) {
        RCLCPP_WARN(this->get_logger(), e.what());
    }

    rclcpp::Time time = this->now();

    sensor_msgs::msg::Imu msg_raw;
    msg_raw.header.stamp = time;
    msg_raw.header.frame_id = param_frame_id;
    msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
    msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
    msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
    msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::msg::MagneticField msg_mag;
    msg_mag.header.stamp = time;
    msg_mag.header.frame_id = param_frame_id;
    msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
    msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
    msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;

    sensor_msgs::msg::Imu msg_data;
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

    sensor_msgs::msg::Temperature msg_temp;
    msg_temp.header.stamp = time;
    msg_temp.header.frame_id = param_frame_id;
    msg_temp.temperature = (double)record.temperature;

    pub_data->publish(msg_data);
    pub_raw->publish(msg_raw);
    pub_mag->publish(msg_mag);
    pub_temp->publish(msg_temp);

    if((seq++) % 50 == 0) {
        current_status.values[DIAG_CALIB_STAT].value = std::to_string(record.calibration_status);
        current_status.values[DIAG_SELFTEST_RESULT].value = std::to_string(record.self_test_result);
        current_status.values[DIAG_INTR_STAT].value = std::to_string(record.interrupt_status);
        current_status.values[DIAG_SYS_CLK_STAT].value = std::to_string(record.system_clock_status);
        current_status.values[DIAG_SYS_STAT].value = std::to_string(record.system_status);
        current_status.values[DIAG_SYS_ERR].value = std::to_string(record.system_error_code);
        pub_status->publish(current_status);
    }

    return true;
}

void BNO055I2CNode::stop() {
    RCLCPP_INFO(this->get_logger(), "Stopping");
    pub_data.reset();
    pub_raw.reset();
    pub_mag.reset();
    pub_temp.reset();
    pub_status.reset();
    srv_reset.reset();
}

bool BNO055I2CNode::onSrvReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if(!(imu->reset())) {
        throw std::runtime_error("chip reset failed");
        return false;
    }
    res->success = true;
    res->message = "IMU reset successful";
    return true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BNO055I2CNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}

