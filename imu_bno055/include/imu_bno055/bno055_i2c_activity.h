#ifndef _bno055_i2c_activity_dot_h
#define _bno055_i2c_activity_dot_h

#include <ros/ros.h>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <linux/i2c-dev.h>
#include <smbus_functions.h>

#define BNO055_ID 0xA0

#define BNO055_ADDRESS_A 0x28 // default
#define BNO055_ADDRESS_B 0x29

#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_ACCEL_REV_ID_ADDR 0x01
#define BNO055_MAG_REV_ID_ADDR 0x02
#define BNO055_GYRO_REV_ID_ADDR 0x03
#define BNO055_SW_REV_ID_LSB_ADDR 0x04
#define BNO055_SW_REV_ID_MSB_ADDR 0x05
#define BNO055_BL_REV_ID_ADDR 0X06
#define BNO055_PAGE_ID_ADDR 0X07

#define BNO055_ACC_CONFIG 0X08 // Page 1
#define BNO055_GYR_CONFIG_0 0X0A // Page 1

#define BNO055_ACCEL_DATA_X_LSB_ADDR 0X08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0X09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0X0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0X0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0X0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0X0D

#define BNO055_MAG_DATA_X_LSB_ADDR 0X0E
#define BNO055_MAG_DATA_X_MSB_ADDR 0X0F
#define BNO055_MAG_DATA_Y_LSB_ADDR 0X10
#define BNO055_MAG_DATA_Y_MSB_ADDR 0X11
#define BNO055_MAG_DATA_Z_LSB_ADDR 0X12
#define BNO055_MAG_DATA_Z_MSB_ADDR 0X13

#define BNO055_GYRO_DATA_X_LSB_ADDR 0X14
#define BNO055_GYRO_DATA_X_MSB_ADDR 0X15
#define BNO055_GYRO_DATA_Y_LSB_ADDR 0X16
#define BNO055_GYRO_DATA_Y_MSB_ADDR 0X17
#define BNO055_GYRO_DATA_Z_LSB_ADDR 0X18
#define BNO055_GYRO_DATA_Z_MSB_ADDR 0X19

#define BNO055_EULER_H_LSB_ADDR 0X1A
#define BNO055_EULER_H_MSB_ADDR 0X1B
#define BNO055_EULER_R_LSB_ADDR 0X1C
#define BNO055_EULER_R_MSB_ADDR 0X1D
#define BNO055_EULER_P_LSB_ADDR 0X1E
#define BNO055_EULER_P_MSB_ADDR 0X1F

#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0X20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0X21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0X22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0X23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0X24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0X25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0X26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0X27

#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0X28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0X29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0X2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0X2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0X2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0X2D

#define BNO055_GRAVITY_DATA_X_LSB_ADDR 0X2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR 0X2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR 0X30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR 0X31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR 0X32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR 0X33

#define BNO055_TEMP_ADDR 0X34

#define BNO055_CALIB_STAT_ADDR 0X35
#define BNO055_SELFTEST_RESULT_ADDR 0X36
#define BNO055_INTR_STAT_ADDR 0X37

#define BNO055_SYS_CLK_STAT_ADDR 0X38
#define BNO055_SYS_STAT_ADDR 0X39
#define BNO055_SYS_ERR_ADDR 0X3A

#define BNO055_UNIT_SEL_ADDR 0X3B
#define BNO055_DATA_SELECT_ADDR 0X3C

#define BNO055_OPR_MODE_ADDR 0X3D
#define BNO055_PWR_MODE_ADDR 0X3E

#define BNO055_SYS_TRIGGER_ADDR 0X3F
#define BNO055_TEMP_SOURCE_ADDR 0X40

#define BNO055_AXIS_MAP_CONFIG_ADDR 0X41
#define BNO055_AXIS_MAP_SIGN_ADDR 0X42

#define BNO055_SIC_MATRIX_0_LSB_ADDR 0X43
#define BNO055_SIC_MATRIX_0_MSB_ADDR 0X44
#define BNO055_SIC_MATRIX_1_LSB_ADDR 0X45
#define BNO055_SIC_MATRIX_1_MSB_ADDR 0X46
#define BNO055_SIC_MATRIX_2_LSB_ADDR 0X47
#define BNO055_SIC_MATRIX_2_MSB_ADDR 0X48
#define BNO055_SIC_MATRIX_3_LSB_ADDR 0X49
#define BNO055_SIC_MATRIX_3_MSB_ADDR 0X4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR 0X4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR 0X4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR 0X4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR 0X4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR 0X4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR 0X50
#define BNO055_SIC_MATRIX_7_LSB_ADDR 0X51
#define BNO055_SIC_MATRIX_7_MSB_ADDR 0X52
#define BNO055_SIC_MATRIX_8_LSB_ADDR 0X53
#define BNO055_SIC_MATRIX_8_MSB_ADDR 0X54

#define BNO055_ACCEL_OFFSET_X_LSB_ADDR 0X55
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR 0X56
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR 0X57
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR 0X58
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR 0X59
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR 0X5A

#define BNO055_MAG_OFFSET_X_LSB_ADDR 0X5B
#define BNO055_MAG_OFFSET_X_MSB_ADDR 0X5C
#define BNO055_MAG_OFFSET_Y_LSB_ADDR 0X5D
#define BNO055_MAG_OFFSET_Y_MSB_ADDR 0X5E
#define BNO055_MAG_OFFSET_Z_LSB_ADDR 0X5F
#define BNO055_MAG_OFFSET_Z_MSB_ADDR 0X60

#define BNO055_GYRO_OFFSET_X_LSB_ADDR 0X61
#define BNO055_GYRO_OFFSET_X_MSB_ADDR 0X62
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR 0X63
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR 0X64
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR 0X65
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR 0X66

#define BNO055_ACCEL_RADIUS_LSB_ADDR 0X67
#define BNO055_ACCEL_RADIUS_MSB_ADDR 0X68
#define BNO055_MAG_RADIUS_LSB_ADDR 0X69
#define BNO055_MAG_RADIUS_MSB_ADDR 0X6A

#define BNO055_POWER_MODE_NORMAL 0X00
#define BNO055_POWER_MODE_LOWPOWER 0X01
#define BNO055_POWER_MODE_SUSPEND 0X02

#define BNO055_OPERATION_MODE_CONFIG 0X00
#define BNO055_OPERATION_MODE_ACCONLY 0X01
#define BNO055_OPERATION_MODE_MAGONLY 0X02
#define BNO055_OPERATION_MODE_GYRONLY 0X03
#define BNO055_OPERATION_MODE_ACCMAG 0X04
#define BNO055_OPERATION_MODE_ACCGYRO 0X05
#define BNO055_OPERATION_MODE_MAGGYRO 0X06
#define BNO055_OPERATION_MODE_AMG 0X07
#define BNO055_OPERATION_MODE_IMUPLUS 0X08
#define BNO055_OPERATION_MODE_COMPASS 0X09
#define BNO055_OPERATION_MODE_M4G 0X0A
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF 0X0B
#define BNO055_OPERATION_MODE_NDOF 0X0C

#define BNO055_REMAP_CONFIG_P0 0x21
#define BNO055_REMAP_CONFIG_P1 0x24 // default
#define BNO055_REMAP_CONFIG_P2 0x24
#define BNO055_REMAP_CONFIG_P3 0x21
#define BNO055_REMAP_CONFIG_P4 0x24
#define BNO055_REMAP_CONFIG_P5 0x21
#define BNO055_REMAP_CONFIG_P6 0x21
#define BNO055_REMAP_CONFIG_P7 0x24

#define BNO055_REMAP_SIGN_P0 0x04
#define BNO055_REMAP_SIGN_P1 0x00 // default
#define BNO055_REMAP_SIGN_P2 0x06
#define BNO055_REMAP_SIGN_P3 0x02
#define BNO055_REMAP_SIGN_P4 0x03
#define BNO055_REMAP_SIGN_P5 0x01
#define BNO055_REMAP_SIGN_P6 0x07
#define BNO055_REMAP_SIGN_P7 0x05

#define DIAG_CALIB_STAT 0
#define DIAG_SELFTEST_RESULT 1
#define DIAG_INTR_STAT 2
#define DIAG_SYS_CLK_STAT 3
#define DIAG_SYS_STAT 4
#define DIAG_SYS_ERR 5

namespace imu_bno055 {

// order of this struct is designed to match the I2C registers
// so all data can be read in one fell swoop
typedef struct {
  int16_t raw_linear_acceleration_x;
  int16_t raw_linear_acceleration_y;
  int16_t raw_linear_acceleration_z;
  int16_t raw_magnetic_field_x;
  int16_t raw_magnetic_field_y;
  int16_t raw_magnetic_field_z;
  int16_t raw_angular_velocity_x;
  int16_t raw_angular_velocity_y;
  int16_t raw_angular_velocity_z;
  int16_t fused_heading;
  int16_t fused_roll;
  int16_t fused_pitch;
  int16_t fused_orientation_w;
  int16_t fused_orientation_x;
  int16_t fused_orientation_y;
  int16_t fused_orientation_z;
  int16_t fused_linear_acceleration_x;
  int16_t fused_linear_acceleration_y;
  int16_t fused_linear_acceleration_z;
  int16_t gravity_vector_x;
  int16_t gravity_vector_y;
  int16_t gravity_vector_z;
  int8_t temperature;
  uint8_t calibration_status;
  uint8_t self_test_result;
  uint8_t interrupt_status;
  uint8_t system_clock_status;
  uint8_t system_status;
  uint8_t system_error_code;
} IMURecord;

class BNO055I2CActivity {
  public:
    BNO055I2CActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);

    bool start();
    bool stop();
    bool spinOnce();

    bool onServiceReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onServiceCalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  private:
    int operation_mode();
    bool configure_sensors();
    bool reset();

    // class variables
    uint32_t seq = 0;
    int file;
    diagnostic_msgs::DiagnosticStatus current_status;

    // ROS parameters
    std::string param_frame_id;
    std::string param_device;
    std::string param_operation_mode;
    int param_address;
    double param_acc_bandwidth;
    double param_gyro_bandwidth;
    int param_acc_range;
    int param_gyro_range;

    // ROS node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    // ROS publishers
    ros::Publisher pub_data;
    ros::Publisher pub_raw;
    ros::Publisher pub_mag;
    ros::Publisher pub_temp;
    ros::Publisher pub_status;

    // ROS subscribers
    ros::ServiceServer service_calibrate;
    ros::ServiceServer service_reset;
};

}

#endif // _bno055_i2c_activity_dot_h
