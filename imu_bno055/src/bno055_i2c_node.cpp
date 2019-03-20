/* bno055_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a BNO055I2C Activity class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include <imu_bno055/bno055_i2c_activity.h>
#include "watchdog/watchdog.h"
#include <csignal>

int main(int argc, char *argv[]) {
    ros::NodeHandle* nh = NULL;
    ros::NodeHandle* nh_priv = NULL;

    imu_bno055::BNO055I2CActivity* activity = NULL;
    watchdog::Watchdog* watchdog = NULL;

    ros::init(argc, argv, "bno055_node");

    nh = new ros::NodeHandle();
    if(!nh) {
        ROS_FATAL("Failed to initialize NodeHandle");
        ros::shutdown();
        return -1;
    }

    nh_priv = new ros::NodeHandle("~");
    if(!nh_priv) {
        ROS_FATAL("Failed to initialize private NodeHandle");
        delete nh;
        ros::shutdown();
        return -2;
    }

    activity = new imu_bno055::BNO055I2CActivity(*nh, *nh_priv);
    watchdog = new watchdog::Watchdog();

    if(!activity) {
        ROS_FATAL("Failed to initialize driver");
        delete nh_priv;
        delete nh;
        ros::shutdown();
        return -3;
    }

    if(!activity->start()) {
        ROS_ERROR("Failed to start activity");
        delete nh_priv;
        delete nh;
        ros::shutdown();
        return -4;
    }

    watchdog->start(5000);

    int param_rate;
    nh_priv->param("rate", param_rate, (int)100);

    ros::Rate rate(param_rate);
    while(ros::ok()) {
        rate.sleep();
        if(activity->spinOnce()) {
            watchdog->refresh();
        }
    }

    activity->stop();
    watchdog->stop();

    delete watchdog;
    delete activity;
    delete nh_priv;
    delete nh;

    return 0;
}
