#ifndef ODOMETRY_PUBLISHER_H_
#define ODOMETRY_PUBLISHER_H_

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

namespace combine_dr_measurements{

    class OdometryPublisher{
    public:
        OdometryPublisher(ros::NodeHandle &nh);
        void odom_cb(const nav_msgs::OdometryConstPtr &msg);
        void imu_cb(const sensor_msgs::ImuConstPtr &msg);
        void run();

    private:
        ros::Publisher odom_pub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber imu_sub_;

        nav_msgs::Odometry received_odom_msg_;
        sensor_msgs::Imu received_imu_msg_;
        realtime_tools::RealtimeBuffer<nav_msgs::Odometry> received_odom_;
        realtime_tools::RealtimeBuffer<sensor_msgs::Imu> received_imu_;
        ros::NodeHandle &nh_;
        double max_update_rate_;
    };

};

#endif //ODOMETRY_PUBLISHER_H_
