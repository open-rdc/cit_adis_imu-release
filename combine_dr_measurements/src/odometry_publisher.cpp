#include <complex>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <realtime_tools/realtime_buffer.h>

class OdometryPublisher
{

public:
    OdometryPublisher(ros::NodeHandle &nh) : 
        nh_(nh)
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("combined_odom", 5, this);
        odom_sub_ = nh_.subscribe("odom", 1, &OdometryPublisher::odom_cb, this);
        imu_sub_ = nh_.subscribe("imu", 1, &OdometryPublisher::imu_cb, this);
    }

    void odom_cb(const nav_msgs::OdometryConstPtr &msg){
        received_odom_msg_ = *msg;
        received_odom_.writeFromNonRT(received_odom_msg_);
    }

    void imu_cb(const sensor_msgs::ImuConstPtr &msg){
        received_imu_msg_ = *msg;
        received_imu_.writeFromNonRT(received_imu_msg_);
    }

    void run(){
        ros::Rate r(50.0);
        tf::TransformBroadcaster odom_broadcaster;
        nav_msgs::Odometry old_odom;

        while(nh_.ok()){
            nav_msgs::Odometry *odom = received_odom_.readFromRT();
            sensor_msgs::Imu *imu = received_imu_.readFromRT();

            if(odom && imu){
                ros::Time time = ros::Time::now();
                geometry_msgs::TransformStamped odom_trans;

                odom->header.stamp    = odom_trans.header.stamp = time;
                odom->header.frame_id = odom_trans.header.frame_id = "odom";
                odom->child_frame_id  = odom_trans.child_frame_id  = "base_link";
                
                odom->pose.pose.orientation = odom_trans.transform.rotation = imu->orientation;
               
                const double dt = (odom->header.stamp - old_odom.header.stamp).toSec();
                if(dt < 0.0001){
                    ROS_WARN_STREAM("dt = " << dt);
                    old_odom.pose.pose.orientation = odom->pose.pose.orientation;
                    old_odom.pose.pose.position = odom->pose.pose.position;
                }else{
                    tf::Quaternion old_q(
                        old_odom.pose.pose.orientation.x,
                        old_odom.pose.pose.orientation.y,
                        old_odom.pose.pose.orientation.z,
                        old_odom.pose.pose.orientation.w
                    );

                    tf::Quaternion q(
                        odom->pose.pose.orientation.x,
                        odom->pose.pose.orientation.y,
                        odom->pose.pose.orientation.z,
                        odom->pose.pose.orientation.w
                    );

                    double roll, pitch, yaw, old_roll, old_pitch, old_yaw;
                    tf::Matrix3x3(old_q).getRPY(old_roll, old_pitch, old_yaw);
                    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    const double linear = odom->twist.twist.linear.x * dt;
                    const double angular = yaw - old_yaw;
                    
                    if(std::abs(angular) < 10e-3){
                        double direction = old_yaw + angular * 0.5;
                        ROS_INFO_STREAM("direction = " << direction);
                        ROS_INFO_STREAM("cos = " << cos(direction));
                        odom->pose.pose.position.x = old_odom.pose.pose.position.x + linear * cos(direction);
                        odom->pose.pose.position.y = old_odom.pose.pose.position.y + linear * sin(direction);
                        ROS_INFO_STREAM("odom->pose.pose.position.x = " << odom->pose.pose.position.x);
                        ROS_INFO_STREAM("odom->pose.pose.position.y = " << odom->pose.pose.position.y);
                    }else{
                        const double r = linear / angular;
                        odom->pose.pose.position.x = old_odom.pose.pose.position.x + r * (sin(yaw) - sin(old_yaw));
                        odom->pose.pose.position.y = old_odom.pose.pose.position.y - r * (cos(yaw) - cos(old_yaw));
                    }

                    odom_trans.transform.translation.x = odom->pose.pose.position.x;
                    odom_trans.transform.translation.y = odom->pose.pose.position.y;
                    odom_trans.transform.translation.z = odom->pose.pose.position.z;

                    odom_broadcaster.sendTransform(odom_trans);
                    odom_pub_.publish(*odom);
                   
                    old_odom = *odom;
                }
            }
            
            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::Publisher odom_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;

    nav_msgs::Odometry received_odom_msg_;
    sensor_msgs::Imu received_imu_msg_;
    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> received_odom_;
    realtime_tools::RealtimeBuffer<sensor_msgs::Imu> received_imu_;
    ros::NodeHandle &nh_;

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "combine_dr_measurements");

    ros::NodeHandle n;
    OdometryPublisher odom_publisher(n);
    odom_publisher.run();

}

