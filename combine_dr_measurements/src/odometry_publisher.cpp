#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
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
        ros::Rate r(100.0);
        tf::TransformBroadcaster odom_broadcaster;

        while(nh_.ok()){
            nav_msgs::Odometry *odom = received_odom_.readFromRT();
            sensor_msgs::Imu *imu = received_imu_.readFromRT();

            if(odom && imu){
                ros::Time time = ros::Time::now();
                ROS_INFO_STREAM("received_odom.stamp = " << odom->header.stamp);
                ROS_INFO_STREAM("received_imu.stamp = " << imu->header.stamp);
                geometry_msgs::TransformStamped odom_trans;

                odom->header.stamp    = odom_trans.header.stamp = time;
                odom->header.frame_id = odom_trans.header.frame_id = "odom";
                odom->child_frame_id  = odom_trans.child_frame_id  = "base_link";
                
                odom->pose.pose.orientation = odom_trans.transform.rotation = imu->orientation;
               
                odom_trans.transform.translation.x = odom->pose.pose.position.x;
                odom_trans.transform.translation.y = odom->pose.pose.position.y;
                odom_trans.transform.translation.z = odom->pose.pose.position.z;

                odom_broadcaster.sendTransform(odom_trans);
                odom_pub_.publish(*odom);
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

