#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pc_pub_, odom_pub_;
std::string new_odom_, new_cloud_;
std::string old_odom_, old_cloud_;
std::string odom_frame_, cloud_frame_;

void pcCallback(const sensor_msgs::PointCloud2Ptr msg)
{
    sensor_msgs::PointCloud2 newCloud = *msg;
    newCloud.header.frame_id = cloud_frame_;
    pc_pub_.publish(newCloud);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg)
{
    nav_msgs::Odometry newOdom = *odometryMsg;
    newOdom.header.frame_id = odom_frame_;
    newOdom.child_frame_id = "";
    odom_pub_.publish(newOdom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "new_topic_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle np("~");
    np.param<std::string>("new_odom", new_odom_, "map");
    np.param<std::string>("new_cloud", new_cloud_, "map");
    np.param<std::string>("old_odom", old_odom_, "map");
    np.param<std::string>("old_cloud", old_cloud_, "map");
    np.param<std::string>("odom_frame", odom_frame_, "world");
    np.param<std::string>("cloud_frame", cloud_frame_, "world");

    ros::Subscriber point_Cloud_Sub = nh.subscribe(old_cloud_, 1, pcCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber odom_Sub = nh.subscribe(old_odom_, 1, odomCallback, ros::TransportHints().tcpNoDelay());

    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>(new_cloud_, 1);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(new_odom_, 1);

    ros::spin();
    return 0;
}