#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <detection_msgs/BoundingBoxes.h>


using namespace std;
using namespace sensor_msgs;

ros::Publisher pub_gray;
ros::Publisher pub_depth;
ros::Publisher pub_boxes;

ros::Publisher pub_pose;
ros::Publisher pub_odom;

Image gray_msg;
Image depth_msg;

detection_msgs::BoundingBoxes boxes_msg;

geometry_msgs::PoseStamped current_pose;
nav_msgs::Odometry current_odom;

ros::Time time_image;
ros::Time time_pose;

void cb_boxes(const detection_msgs::BoundingBoxesConstPtr& boxes)
{
    boxes_msg = *boxes;
    boxes_msg.header.stamp = ros::Time::now();

    pub_boxes.publish(boxes_msg);
}

void cb_gray(const sensor_msgs::ImageConstPtr& gray)
{
    gray_msg = *gray;
    gray_msg.header.stamp = ros::Time::now();

    pub_gray.publish(gray_msg);
}

void cb_depth(const sensor_msgs::ImageConstPtr& depth)
{
    depth_msg = *depth;
    depth_msg.header.stamp = ros::Time::now();

    pub_depth.publish(depth_msg);
}


void cb_pose(const geometry_msgs::PoseStampedConstPtr& pose)
{
    current_pose = *pose;
    current_pose.header.stamp = ros::Time::now();

    pub_pose.publish(current_pose);
}

void cb_odom(const nav_msgs::Odometry::ConstPtr& odom)
{
    current_odom = *odom;
    current_odom.header.stamp = ros::Time::now();

    pub_odom.publish(current_odom);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_sync_node");
    ros::NodeHandle nh;

    pub_gray = nh.advertise<Image>("/sync/gray/image_raw", 1);
    pub_depth = nh.advertise<Image>("/sync/depth/image_raw", 1);
    pub_boxes = nh.advertise<detection_msgs::BoundingBoxes>("/sync/boxes", 1);

    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/sync/local_position/pose", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/sync/local_position/odom", 1);

    ros::Subscriber sub_gray = nh.subscribe<Image>("/gray", 10, &cb_gray);
    ros::Subscriber sub_depth = nh.subscribe<Image>("/depth", 10, &cb_depth);
    ros::Subscriber sub_boxes = nh.subscribe<detection_msgs::BoundingBoxes>("/boxes", 10, &cb_boxes);

    ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 1, &cb_pose);
    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &cb_odom);

    ros::spin();

    return 0;
}
