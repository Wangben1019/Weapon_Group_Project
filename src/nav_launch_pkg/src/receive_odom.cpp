#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ros/subscriber.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/tf.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int32.h"

int cnt;

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z));

    q.setW(pose_msg->pose.pose.orientation.w);
    q.setX(pose_msg->pose.pose.orientation.x);
    q.setY(pose_msg->pose.pose.orientation.y);
    q.setZ(pose_msg->pose.pose.orientation.z);

    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, pose_msg->header.stamp, "/base_link", "/odom"));
    // ROS_INFO("succeed transform!!");
}

void debug_callback(const std_msgs::Int32ConstPtr &msg)
{
    ROS_INFO("node can receive :: %d", msg->data);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rececive_odom_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_vio = nh.subscribe("/vins_estimator/odometry", 100, vio_callback);

    ros::Subscriber sub_debug = nh.subscribe("debug_node", 5, debug_callback);

    ros::Rate rate(30);

    while (ros::ok()) 
    {
        ros::spinOnce();
        cnt++;
        if (cnt == 30) {
            // ROS_INFO("node is running");
            cnt = 0;
        }
        rate.sleep();
    }

    return 0;
}
