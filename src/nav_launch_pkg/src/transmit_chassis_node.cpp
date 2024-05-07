#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "nav_launch_pkg/transmit_chassis_node.h"
#include "geometry_msgs/Twist.h"
#include "ros/subscriber.h"

void Cmd_Vel_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    Expect_Speed_Typedef Expect_Speed;
    VCOMCOMM vcom_user;
    QByteArray serial_data_user;

    Expect_Speed.Forward_Back_Remote = msg->linear.x * 5;
    Expect_Speed.Rotate_Remote = -msg->angular.z * 0.925925926;

    serial_data_user.append((char *)&Expect_Speed, sizeof(Expect_Speed_Typedef));

    ROS_INFO("FB is :%f, LR is :%f, Rot is :%f",Expect_Speed.Forward_Back_Remote, Expect_Speed.Left_Right_Remote, Expect_Speed.Rotate_Remote);

    vcom_user.Transmit(1, 1, serial_data_user);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "transmit_chassis_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_cmd = nh.subscribe("/cmd_vel", 5, Cmd_Vel_Callback);

    ros::Rate rate(30);

    while (ros::ok()) 
    {
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
