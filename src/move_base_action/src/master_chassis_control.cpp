#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "ros/service.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include <string>
#include "move_base_action/master_chassis_control.h"

/**
  * TODO：
  *     1. 订阅激光雷达距离地面距离
  *     2. 发布无人机起飞话题
  *     3. 编写无人机落地逻辑（当前构思：起飞时距目标点小于()米，直接飞到目标点，大于()米，飞过障碍物后落地）
  *     4. 关于落地逻辑细分：当已经飞过()米时，判断仍未飞过障碍物，则一直飞到无障碍物为止。若已经飞过障碍物，则直接落地即可。
  *     5. 关于激光雷达对地距离对于障碍物检测的逻辑判断
  *     6. 保证无人机起飞时，距离障碍物足够远，起飞时足够安全。
  */

nav_msgs::Odometry vin_odom_Info;               // 订阅vins odom
geometry_msgs::PoseStamped take_off_info;       // 该变量中存储了 所有的goal point信息
geometry_msgs::PoseStamped sub_goal_info;       // 订阅原始goal point。（信息应该与take_off_info中一样）
std_msgs::Bool If_Take_Off;                     // 是否可以起飞标志位
geometry_msgs::Quaternion reset_system;         // 重置move_base状态机，数据类型自定义（）
geometry_msgs::PoseStamped pub_goal_after_reset;// 若落地后仍需车移动。发布goal point给move_base
sensor_msgs::Range lidar_info;                  // 激光雷达消息
sensor_msgs::Range lidar_info_last;             // 激光雷达上一帧消息
ros::Publisher pub_uav_pose;

double flight_dis_x_thre;               // 飞行距离阈值，若起飞时距离目标点的距离小于该阈值，则直接飞到目标点后降落
double flight_dis_y_thre;
bool Need_Lookout = true;               // 是否需要程序监视飞机的降落，当他为true时证明无人机距目标点的距离大于上述阈值
UAV_State_e UAV_State = Land ;                  // 无人机的飞行状态：起飞/着陆
UAV_State_e UAV_State_Last;             // 无人机的上一帧飞行状态
bool uav_first_take_off = false;        // 无人机再一次运动循环内首次起飞标志位，为true的话，证明无人机已经完成从着陆到起飞的状态变化
float lidar_range_max;                  // 雷达探测距离的最大值，被认为是起飞到最高点时距离平地的距离，用作判断是否越过障碍物
UAV_Fight_Schedule_e UAV_Fight_Schedule;
UAV_Fight_Schedule_e UAV_Fight_Schedule_last;
float Obstacles_Proportion_Thre = 0.8;             // 扫描到障碍物之后，飞机距离障碍物的高度与距离地面的高度的比例阈值，如果是坑应该大于1，如果是障碍物应该小于1，此处只考虑小于1的情况

int Lidar_Data_Filter_num = 10;        // 采集无人机在地面上时雷达探测的距离时的过滤数。
float Lidar_LandRange;                  // 无人机在地面上时雷达探测的距离
// bool get_Lidar_LandRange_flag = true;   // 无人机采集在地面上时雷达探测的距离的标志位，每次着陆，或者当开机时，应为true，采集数据
// int skip_num_get_RangeMax = 10;

double Flown_Over_Obstacles_Odom_X;

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    sub_goal_info = *msg;
}

void take_off_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(UAV_State == Land)
    {
        std::string frame;
        std::string str_flag = "takeoff";
        size_t num = msg->header.frame_id.find(str_flag);
        if (num != std::string::npos) 
        {
            If_Take_Off.data = true;
            frame = msg->header.frame_id.substr(0, num);
        }

        take_off_info = *msg;
        take_off_info.header.frame_id = frame;
        if (take_off_info.pose.position.x - vin_odom_Info.pose.pose.position.x < flight_dis_x_thre &&
        take_off_info.pose.position.y - vin_odom_Info.pose.pose.position.y < flight_dis_y_thre) 
        {
            Need_Lookout = false; /* 起飞时距离目标点已经很近，不需要进行监视落地，直接飞到目标点即可 */
        }
        else 
        {
            Need_Lookout = true;
        }
    }
}

void vins_odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    vin_odom_Info = *msg;
}

void uav_state_cb(const std_msgs::UInt8::ConstPtr &msg)
{
    UAV_State_Last = UAV_State;
    if (msg->data == 1) 
    {
        UAV_State = TakeOff;
        ROS_INFO("UAV state take off");
    }
    else if(msg->data == 2)
    {
        UAV_State = Land;
        ROS_INFO("UAV state land");
    }
    else
    {
        UAV_State = Unknown;
    }

    /* 由降落到起飞的状态变化，证明已经起飞，状态变为land，则证明已经是起飞后降落 */
    if (UAV_State_Last == Land && UAV_State == TakeOff) 
    {
        uav_first_take_off = true;
        ROS_INFO("UAV state from land to Take off");
    }
}

void lidar_msg_cb(const sensor_msgs::Range::ConstPtr &msg)
{
    static int cnt_min;
    static int cnt_max;
    static int cnt_obs;
    static int cnt_end;
    static float maybe_max;
    static bool get_maybe_max_flag = false;
    static float Above_Obstacles_range;
    lidar_info_last = lidar_info;
    UAV_Fight_Schedule_last = UAV_Fight_Schedule;
    lidar_info = *msg;
    // ROS_INFO("get msg");
    if(UAV_State == Land)
    {
        cnt_min++;
        if(cnt_min > Lidar_Data_Filter_num)
        {
            Lidar_LandRange = lidar_info.range;
            // get_Lidar_LandRange_flag = false;
            cnt_min = 0;
            ROS_INFO("get lidar land : %f", Lidar_LandRange);
        }
    }

    if(UAV_State == TakeOff && UAV_Fight_Schedule != Flown_Over_Obstacles)
    {
        // if(lidar_info.range >= lidar_info_last.range && lidar_info.range >= lidar_range_max)
        // {
        //     if(!get_maybe_max_flag)
        //     {
        //         maybe_max = lidar_info.range;
        //         get_maybe_max_flag = true;    
        //     }
        // }
        
        if(lidar_info.range > lidar_range_max)
        {
            cnt_max++;
            if(cnt_max > Lidar_Data_Filter_num)
            {
                UAV_Fight_Schedule = Prepare_Over_Obstacles;
                lidar_range_max = lidar_info.range;
                ROS_INFO("\033[1;32m----> UAV is on the rise, the lidar max range is %f.\033[0m", lidar_range_max);
                cnt_max = 0;
                // get_maybe_max_flag = false;
            }
        }
        else
        {
            cnt_max = 0;
            // get_maybe_max_flag = false;
            /* 如果距离地面的高度的Obstacles_Proportion_Thre倍，仍然小于当前探测高度，则证明已经在障碍物上空 */
            if((lidar_info.range / lidar_range_max) < Obstacles_Proportion_Thre)
            {
                cnt_obs++;
                if(cnt_obs > Lidar_Data_Filter_num)
                {
                    UAV_Fight_Schedule = Above_Obstacles;
                    Above_Obstacles_range = lidar_info.range;
                    ROS_INFO("\033[1;32m----> UAV is above obstacles, because now range is less than max range,lidar_range_max is : %f .\033[0m", lidar_range_max);
                    // lidar_range_max = 0;
                    cnt_obs = 0;
                }
            }
            else
            {
                cnt_obs = 0;
            }
        }
    }

    if(UAV_Fight_Schedule_last == Above_Obstacles && (lidar_info.range /  lidar_range_max) > Obstacles_Proportion_Thre)
    {
        cnt_end++;
        if(cnt_end > Lidar_Data_Filter_num)
        {
            UAV_Fight_Schedule = Flown_Over_Obstacles;
            Flown_Over_Obstacles_Odom_X = vin_odom_Info.pose.pose.position.x;
            geometry_msgs::Point uav_goal__;
            uav_goal__.x = Flown_Over_Obstacles_Odom_X + 2;
            uav_goal__.y = vin_odom_Info.pose.pose.position.y;
            uav_goal__.z = vin_odom_Info.pose.pose.position.z;
            pub_uav_pose.publish(uav_goal__);
             ROS_INFO("\033[1;32m----> UAV has flown over obstacles .\033[0m");
        }
    }
    else 
    {
        cnt_end = 0;
    }
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "master_chassis_control");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("flight_dis_x_thre", flight_dis_x_thre);
    nhPrivate.getParam("flight_dis_y_thre", flight_dis_y_thre);
    nhPrivate.getParam("Lidar_Data_Filter_num", Lidar_Data_Filter_num);
    nhPrivate.getParam("Obstacles_Proportion_Thre", Obstacles_Proportion_Thre);
    

    /* 订阅地面机器人的goal point */
    ros::Subscriber sub_goal = nh.subscribe("/move_base_simple/goal", 5, goal_cb);
    /* 订阅来自move_base的起飞指令 */
    ros::Subscriber sub_take_off = nh.subscribe("/take_off", 5, take_off_cb);
    /* 订阅vins的里程计信息 */
    ros::Subscriber sub_odometry = nh.subscribe("/vins_estimator/odometry", 5, vins_odometry_cb);
    /* 订阅uav的运行状态 */
    ros::Subscriber sub_uav_state = nh.subscribe("/uav_state", 5, uav_state_cb);
    /* 订阅激光雷达测距消息 */
    ros::Subscriber sub_lidar_mavros = nh.subscribe("/mavros/distance_sensor/hrlv_ez4_pub", 5, lidar_msg_cb);

    /* 发布无人机的目标点 */
    pub_uav_pose = nh.advertise<geometry_msgs::Point>("/uav_goal_point", 5);
    /* 发布无人机的起飞/降落指令 */
    ros::Publisher pub_takeoff_land = nh.advertise<std_msgs::UInt8>("/uav_takeoff_land", 5);
    /* 发布话题重置move_base状态机 */
    ros::Publisher pub_system_reset = nh.advertise<geometry_msgs::Quaternion>("/system_reset", 5);
    /* 发布目标点 */
    ros::Publisher repub_move_base_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);

    ros::ServiceClient client_empty = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    If_Take_Off.data = false;

    // ros::service::waitForService("/move_base/clear_costmaps");

    ros::Rate rate(30);
    while (ros::ok()) 
    {
        ros::spinOnce();

        if (If_Take_Off.data) 
        {
            /* 发布起飞话题 */
            std_msgs::UInt8 cmd;
            cmd.data = 1;
            pub_takeoff_land.publish(cmd);
            /* 发布goal point */
            geometry_msgs::Point uav_goal;
            uav_goal.x = take_off_info.pose.position.x;
            uav_goal.y = take_off_info.pose.position.y;
            uav_goal.z = take_off_info.pose.position.z;
            pub_uav_pose.publish(uav_goal);
            /* 重置可以起飞的标志位 */
            If_Take_Off.data = false;
        }

        /* 此处为需要手动监视无人机的降落逻辑 */
        if(Need_Lookout)
        {
            if(UAV_Fight_Schedule == Flown_Over_Obstacles)
            {
                // Flown_Over_Obstacles_Odom_X
                if (vin_odom_Info.pose.pose.position.x > (Flown_Over_Obstacles_Odom_X + 1))
                {
                    // geometry_msgs::Point uav_goal_temp;
                    // uav_goal_temp.x = vin_odom_Info.pose.pose.position.x;
                    // uav_goal_temp.y = vin_odom_Info.pose.pose.position.y;
                    // uav_goal_temp.z = vin_odom_Info.pose.pose.position.z;
                    // pub_uav_pose.publish(uav_goal_temp);

                    std_msgs::UInt8 cmd;
                    cmd.data = 2;
                    pub_takeoff_land.publish(cmd);
                }
                
            }

            if (uav_first_take_off && UAV_State == Land) 
            {
                ROS_INFO("\033[1;32m----> UAV has land ! \033[0m");
                /* 重置代价地图 */
                std_srvs::Empty empty;
                bool retu = client_empty.call(empty);
                if(retu)
                {
                    ROS_INFO("\033[1;32m----> The costmaps clear success but goal not reached ! \033[0m");
                }
                else
                {
                    ROS_INFO("\033[1;31m----> The costmaps clear fail and goal not reached ! \033[0m");
                }
                /* 重新发布目标点 */
                repub_move_base_goal.publish(sub_goal_info);
                /* 发布话题重置move_base状态机 */
                geometry_msgs::Quaternion flag;
                flag.w = 5;
                pub_system_reset.publish(flag);
                uav_first_take_off = false;
                // get_Lidar_LandRange_flag = true;
                UAV_Fight_Schedule = Prepare_Over_Obstacles;
                lidar_range_max = 0;
            }
        }
        /* 不需要手动监视，到达目标点自动落地 */
        else 
        {
            /* 不需要监视到达目的地后降落，则代表已经到达目标点 */
            if (uav_first_take_off && UAV_State == Land) 
            {
                ROS_INFO("\033[1;32m----> UAV has land ! \033[0m");
                /* 重置代价地图 */
                std_srvs::Empty empty;
                bool retu = client_empty.call(empty);
                if(retu)
                {
                    ROS_INFO("\033[1;32m----> The costmaps clear success and goal reached ! \033[0m");
                }
                else
                {
                    ROS_INFO("\033[1;31m----> The costmaps clear fail but goal reached ! \033[0m");
                }
                /* 发布话题重置move_base状态机 */
                geometry_msgs::Quaternion flag;
                flag.w = 5;
                pub_system_reset.publish(flag);
                uav_first_take_off = false;
                // get_Lidar_LandRange_flag = true;
                UAV_Fight_Schedule = Prepare_Over_Obstacles;
                lidar_range_max  = 0;
            }
        }
        
        // /* 满足落地条件之后且仍需小车走一段路 */
        // if (0) 
        // {
        //     /* 重置代价地图 */
        //     std_srvs::Empty empty;
        //     bool retu = client_empty.call(empty);
        //     if(retu)
        //     {
        //         ROS_INFO("\033[1;32m----> The costmaps clear success ! \033[0m");
        //     }
        //     else
        //     {
        //         ROS_INFO("\033[1;31m----> The costmaps clear fail ! \033[0m");
        //     }
        //     /* 重新发布目标点 */
        //     repub_move_base_goal.publish(sub_goal_info);
        //     /* 发布话题重置move_base状态机 */
        //     geometry_msgs::Quaternion flag;
        //     flag.w = 5;
        //     pub_system_reset.publish(flag);
        // }
        
        rate.sleep();
    }

    return 0;
}