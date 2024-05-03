#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionGoal.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "master_chassis_control");
    ros::NodeHandle nh;


    return 0;
}