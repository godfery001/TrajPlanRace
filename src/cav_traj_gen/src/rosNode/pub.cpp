/*
 * pub.cpp
 * Publish function and init functions for ROS.
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE**
 */


#include "rosNode.h"

void ROSNode::ini(
    ros::NodeHandle *const rosNodeH,
    DataPool_S *const ssData)
{
    if (rosNodeH == NULL || ssData == NULL)
        return;

    _nh = rosNodeH;
    _ssData = ssData;

    // pub
    pub_plan_result = _nh->advertise<cav_msgs::PlanedPath>("/PlanedPath", 2);

    // recv
    sub_obstacles = _nh->subscribe("/Obstacle", 2, &ROSNode::obstaclesCB, this);
    sub_goal = _nh->subscribe("/Goal", 2, &ROSNode::goalCB, this);
    sub_vehstate = _nh->subscribe("/VehicleState", 2, &ROSNode::vehstateCB, this);
}



void ROSNode::publishPlanResult(std::vector<Point_Xd> & vec)
{
    cav_msgs::PlanedPath msg;

    msg.timestamp = ros::Time::now().toSec();
    msg.Go = _ssData->goal.Go;
    msg.Estop = _ssData->goal.Estop;
    msg.forward_flag = true;
    msg.backward_flag = false;
    cav_msgs::Point pt;
    for (int i = 0; i < vec.size(); ++i)
    {
        pt.t = vec.at(i).t;
        pt.x = vec.at(i).x + _ssData->goal.world_cx;
        pt.y = vec.at(i).y + _ssData->goal.world_cy;
        pt.v = vec.at(i).v;
        pt.cr = vec.at(i).cr;
        pt.heading = vec.at(i).heading;
        msg.planedPath.push_back(pt);
    }

    pub_plan_result.publish(msg);
}
