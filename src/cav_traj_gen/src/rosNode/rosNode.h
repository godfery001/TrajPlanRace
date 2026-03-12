/*
 * rosNode.h
 * ROSNode class is defined here to handle ROS communication.
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE**
 */
#ifndef ROSNODE_H
#define ROSNODE_H

#include "ros/ros.h"
#include <iostream>
#include "dataDefine.hpp"
#include "opt.h"

class ROSNode
{

public:
    ROSNode(){};
    ~ROSNode(){};

    // ini
    void ini(ros::NodeHandle *const rosNodeH,
             DataPool_S *const ssData);

    // pub
    void publishPlanResult(std::vector<Point_Xd> & vec);
    ros::Publisher pub_plan_result;

    // recv
    ros::Subscriber sub_goal;
    ros::Subscriber sub_vehstate;
    ros::Subscriber sub_obstacles;

    // callback functions
    void goalCB(const cav_msgs::Goal::ConstPtr &msg);
    void vehstateCB(const cav_msgs::VehicleState::ConstPtr &msg);
    void obstaclesCB(const cav_msgs::ObstacleVec::ConstPtr &msg);

private:
    ros::NodeHandle *_nh = NULL;
    // data pool 
    DataPool_S *_ssData = NULL;
};

#endif /*ROSNODE_H*/
