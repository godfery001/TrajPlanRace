/*
 * recv.cpp
 * Callback functions for receiving messages for ROS.
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE**
 */

#include <math.h>
#include <algorithm>
#include "rosNode.h"
#include "globalDef.hpp"


// global reference path message
void ROSNode::goalCB(const cav_msgs::Goal::ConstPtr &msg)
{
    if (_ssData == NULL || msg == NULL)
        return;

    if (msg->refPath.size() < 2)
        return;
    auto goal = &_ssData->goal;

    // update Go and Estop
    goal->Go = msg->Go;
    goal->Estop = msg->Estop;
    goal->timestamp = msg->timestamp;

    // update task path
    goal->cmd_num += 1;
    goal->NPN = -1;
    goal->whole_path.clear();
    X_Point pt;
    double distance = 0.0;
    for (int i = 0; i < msg->refPath.size(); ++i)
    {
        auto _p = &msg->refPath.at(i);
        pt.t            = _p->t;
        pt.x            = _p->x - goal->world_cx;
        pt.y            = _p->y - goal->world_cy;
        pt.heading      = _p->heading;
        pt.cr           = _p->cr;
        pt.bound_left   = _p->left_width;
        pt.bound_right  = _p->right_width;
        pt.v            = _p->v;
        goal->whole_path.push_back(pt);
    }
    //update mileage
    XM::update_dist(&goal->whole_path);
    
}


// vehicle state message
void ROSNode::vehstateCB(const cav_msgs::VehicleState::ConstPtr &msg)
{
    if (_ssData == NULL || msg == NULL)
        return;
    auto goal = &_ssData->goal;

    goal->x = msg->x - goal->world_cx;
    goal->y = msg->y - goal->world_cy;
    goal->heading = msg->heading;
    goal->speed_x = msg->speed_x;
    goal->acc_x = msg->acc_x;
    goal->yaw_rate = msg->yaw_rate;

    goal->size_x = msg->size_x;
    goal->size_y = msg->size_y;
    goal->size_z = msg->size_z;
    goal->wheelbase = msg->wheelbase;
    
    goal->battery_soc = msg->battery_SOC;
    goal->by_wire_enabled = msg->by_wire_enabled;
}


// static obstacles message
void ROSNode::obstaclesCB(const cav_msgs::ObstacleVec::ConstPtr &msg)
{
    if (_ssData == NULL || msg == NULL)
        return;
    auto goal = &_ssData->goal;

    Obstacle_S obj;
    goal->obstacleVec.clear();
    for (int i = 0; i < msg->obstacleVec.size(); ++i)
    {
        auto inObj = &msg->obstacleVec.at(i);
        obj.timestamp = inObj->timestamp;
        obj.id = inObj->id;
        obj.type = inObj->type;
        obj.radius = inObj->radius;
        obj.x_global = inObj->x_global;
        obj.y_global = inObj->y_global;
        obj.x_local = inObj->x_global- goal->world_cx;
        obj.y_local = inObj->y_global- goal->world_cy;
        goal->obstacleVec.push_back(obj);
    }
}
