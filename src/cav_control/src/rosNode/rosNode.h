

#ifndef ROSNODE_H
#define ROSNODE_H

#include "ros/ros.h"
#include <iostream>


#include "cav_msgs/VehicleState.h"
#include "cav_msgs/Control.h"
#include "cav_msgs/PlanedPath.h"

#include "dataModel.hpp"


class ROSNode
{

public:
    ROSNode(){};
    ~ROSNode(){};
    void ini(ros::NodeHandle * const rosNodeH,
             SYSTEM_DATA_S * const ssData);
    
    void publishCmd();
    void gen_p2c();

private:
    ros::NodeHandle         * _nh   = NULL;
    Plan_Rlt_S              * _p2c  = NULL;
    VehState_S              * _vs   = NULL;
    Control_Value_S         * _ctrl = NULL;

    long count = 0;

    //pub
    ros::Publisher  pub_cmd;
    
    //recv
    ros::Subscriber sub_veh_state;
    ros::Subscriber sub_path;
    void vehStateCB(const cav_msgs::VehicleState::ConstPtr& msg);
    void pathCB(const cav_msgs::PlanedPath::ConstPtr& msg);

};

#endif /*ROSNODE_H*/


 