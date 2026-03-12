

#include "rosNode.h"


void ROSNode::ini(
    ros::NodeHandle   * const rosNodeH,
    SYSTEM_DATA_S * const ssData)
{
    if (rosNodeH==NULL || ssData==NULL) return;

    _nh = rosNodeH;
    _p2c = & ssData->p2c;
    _vs = & ssData->vs;
    _ctrl = & ssData->ctrl;

    //ini
    count = 0;

    //register pub
    pub_cmd = _nh->advertise<cav_msgs::Control>("/Control", 2); 
    
    //register sub
    sub_path         = _nh->subscribe("/PlanedPath", 2, & ROSNode::pathCB, this);
    sub_veh_state    = _nh->subscribe("/VehicleState", 2, & ROSNode::vehStateCB, this);
}


void ROSNode::publishCmd()
{
    if (_nh == NULL || _ctrl == NULL)
        return;

    cav_msgs::Control msg;

    if(count ++ > 1e5)  count = 0;
    msg.timestamp = ros::Time::now().toSec();
    msg.count = count;
    msg.steering_cmd_front_wheel = _ctrl->steering;
    msg.gear_cmd = _ctrl->gear;
    msg.speed_cmd = _ctrl->speed_cmd;
    msg.bywire_control_enable  = 1; //self-driving enabled, 0-off, 1-on

    pub_cmd.publish(msg);
};

