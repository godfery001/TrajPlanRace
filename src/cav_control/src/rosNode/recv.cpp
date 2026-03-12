

#include "rosNode.h"
#include <math.h>
 

void ROSNode::vehStateCB(const cav_msgs::VehicleState::ConstPtr& msg)
{
    if (_vs == NULL) return;

    _vs->timestamp = msg->timestamp;
    _vs->RTK_state_string = msg->RTK_state_string;
    _vs->latitude = msg->RTK_gps_latitude;
    _vs->longitude = msg->RTK_gps_longitude;
    _vs->x = msg->x;
    _vs->y = msg->y;
    _vs->z = msg->z;

    _vs->heading = msg->heading;
    _vs->yaw_rate = msg->yaw_rate;
    _vs->speed_x = msg->speed_x;
    _vs->speed_y = msg->speed_y;
    _vs->acc_x = msg->acc_x;
    _vs->acc_y = msg->acc_y;

    _vs->by_wire_enabled = msg->by_wire_enabled;
    _vs->steer_state_front_wheel = msg->steer_state_front_wheel;
    _vs->brake_state = msg->brake_state;
    _vs->throttle_state = msg->throttle_state;
    _vs->gear_pos = msg->gear_pos;

    _vs->size_x = msg->size_x;
    _vs->size_y = msg->size_y;
    _vs->size_z = msg->size_z;
    _vs->wheelbase = msg->wheelbase;

}




void ROSNode::pathCB(const cav_msgs::PlanedPath::ConstPtr& msg)
{

    if (_p2c == NULL) return;
    
    //set the command
    _p2c->timestamp =msg->timestamp;
    _p2c->Estop =msg->Estop;
    _p2c->Go =msg->Go;

    //copy data
    if (msg->planedPath.size()>=2)
        _p2c->path.clear();

    X_Point pt;
    for (int i = 0; i < msg->planedPath.size(); ++i)
    {
        pt.t = msg->planedPath.at(i).t;
        pt.x = msg->planedPath.at(i).x;
        pt.y = msg->planedPath.at(i).y;
        pt.v = msg->planedPath.at(i).v;
        pt.cr = msg->planedPath.at(i).cr;
        pt.heading = msg->planedPath.at(i).heading;
        _p2c->path.push_back(pt);
    }

    //update mileage
    XM::update_dist(&_p2c->path);
}


void ROSNode::gen_p2c()
{
    auto vec = &_p2c->path;
    if (vec == NULL || vec->size() < 3)
    {
        _p2c->Estop = ESTOP_HIGH; //_env->Estop;
        ROS_WARN_THROTTLE(2,"@gen_p2c, Check the number of path point, less than 3, set to ESTOP_HIGH");
        return;
    }

    // step 1: find the NPN and t_tg on the best trajectory
    double d_min;
    _p2c->NPN = -1;
    XM::find_NPN(vec, _vs->x, _vs->y, _p2c->NPN, d_min);
    if (_p2c->NPN < 0 || _p2c->NPN > vec->size() - 2)
    {
        _p2c->Estop = MAX(_p2c->Estop, ESTOP_MEDIUM);
        ROS_WARN_THROTTLE(2, "@gen_p2c, Check NPN: wrong _p2c->NPN=%d, set to ESTOP_MEDIUM",_p2c->NPN);
        return;
    }


    // step 2: longitudinal
    int N_v = MIN(_p2c->NPN+40, vec->size()-1); //1.6seconds ahead
    _p2c->vd = vec->at(N_v).v;
    _p2c->cr = vec->at(_p2c->NPN).cr;


    // step 3: lateral ey
    _p2c->ey = 0 - XM::dist2LineWithSign(
        _vs->x, _vs->y,
        vec->at(_p2c->NPN).x, vec->at(_p2c->NPN).y,
        vec->at(_p2c->NPN + 1).x, vec->at(_p2c->NPN + 1).y);

    // step 4: heading angle information
    _p2c->ephi = 0 - XM::Normalise_PI(_vs->heading - vec->at(_p2c->NPN).heading);

    return;
}