
/*
 *
 * Used to control longitudinal speed
 * speed_track.cpp
 *
 */


#include <iostream>
#include <algorithm>
#include <math.h>
#include "ros/ros.h"

#include "speed_track.h"


void SpeedTrack::ini(
    Plan_Rlt_S * const p2c_msg,
    VehState_S * const vs,
    Control_Value_S * const ctrl)
{
    _p2c = p2c_msg;
    _vs = vs;
    _ctrl = ctrl;

    SC.vd            = 0.0f;
    SC.speed_x       = 0.0f;
    SC.gear_cmd      = GEAR_NONE;
}


int SpeedTrack::run()
{
    if (_p2c == NULL || _vs == NULL || _ctrl == NULL ) return -1;
    if (_p2c->path.size()<3) return -1;
    if (_p2c->NPN >= _p2c->path.size() || _p2c->NPN <0) return -1;
    
    count++;

    //step 1: set the target speed
    SC.speed_x  = MAX(0.0, _vs->speed_x);
    SC.vd = _p2c->vd;

    //step 2: speed limit by road
    SC.v_limit_cr = roadLimitedSpeed(_p2c->cr);


    //step 3: update speed
    _p2c->len = _p2c->path.back().mileage - _p2c->path.at(_p2c->NPN).mileage;
    double len_temp = MAX(0, _p2c->len);
    SC.v_limit_len = sqrt(2*1.0*len_temp);
    SC.vd = MIN(SC.vd, MIN(SC.v_limit_cr, SC.v_limit_len));

    if (_p2c->len < 0.25)
    {
        _p2c->Estop = MAX(_p2c->Estop, ESTOP_MEDIUM);
        ROS_WARN_THROTTLE(2, "@speed track, Check path len <0.25 m, set to ESTOP_MEDIUM");
    }


    //step 4: set gear
    SC.gear_cmd = GEAR_DRIVE;
    

    //step 6: set the cmd
    _ctrl->speed_cmd    = SC.vd;
    _ctrl->gear         = SC.gear_cmd;


    //step 7: set Estop
    if( _p2c->Estop != ESTOP_NONE )
    {        
        setEstop();
        ROS_WARN_THROTTLE(2, "@speed track, Set to Estop, Estop level%d\n", 
            _p2c->Estop);
    }

    return 0;
}


// estimate the max speed limited by road shape
double SpeedTrack::roadLimitedSpeed(double cr)
{
    cr = ABS(cr);
    double maxLatAcc = 1.75f;
    double v = MIN(max_speed, sqrt(maxLatAcc /cr));
    if (cr < 0.001) v = max_speed;
    return v;
}




void SpeedTrack::setStop()
{    
    _ctrl->speed_cmd = 0;
    return;
}


void SpeedTrack::setEstop()
{   
    _ctrl->speed_cmd = 0;
    return;
}





