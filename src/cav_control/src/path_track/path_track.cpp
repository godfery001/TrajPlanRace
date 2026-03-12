
/*
 * Used for path tracking control
 * by Shaobing Xu
 * email: shaobxu@gmail.com
 *
 */

#include "ros/ros.h"
#include "path_track.h"


double PathTrack::pure_pursuit()
{

    if (_p2c->path.size()< 20)
        return steer_pp; //the angle of last step

    //step 0: set value
    int N_tg = MIN( _p2c->NPN+pp_preview_step, _p2c->path.size()); //1 step is 0.04s
    double tg_x = _p2c->path.at(N_tg-1).x;
    double tg_y = _p2c->path.at(N_tg-1).y;

    //step 1: alpha, ld  
    double alpha = XM::AngleFromXY(_vs->x, _vs->y, _vs->heading, tg_x, tg_y);
    if (_vs->speed_x < 0.2)
        alpha = 0;

    double ld = XM::distFromXY(_vs->x, _vs->y, tg_x, tg_y);
    ld = MAX(_vs->wheelbase*2.0, ld);

    //step 2: steering
    double steer = atan(2*_vs->wheelbase * sin(alpha)/ld);
    
    pp_smooth_cof = MAX(0, MIN(1.0, pp_smooth_cof));
    steer_pp = steer_pp *pp_smooth_cof + steer*(1-pp_smooth_cof);

    //step 3: output
    ROS_INFO_THROTTLE(5, "Ntg%d, ahead-L%.2lf, alpha%.2lf steer%.2lf steer_pp%.2lf wheelbase%.2lf\n", 
            N_tg, ld, alpha*180/M_PI, 
            steer*180/M_PI, steer_pp*180/M_PI,
            _vs->wheelbase);

    return steer_pp;
}



int PathTrack::run()
{
    if (_p2c == NULL || _vs == NULL || _ctrl == NULL )
        return -1;

    count++;

    //step 0: check if the car is near the path or not
    if (std::abs(_p2c->ey) < max_allowed_ey && 
        std::abs(_p2c->ephi) < max_allowed_eh) //rad
        inPathFlag  = true;
    else
    {
        inPathFlag  = false;
        _p2c->Estop = MAX(_p2c->Estop, ESTOP_MEDIUM);
        ROS_WARN_THROTTLE(2, "@path track, Depart from the path, ey:%.2lf/%.2lfm ephi=%.2lf/%.2lfdeg, set to ESTOP_MEDIUM\n",
                _p2c->ey, 
                max_allowed_ey, 
                _p2c->ephi*180.0f/M_PI, 
                max_allowed_eh*180.0f/M_PI);
    }

    //step 1: control
    if (_vs->speed_x > 0.1f)
        _ctrl->steering = pure_pursuit();

    return 0;
}


