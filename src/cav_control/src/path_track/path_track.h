
/*
 *
 * Used for path tracking
 * path_track.h
 *
 */

#ifndef PATHTRACK_HPP
#define PATHTRACK_HPP

#include <iostream> 
#include <algorithm> 
#include <math.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include "xdefine.hpp"

#include "ros/ros.h"

class PathTrack
{

public:
    PathTrack(){};
    ~PathTrack(){};

    void ini(
        Plan_Rlt_S * const p2c_msg,
        VehState_S * const vs,
        Control_Value_S * const ctrl)
    {
        _p2c = p2c_msg;
        _vs = vs;
        _ctrl = ctrl;
    };

    int run();

    bool  inPathFlag = false;


    //----------------------------------------------------------
    //setting
    double max_allowed_ey = 2.0; //meters
    double max_allowed_eh = 45*M_PI/180.f; //45 degrees

    //----------------------------------------------------------
    //setting of pure-pursuit control
    int   pp_preview_step = 40; //time steps, each step is 0.04 seconds
    double pp_smooth_cof = 0.85; //the higher, the smoother [0, 0.95]
    double steer_pp = 0;
    double pure_pursuit();

    //----------------------------------------------------------
    Plan_Rlt_S * _p2c  = NULL;
    Control_Value_S * _ctrl = NULL;

private:
    VehState_S * _vs   = NULL;
    long count = 0;
};

#endif // PATHTRACK_HPP