
/*
 * Used to control longitudinal speed
 * speed_track.h
 */

#ifndef SPEEDTRACK_H
#define SPEEDTRACK_H

#include <iostream> 
#include <algorithm> 
#include <math.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include "xdefine.hpp"

// data definition for speed control
struct SpeedT_S
{
    double vd        = 0.0f;
    double speed_x   = 0.0f;
    int gear_cmd    = 0.0f;

    double v_limit_len = 15.0f;
    double v_limit_cr = 15.0f;
};
 

class SpeedTrack
{

public:
    SpeedTrack() {};
    ~SpeedTrack() {};

    void ini(Plan_Rlt_S * const p2c_msg,
            VehState_S * const vs,
            Control_Value_S * const ctrl);

    int  run();
    void setStop();
    void setEstop();

    SpeedT_S SC;

    //setting
    double max_speed = 5; //m/ss

    Plan_Rlt_S * _p2c  = NULL;
    VehState_S * _vs   = NULL;
    Control_Value_S * _ctrl = NULL;

private:
    double roadLimitedSpeed(double cr);
    long count = 0;
};

#endif
