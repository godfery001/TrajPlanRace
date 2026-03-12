

#ifndef DATAMODEL_CONTROL_HPP
#define DATAMODEL_CONTROL_HPP

#include <iostream> 
#include <unistd.h>
#include <stdbool.h>


#include "xdefine.hpp"
#include "cav_msgs/VehicleState.h"
#include "cav_msgs/Control.h"
#include "cav_msgs/PlanedPath.h"


#define FREQ    (50)

struct SYSTEM_DATA_S
{
    Plan_Rlt_S p2c;
    VehState_S vs;
    Control_Value_S ctrl;
};


//data models are for GUI
struct GUI_Set_S
{
    //max error
    double ros_max_ey = 1.5; //m
    double ros_max_ephi = 80; //degree

    //pure pursuit
    int look_ahead_step = 40;
    double smooth_coe = 0.85;
};

extern GUI_Set_S guiSet;
 

#endif //DATAMODEL_CONTROL_HPP
