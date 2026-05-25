
#ifndef GLOBALDEF_HPP
#define GLOBALDEF_HPP

#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "rosNode.h"

#include "dataModel.hpp"
#include "speed_track.h"
#include "path_track.h"




class MOTIONCONTROL
{

public:
    PathTrack   path_tracker;
    SpeedTrack  speed_tracker;
    ROSNode     rosNode;

    ros::NodeHandle   * _rosnh = NULL;
    SYSTEM_DATA_S ssData;

    //method
    MOTIONCONTROL() {};
};


#endif
