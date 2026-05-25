#ifndef GLOBALDEF_HPP
#define GLOBALDEF_HPP

#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include "dataDefine.hpp"
#include "rosNode.h"
#include "opt.h"
#include "path_slicer.h"

class TRAJ_GEN
{

public:
    ROSNode rosNode;
    ros::NodeHandle *_rosnh = NULL;

    DataPool_S ssData;
    Opt opt;
    Path_Slicer path_slicer;

    // method
    TRAJ_GEN(){};
};

extern TRAJ_GEN TG;

#endif
