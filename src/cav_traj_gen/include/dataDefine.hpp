#ifndef DATAMODEL_HPP
#define DATAMODEL_HPP

#include <iostream>
#include "XMath.h"

#include "cav_msgs/Point.h"
#include "cav_msgs/PlanedPath.h"
#include "cav_msgs/VehicleState.h"
#include "cav_msgs/Goal.h"
#include "cav_msgs/Obstacle.h"
#include "cav_msgs/ObstacleVec.h"

#define FREQ (10)


struct Obstacle_S
{
    double timestamp = 0;
    double id = 0;
    double type = 0;
    double radius = 0;

    double x_global = 0;
    double y_global = 0;

    double x_local = 0;
    double y_local = 0;
};

/// @brief data structure for planning environment
/// @details All information you need for planning a trajectory is in here
struct PlanningEnv_S
{
    double timestamp = 0;

    // five parts
    int Estop = 0;
    bool Go = false;
    int Signal = 0;

    // part 1 -- vehicle state
    double x = 0;
    double y = 0;
    double heading = 0;
    double speed_x = 0;
    double acc_x = 0;
    double yaw_rate = 0;

    double size_x = 5.0;
    double size_y = 2.0;
    double size_z = 1.75;
    double wheelbase = 2.85;

    double battery_soc = 0;
    bool by_wire_enabled = false;

    // part 2 -- targets
    double tf = 0;
    double yf = 0;
    double xf = 0;
    double xf_d = 0;
    double xf_dd = 0;

    double tf_min = 0;
    double yf_min = 0;
    double xf_min = 0;

    double tf_max = 0;
    double yf_max = 0;
    double xf_max = 0;
    double vd_yield = 50; // max desired speed for control
    double ad_yield = 5;  // max desired acceleration for control

    // part 3 -- obstacles
    std::vector<Obstacle_S> obstacleVec;

    // part 4 -- lane center and drivable area
    long NPN = 0;
    std::vector<X_Point> refPathVec;

    // part 5 -- reference path
    long cmd_num = 0;
    std::vector<X_Point> whole_path;
    double world_cx = 443673.722475;
    double world_cy = 4429497.481223;
};


/// @brief data definition for data pool
struct DataPool_S
{
    PlanningEnv_S goal;
    long num_valid_path = 0;
};

/// @brief data definition for GUI settings
struct GUI_Set_S
{
    bool test = false;

    double scale_gui = 0;
    double x_gui = 0;
    double y_gui = 0;
    double obstacle_sensitivity = 3;
    double target_time = 5.0;

    int view_mode = 2;

    bool flag_Axis = true;
    bool flag_All_Path = true;
    bool flag_Show_Valid_Only = true;
    bool flag_Details = false;
    bool flag_Obstacle = true;
    bool flag_Pred_Path = true;

    bool flag_Ref_Path = true;
    bool flag_RefPath_Heading = false;
    bool flag_RefPath_Cr = false;

    bool flag_BestPath = true;
    bool flag_BestPath_Heading = false;
    bool flag_BestPath_Cr = false;
    
    bool flag_checkObstacles = true;
    bool flag_ay_soft_const = true;

};



extern GUI_Set_S guiSet;

#endif
