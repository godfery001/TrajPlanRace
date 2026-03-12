
/*
 *
 * Control Module for Lynx and YHS
 * by Shaobing Xu
 * email: shaobxu@tsinghua.edu.cn
 * main.cpp
 *
 */


#include <iostream>
#include "globalDef.hpp"



MOTIONCONTROL MC;
GUI_Set_S guiSet;

void ini()
{
    MC.rosNode.ini(MC._rosnh, &MC.ssData);
    
    //pat tracker
    MC.path_tracker.ini(
        &MC.ssData.p2c,
        &MC.ssData.vs,
        &MC.ssData.ctrl);
    MC.path_tracker.max_allowed_ey = guiSet.ros_max_ey;
    MC.path_tracker.max_allowed_eh = guiSet.ros_max_ephi;

    MC.path_tracker.pp_preview_step = guiSet.look_ahead_step;
    MC.path_tracker.pp_smooth_cof = guiSet.smooth_coe;

    //speed tracker
    MC.speed_tracker.ini(
        &MC.ssData.p2c,
        &MC.ssData.vs,
        &MC.ssData.ctrl);

    MC.speed_tracker.max_speed = 20; //m/s
}



int main(int argc, char *argv[])
{

    //ros
    ros::init (argc, argv, "cav_control");
    ros::NodeHandle nh;
    MC._rosnh = &nh;

    //input
    ros::NodeHandle priv_nh("~");
    priv_nh.getParam("ros_max_ey", guiSet.ros_max_ey);
    priv_nh.getParam("ros_max_ephi", guiSet.ros_max_ephi);
    priv_nh.getParam("look_ahead_step", guiSet.look_ahead_step);
    priv_nh.getParam("smooth_coe", guiSet.smooth_coe);

    printf("\n\n");
    printf("  getParam ros_max_ey               %lf m\n", guiSet.ros_max_ey);
    printf("  getParam ros_max_ephi             %lf degree\n", guiSet.ros_max_ephi);
    printf("  getParam look_ahead_step          %d\n", guiSet.look_ahead_step);
    printf("  getParam smooth_coe               %lf\n", guiSet.smooth_coe);

    guiSet.ros_max_ephi = guiSet.ros_max_ephi * M_PI/180.0f;

    //ros setup and loop
    ini();
    long count = 0;
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        if(count++ > 1e8)
            count = 0;

        //step 1: update recv data
        ros::spinOnce();
        MC.rosNode.gen_p2c();//update tracking info


        //step 2: check in path or not, and path tracking
        MC.path_tracker.run();


        // step 3: speed control
        MC.speed_tracker.run();


        // step 4: check stop/go
        if(MC.ssData.p2c.Go == 0 && 
           MC.ssData.p2c.Estop == ESTOP_NONE)
        {
            MC.speed_tracker.setStop();
            ROS_WARN_THROTTLE(2,"Decision Go = 0, set stop");
        }
        

        // step 5: check recv state
        if (ros::Time::now().toSec() - 
                MC.ssData.p2c.timestamp > 0.5)
        {
            MC.speed_tracker.setEstop();
            ROS_WARN_THROTTLE(5,
                "NOT able to recv decision result in 0.5 seconds, set Estop");
        }


        // step 6: publish commands
        MC.rosNode.publishCmd();
        ROS_INFO_THROTTLE(1,"Control cmd sent %ld",count);


        // step 7： sleep        
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}



