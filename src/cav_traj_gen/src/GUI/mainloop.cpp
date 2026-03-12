/*
 * mainloop.cpp
 * This is the main loop for the GUI and whole program.
 * All you want to do in one loop is here.
 * **YOU NEED TO FINISH mainloop.cpp**
 */
#include "gui.h"
#include "ui_gui.h"
#include "XMath.h"

/**
 * @brief Reinitializes system components
 * Resets ROS nodes, trajectory optimizer, and path slicer
 */
void GUI::re_ini()
{
    // Initialize ROS node with current settings
    TG.rosNode.ini(TG._rosnh, &TG.ssData);
    
    // Configure trajectory optimizer with current goal
    TG.opt.ini(&TG.ssData.goal);
    
    // Initialize path slicer for reference path processing
    TG.path_slicer.initialize(&TG.ssData.goal);

    // Reset system flags and counters
    ini_flag = true;
    count = 0;
}

/**
 * @brief 50Hz main control loop
 * @param event Timer event triggering this callback
 * 
 * Execution sequence:
 * 1. System initialization check
 * 2. ROS data processing
 * 3. Local path generation
 * 4. Trajectory planning
 * 5. Result publication
 * 6. GUI updates
 */
void GUI::timerEvent(QTimerEvent *event)
{
    // Validate timer source
    if (timer.timerId() != event->timerId())
        return;

    // Timing and cycle management
    double t0 = ros::Time::now().toSec();
    if (count++ > 9999999) count = 0;

    // Initialization check
    if (!ini_flag) {
        re_ini();
        return;
    }
    if (flag_planning) return; // Prevent re-entry during planning

    // Stage 1: Process incoming ROS messages
    ros::spinOnce();

    // Stage 2: Global to local path conversion
    if (cmd_num != TG.ssData.goal.cmd_num) {
        cmd_num = TG.ssData.goal.cmd_num;
        TG.path_slicer.NPN = -1; // Reset nearest path node index
        return;
    }
    TG.path_slicer.get_local_reference_path();

    // Stage 3: Core planning algorithm
    flag_planning = true;
    double t1 = ros::Time::now().toSec();
    
    // Configure planner parameters from GUI settings
    TG.opt.flag_checkObstacles = guiSet.flag_checkObstacles;
    TG.opt.flag_ay_soft_const = guiSet.flag_ay_soft_const;
    
    // Set motion constraints
    TG.opt.target_time = guiSet.target_time;    // Planning horizon (seconds)
    TG.opt.set_max_speed = 8.0;   // m/s
    TG.opt.set_max_acc = 4.5;     // m/s² (longitudinal)
    TG.opt.set_min_acc = -3;      // m/s² (braking)
    TG.opt.set_max_yawrate = 5/2.4; // rad/s (steering rate limit)
    TG.opt.set_max_acc_y = 2.0;   // m/s² (lateral)
    
    // Execute trajectory optimization
    TG.opt.run();
    plan_time = ros::Time::now().toSec() - t1;

    // Stage 4: Publish planning results to ROS network
    TG.rosNode.publishPlanResult(TG.opt.traj_best.path);

    // Stage 5: Update visualization
    update_info();              // Refresh GUI information panel
    ui->display_3d->updateGL(); // Trigger 3D display redraw

    // Stage 6: Cycle completion
    flag_planning = false;
    cal_time = ros::Time::now().toSec() - t0; // Total cycle time
}