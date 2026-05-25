/*
 * Local trajectory generation
 * main.cpp
 * Several global variables are defined here, including the trajectory generator and GUI settings.
 * ROS node and GUI are initialized and started here.
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE**
 */

 
#include <QApplication>
#include <iostream>

#include "gui.h"
#include "globalDef.hpp"


TRAJ_GEN TG;
GUI_Set_S  guiSet;


int main(int argc, char *argv[])
{
    //ros

    ros::init (argc, argv, "cav_traj_gen");
    ros::NodeHandle nh;
    TG._rosnh = &nh;

    ros::NodeHandle priv_nh("~");

    // gui
	QApplication app(argc, argv);
	QApplication::setStyle("plastique");
	QApplication::setWindowIcon(QIcon("/opt/cav_deploy/icon/traj_gen.png"));
    QApplication::setApplicationName( QString("cav_traj_gen") );

    
    GUI gui;
    gui.show();
    int appReturn = app.exec();

    //exit
    ros::shutdown();
    ROS_WARN("cav_traj_gen Quit");
	return appReturn;
}