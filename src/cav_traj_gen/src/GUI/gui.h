/*
 * gui.h
 * GUI class for display is defined here. Most of the widgets of Qt gui are defined here.
 * Feel free to add widgets and change their functions.
 * **PART OF THEM CAN BE REVISED**
 */
#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <QBasicTimer>

#include "dataDefine.hpp"
#include "globalDef.hpp"
#include "display_3d.h"
#include "opt.h"

namespace Ui
{
    class GUI;
}

class GUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit GUI(QWidget *parent = 0);
    ~GUI();

public:
private slots:
    void on_horizontalSlider_scale_valueChanged(int value);
    void on_horizontalSlider_X_valueChanged(int value);
    void on_horizontalSlider_Y_valueChanged(int value);

    void on_checkBox_Axis_toggled(bool checked);
    void on_checkBox_Details_toggled(bool checked);
    void on_checkBox_All_Path_toggled(bool checked);
    void on_checkBox_Show_Valid_toggled(bool checked);
    void on_checkBox_Obstacle_toggled(bool checked);
    void on_checkBox_Pred_Path_toggled(bool checked);
    
    void on_checkBox_Ref_Path_toggled(bool checked);
    void on_checkBox_RefPath_Heading_toggled(bool checked);
    void on_checkBox_RefPath_Cr_toggled(bool checked);
    void on_checkBox_BestPath_toggled(bool checked);
    void on_checkBox_BestPath_Heading_toggled(bool checked);
    void on_checkBox_BestPath_Cr_toggled(bool checked);
    
    void on_pushButton_Change_View_clicked(bool clicked);

    void on_checkBox_ay_soft_const_toggled(bool checked);
    void on_checkBox_check_obstacles_toggled(bool checked);

    void on_horizontalSlider_obstacle_sensitivity_valueChanged(int value);
    void on_horizontalSlider_target_time_valueChanged(int value);

    void on_checkBox_test_toggled(bool checked);
    void resizeEvent(QResizeEvent *event);

private:
    Ui::GUI *ui;
    QBasicTimer timer;

    void timerEvent(QTimerEvent *event);
    void initialize();
    void re_ini();
    void update_info();

    bool flag_planning = false;
    bool ini_flag = false;
    long count = 0;
    clock_t t0, t1;
    char text[256];
    double cal_time, plan_time;
    int cmd_num = -1;
};

#endif
