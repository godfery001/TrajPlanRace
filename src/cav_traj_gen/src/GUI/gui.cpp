/*
 * gui.cpp
 * Slot functions and init functions for widgets of Qt gui are defined here.
 * Feel free to add widgets and change their functions.
 * **PART OF THEM CAN BE REVISED**
 */
#include "gui.h"
#include "ui_gui.h"


GUI::GUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GUI)
{
    ui->setupUi(this);
    move(1,1);
    resize(1850, 1080);

    initialize();

    //enable main loop
    timer.start((int)(1000/FREQ), this);

    setWindowTitle("Trajectory Generation - Tsinghua");
  
}

GUI::~GUI()
{
	delete ui;
}


void GUI::initialize()
{
	ui->horizontalSlider_scale->setValue(0);
    ui->horizontalSlider_X->setValue(0);
    ui->horizontalSlider_Y->setValue(0);
    
    ui->horizontalSlider_obstacle_sensitivity->setValue(3);
    guiSet.obstacle_sensitivity = 3.0f;

    ui->checkBox_Axis->setChecked(true);
    ui->checkBox_Obstacle->setChecked(true);
    ui->checkBox_Ref_Path->setChecked(true);
    ui->checkBox_RefPath_Heading->setChecked(false);
    ui->checkBox_RefPath_Cr->setChecked(false);
    ui->checkBox_BestPath->setChecked(true);
    ui->checkBox_BestPath_Heading->setChecked(false);
    ui->checkBox_BestPath_Cr->setChecked(false);

    ui->checkBox_ay_soft_const->setChecked(true);
    ui->checkBox_check_obstacles->setChecked(true);
    ui->lcdNumber->setDigitCount(4);
    ui->lcdNumber->setSmallDecimalPoint(true);

    ini_flag = false;
}

void GUI::on_horizontalSlider_scale_valueChanged(int value)
{
    guiSet.scale_gui = value; //[-100,100]
}

void GUI::on_horizontalSlider_X_valueChanged(int value)
{
    guiSet.x_gui = value; //[-100,100]
}

void GUI::on_horizontalSlider_Y_valueChanged(int value)
{
    guiSet.y_gui = value; //[-100,100]
}

void GUI::on_checkBox_Axis_toggled(bool checked)
{guiSet.flag_Axis = checked;}

void GUI::on_checkBox_All_Path_toggled(bool checked)
{guiSet.flag_All_Path = checked;}

void GUI::on_checkBox_Show_Valid_toggled(bool checked)
{guiSet.flag_Show_Valid_Only = checked;}

void GUI::on_checkBox_Obstacle_toggled(bool checked)
{guiSet.flag_Obstacle = checked;}

void GUI::on_checkBox_Pred_Path_toggled(bool checked)
{guiSet.flag_Pred_Path = checked;}

void GUI::on_checkBox_Ref_Path_toggled(bool checked)
{guiSet.flag_Ref_Path = checked;}

void GUI::on_checkBox_RefPath_Heading_toggled(bool checked)
{guiSet.flag_RefPath_Heading = checked;}

void GUI::on_checkBox_RefPath_Cr_toggled(bool checked)
{guiSet.flag_RefPath_Cr = checked;}

void GUI::on_checkBox_BestPath_toggled(bool checked)
{guiSet.flag_BestPath = checked;}

void GUI::on_checkBox_BestPath_Heading_toggled(bool checked)
{guiSet.flag_BestPath_Heading = checked;}

void GUI::on_checkBox_BestPath_Cr_toggled(bool checked)
{guiSet.flag_BestPath_Cr = checked;}

void GUI::on_pushButton_Change_View_clicked(bool clicked)
{
    guiSet.view_mode += 1;
    if (guiSet.view_mode>2)
        guiSet.view_mode = 1;
    printf("view mode=%d\n", guiSet.view_mode);

    ui->horizontalSlider_scale->setValue(0);
    ui->horizontalSlider_X->setValue(0);
    ui->horizontalSlider_Y->setValue(0);
}


void GUI::on_checkBox_ay_soft_const_toggled(bool checked)
{guiSet.flag_ay_soft_const = checked;}

void GUI::on_checkBox_check_obstacles_toggled(bool checked)
{guiSet.flag_checkObstacles = checked;}

void GUI::on_horizontalSlider_obstacle_sensitivity_valueChanged(int value)
{
    guiSet.obstacle_sensitivity = 1+(value-3.0)/4.0; //50%-150%
}

void GUI::on_horizontalSlider_target_time_valueChanged(int value)
{
    guiSet.target_time = value;
}

void GUI::on_checkBox_test_toggled(bool checked)
{//guiSet.flag_test = checked;
}


void GUI::on_checkBox_Details_toggled(bool checked)
{guiSet.flag_Details = checked;}


void GUI::resizeEvent(QResizeEvent *event)
{
   // 获取窗口宽度来计算新的字体大小
    int newFontSize = this->height() / 70; // 例如，根据窗口宽度计算字体大小
    
    // 设置父组件的字体大小
    QFont font = this->font();
    font.setPointSize(newFontSize);
    this->setFont(font);

    // 更新所有子组件的字体
    foreach (QWidget* widget, this->findChildren<QWidget*>())
    {
        widget->setFont(font);
    }
    // 调用父类的 resizeEvent
    QMainWindow::resizeEvent(event);
}
