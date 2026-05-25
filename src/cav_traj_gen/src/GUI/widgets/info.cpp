/*
 * info.cpp
 * Infomation to be shown in the info box can be changed here.
 * **LEARN AND FEEL FREE TO REVISE**
 */

#include "gui.h"
#include "ui_gui.h"
#include "XMath.h"

void GUI::update_info()
{
    ui->textEdit_info->setStyleSheet("background-color: rgb(100, 100, 100);");
    ui->textEdit_info->setStyleSheet("border-color: rgb(200, 10, 10);");
    ui->textEdit_info->clear();
    
    /************************************************************************/
    /*              You can display any info in text format here            */
    /*                Below is an example you can learn from                */
    /*                    Feel free to revise and debug                     */
    /************************************************************************/
    //time
    ui->textEdit_info->setFontWeight(QFont::Bold);
    ui->textEdit_info->setTextColor(QColor(183, 153, 0));
    sprintf(text,"Plan time:%ldms    All:%ldms",
        long(plan_time*1000), 
        long(cal_time*1000) );
    ui->textEdit_info->setText(text);

    ui->textEdit_info->setTextColor(QColor(183, 183, 183));
    sprintf(text,"Current cmd num:%ld",TG.ssData.goal.cmd_num);
    ui->textEdit_info->append(text);

    /************************************************************************/
    /*                     Do not change anything below                      */
    /************************************************************************/
    //speed
    ui->lcdNumber->display(ABS(TG.ssData.goal.speed_x));
    //battery soc
    ui->battery->setValue(TG.ssData.goal.battery_soc);
}
 
