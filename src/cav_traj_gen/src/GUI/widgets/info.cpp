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
    ui->textEdit_info->setFontWeight(QFont::Bold);
    ui->textEdit_info->setTextColor(QColor(183, 153, 0));
    sprintf(text, "Plan time:%ldms    All:%ldms",
        long(plan_time * 1000),
        long(cal_time * 1000));
    ui->textEdit_info->setText(text);

    ui->textEdit_info->setTextColor(QColor(183, 183, 183));
    sprintf(text, "Current cmd num:%ld", TG.ssData.goal.cmd_num);
    ui->textEdit_info->append(text);

    ui->textEdit_info->append("----------------------------------------");

    sprintf(text, "Obstacle min dist (latest plan): %.3fm", TG.opt.ui_latest_min_obs_dist);
    ui->textEdit_info->append(text);

    sprintf(text, "Obstacle avg min dist (past 10s): %.3fm", TG.opt.ui_avg_min_obs_dist_10s);
    ui->textEdit_info->append(text);

    sprintf(text, "Candidates total/valid: %d / %d",
        TG.opt.ui_candidate_total_num,
        TG.opt.ui_candidate_valid_num);
    ui->textEdit_info->append(text);

    sprintf(text, "Best cost total: %.6f", TG.opt.ui_best_cost_total);
    ui->textEdit_info->append(text);

    sprintf(text, "  jerk  : %.6f", TG.opt.ui_best_cost_jerk);
    ui->textEdit_info->append(text);

    sprintf(text, "  target: %.6f", TG.opt.ui_best_cost_target);
    ui->textEdit_info->append(text);

    sprintf(text, "  bound : %.6f", TG.opt.ui_best_cost_bound);
    ui->textEdit_info->append(text);

    sprintf(text, "  obs   : %.6f", TG.opt.ui_best_cost_obs);
    ui->textEdit_info->append(text);

    sprintf(text, "Best path min boundary margin: %.3fm", TG.opt.ui_best_min_margin_bound);
    ui->textEdit_info->append(text);

    /************************************************************************/
    /*                     Do not change anything below                      */
    /************************************************************************/
    ui->lcdNumber->display(ABS(TG.ssData.goal.speed_x));
    ui->battery->setValue(TG.ssData.goal.battery_soc);
}
