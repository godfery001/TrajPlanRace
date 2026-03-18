#include <stdio.h>
#include "display_3d.h"

static inline bool trajValid(const Trajectory_S &traj)
{
    return !traj.path.empty();
}

void DISPLAY_3D::draw_mode1()
{
    if (guiSet.flag_Axis)
        showAxis();

    drawVehicle();

    // 目标点
    glColor3f(136.0/255.0, 238.0/255.0, 170.0/255.0);
    D3D::drawOneTriangle(TG.opt.pt_goal.x, TG.opt.pt_goal.y,
                         TG.opt.pt_goal.heading + M_PI,
                         M_PI / 6.0,
                         1.0, 0.2, true);

    D3D::drawOneCircle(TG.opt.pt_goal_real.x, TG.opt.pt_goal_real.y,
                       0.25, 20, 0.2, true);

    // 障碍物
    if (guiSet.flag_Obstacle)
        drawObstacle(&TG.ssData.goal.obstacleVec);

    // 局部参考线
    if (guiSet.flag_Ref_Path)
    {
        drawTraj(&TG.ssData.goal.refPathVec,
                 85.0/255.0, 204.0/255.0, 238.0/255.0,
                 1, 2.0, 0.25,
                 false, true,
                 guiSet.flag_RefPath_Heading,
                 guiSet.flag_RefPath_Cr,
                 false);
    }

    // 全局参考线
    drawTraj(&TG.ssData.goal.whole_path,
             1.0, 170.0/255.0, 0.0,
             2, 1.5, 0.15,
             false, true,
             false, false,
             true);

    // 所有候选轨迹
    if (guiSet.flag_All_Path)
    {
        for (size_t i = 0; i < TG.opt.candidate_trajs.size(); ++i)
        {
            const Trajectory_S &traj = TG.opt.candidate_trajs[i];
            if (!trajValid(traj)) continue;

            if (guiSet.flag_Show_Valid_Only && !traj.feasible)
                continue;

            if (traj.feasible)
            {
                // 可行轨迹：绿色
                drawTraj((std::vector<Point_Xd> *)&traj.path,
                         0.15, 0.80, 0.20,
                         2, 1.3, 0.32,
                         false, true,
                         false, false,
                         false);
            }
            else
            {
                // 不可行轨迹：灰色
                drawTraj((std::vector<Point_Xd> *)&traj.path,
                         0.60, 0.60, 0.60,
                         2, 1.0, 0.30,
                         false, true,
                         false, false,
                         false);
            }
        }
    }

    // 再额外强调一次 feasible_trajs（可选层）
    if (guiSet.flag_All_Path && !guiSet.flag_Show_Valid_Only)
    {
        for (size_t i = 0; i < TG.opt.feasible_trajs.size(); ++i)
        {
            const Trajectory_S &traj = TG.opt.feasible_trajs[i];
            if (!trajValid(traj)) continue;

            drawTraj((std::vector<Point_Xd> *)&traj.path,
                     0.0, 0.95, 0.35,
                     3, 1.6, 0.38,
                     false, true,
                     false, false,
                     false);
        }
    }

    // 最优轨迹：黄色粗线高亮
    if (guiSet.flag_BestPath && TG.opt.traj_best.feasible && !TG.opt.traj_best.path.empty())
    {
        drawTraj(&TG.opt.traj_best.path,
                 1.0, 0.95, 0.0,
                 1, 2.8, 0.50,
                 false, true,
                 guiSet.flag_BestPath_Heading,
                 guiSet.flag_BestPath_Cr,
                 false);
    }
}

template <typename T>
void DISPLAY_3D::drawTraj(std::vector<T> *const vec,
                          double r, double g, double b,
                          int gap, double line_width, double level,
                          bool flag_point, bool flag_line,
                          bool flag_heading, bool flag_cr, bool flag_bound)
{
    if (vec == NULL || vec->empty()) return;
    if (gap <= 0) gap = 1;

    if (flag_point)
        D3D::drawTraj_points(vec, r, g, b, gap, level, 0.15, true, 0.5);

    if (flag_line)
        D3D::drawTraj_line(vec, r, g, b, gap, level, line_width);

    if (flag_heading)
        D3D::drawTraj_heading(vec, r * 0.6, g * 0.6, b * 0.6, gap, level + 0.02, 0.5);

    if (flag_cr)
        D3D::drawTraj_curvature(vec, r * 0.6, g * 0.6, b * 0.6, gap, level + 0.02, 0.5);

    if (flag_bound)
        D3D::drawTraj_bound(vec, 1.0, 0.6, 0.3, gap, level + 0.01, 1.0, true, true);
}

void DISPLAY_3D::drawOnePath(Trajectory_S *const traj)
{
    if (traj == NULL || traj->path.empty()) return;

    int gap = (traj->path.size() > 16) ? 2 : 1;

    if (traj->feasible)
    {
        D3D::drawTraj_line(&traj->path, 0.15, 0.80, 0.20, gap, 0.30, 1.2);
    }
    else
    {
        D3D::drawTraj_line(&traj->path, 0.60, 0.60, 0.60, gap, 0.28, 1.0);
    }
}

void DISPLAY_3D::drawObstacle(std::vector<Obstacle_S> *const p)
{
    if (p == NULL) return;

    glLineWidth(1.0);
    glColor3f(1.0, 0.5, 0.5);

    for (size_t i = 0; i < p->size(); ++i)
    {
        D3D::drawOneCircle(p->at(i).x_local,
                           p->at(i).y_local,
                           p->at(i).radius,
                           20, 0.2, true);
    }
}

void calcNormVec(
    double x1, double y1, double z1,
    double x2, double y2, double z2,
    double &x_norm, double &y_norm, double &z_norm)
{
    x_norm = y1 * z2 - y2 * z1;
    y_norm = x2 * z1 - x1 * z2;
    z_norm = x1 * y2 - x2 * y1;

    double length = sqrt(SQR(x_norm) + SQR(y_norm) + SQR(z_norm));
    if (length < 1e-6)
    {
        x_norm = 0.0;
        y_norm = 0.0;
        z_norm = 1.0;
        return;
    }

    x_norm /= length;
    y_norm /= length;
    z_norm /= length;
}

