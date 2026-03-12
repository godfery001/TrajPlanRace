#include <iostream>
#include <algorithm>
#include <math.h>
#include <fstream>

#include "opt.h"

template <typename T, typename T1>
void spline_interpolation_by_time(std::vector<T> *const in_vec, 
    std::vector<T1> & out_vec, double time_accurance)
{
    // step 0: prepare
    if (in_vec == NULL) return;
    if (in_vec->size() < 3) return;

    tk::spline spline_x;
    // step 1: set data, filter points too close
    std::vector<double> XX, YY, TIME, SPEED;
    double time_gap = 0;

    for (int i = 0; i < in_vec->size(); ++i)
    {
        if (i > 0) time_gap = in_vec->at(i).t - in_vec->at(i - 1).t;

        if (i == 0 || time_gap > 0.001)
        {
            XX.push_back(in_vec->at(i).x);
            YY.push_back(in_vec->at(i).y);
            TIME.push_back(in_vec->at(i).t);
            SPEED.push_back(in_vec->at(i).v);
        }
    }

    // step 2: set spline
    spline_x.set_boundary(tk::spline::bd_type::second_deriv, 0,
                          tk::spline::bd_type::second_deriv, 0, false);
    spline_x.set_points(TIME, XX);


    // step 3: interpolate
    long N = long(TIME.back() / time_accurance);

    out_vec.clear();
    T1 pt;
    long Ntg;
    double x_d, x_dd, y_d, y_dd, beta;
    for (long i = 0; i < N; ++i)
    {
        pt.t = MIN(i * time_accurance, TIME.back()-0.001);
        spline_x.cal(pt.t, pt.x, x_d, x_dd, Ntg);
        beta = (TIME.at(Ntg + 1) - pt.t) / (TIME.at(Ntg + 1) - TIME.at(Ntg));
        pt.y = YY.at(Ntg) * beta + YY.at(Ntg + 1) * (1 - beta);
        pt.v = SPEED.at(Ntg) * beta + SPEED.at(Ntg + 1) * (1 - beta);
        pt.x_d = pt.v;
        out_vec.push_back(pt);
    }
}


/**
 * @brief After trajectory optimazation, current traj_best may not be suitable for control.
 * We need to reformat it to make sure the time interval of trajectory is t_gap 
 * and heading, cr of each point is calculated.
 * @param traj The trajectiry that need to be reformulated into control valid form. Usually is `traj_best`
 * @param t_gap The time interval of each point in the trajectory. Usually it is `TIME_GAP_CONTROL`
 */
void Opt::genControlPath(Trajectory_S *const traj, double t_gap)
{

    if (traj==NULL) return;
    if (!traj->feasible) return;
    if (traj->path.size()<2) return;

    Trajectory_S t_temp;
    spline_interpolation_by_time(&traj->path, t_temp.path, t_gap);
    traj->path = t_temp.path;

    //update milage
    XM::update_dist(&traj->path);


    // cal curvature and heading
    bool flag_forward = true, flag_R_greater_than_min = true;
    XM::cal_curvature_x(&traj->path, _env->wheelbase*1.5, 
        _env->wheelbase*2, 
        flag_forward,
        flag_R_greater_than_min);

    XM::cal_heading_by_2pts(&traj->path, 0.6, 1); //mode=2: using atan, 1:using approximation
}


