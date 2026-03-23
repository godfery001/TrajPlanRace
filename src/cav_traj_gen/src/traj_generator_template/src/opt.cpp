/*
 * opt.cpp
 * This is the main class for trajectory optimization
 * We already define some structes and variables that might be useful
 * Your main goal is to finish the constructor function ini() and run()
 * **YOU REALLY NEED TO FINISH THIS**
 */
#include <iostream>
#include <algorithm>
#include <math.h>
#include <fstream>

#include "opt.h"
using namespace std;

// Helper functions
// =========================================================================
// 内部辅助函数：计算基准纵向采样距离 (基于梯形速度曲线预估)
// =========================================================================
static double calc_base_lon_dist(double v0, double v_target, double T, double max_acc, double min_acc)
{
    double a = (v_target > v0) ? max_acc : min_acc;
    if (std::abs(a) < 1e-5)
        return v0 * T;

    double t_acc = std::abs((v_target - v0) / a);

    if (t_acc >= T)
    {
        return v0 * T + 0.5 * a * T * T;
    }
    else
    {
        double dist_acc = (v0 + v_target) * t_acc / 2.0;
        double dist_cruise = v_target * (T - t_acc);
        return dist_acc + dist_cruise;
    }
}

// =========================================================================
// 内部辅助函数：根据采样距离，使用梯形曲线反算终端速度
// 公式推导：S = (v0 + vf) * T / 2  =>  vf = 2*S/T - v0
// =========================================================================
static double calc_lon_vf(double delta_s, double v0, double T, double max_speed)
{
    if (T <= 0.01)
        return 0.0;
    double vf = (2.0 * delta_s / T) - v0;
    return std::max(0.0, std::min(vf, max_speed));
}

// =========================================================================
// 内部辅助函数：维护过去10秒历史窗口
// =========================================================================
static void prune_obs_hist(std::deque<std::pair<double, double> > &hist, double now_sec)
{
    while (!hist.empty() && (now_sec - hist.front().first) > 10.0)
        hist.pop_front();
}

static double calc_obs_hist_avg(const std::deque<std::pair<double, double> > &hist)
{
    if (hist.empty())
        return 100.0;

    double sum = 0.0;
    for (size_t i = 0; i < hist.size(); ++i)
        sum += hist[i].second;
    return sum / double(hist.size());
}

// =========================================================================
// 核心采样生成器
// =========================================================================
static void generate_frenet_paths(
    double lon_0, double lon_v0, double lon_a0,
    double lat_0, double lat_v0, double lat_a0,
    double base_time, double max_lat_bound, double target_lon_v,
    double max_speed, double max_acc, double min_acc,
    const std::vector<double> &tf_pcts,
    const std::vector<double> &lat_pcts,
    const std::vector<double> &lon_pcts,
    std::vector<FrenetPath> &candidate_trajs)
{
    candidate_trajs.clear();

    for (double tf_pct : tf_pcts)
    {
        double T = base_time * (1.0 + tf_pct);
        if (T <= 0.1)
            continue;

        double base_lon_dist = calc_base_lon_dist(lon_v0, target_lon_v, T, max_acc, min_acc);

        for (double lat_pct : lat_pcts)
        {
            double lat_f = max_lat_bound * lat_pct;
            double lat_vf = 0.0;
            double lat_af = 0.0;

            QuinticPolynomial lat_qp(lat_0, lat_v0, lat_a0, lat_f, lat_vf, lat_af, T);
            double jerk_cost_lat = lat_qp.calc_jerk_sq_integral(T);

            for (double lon_pct : lon_pcts)
            {
                double delta_lon = base_lon_dist * (1.0 + lon_pct);
                double lon_f = lon_0 + delta_lon;

                double lon_vf = calc_lon_vf(delta_lon, lon_v0, T, max_speed);
                double lon_af = 0.0;

                QuinticPolynomial lon_qp(lon_0, lon_v0, lon_a0, lon_f, lon_vf, lon_af, T);
                double jerk_cost_lon = lon_qp.calc_jerk_sq_integral(T);

                FrenetPath fp;
                fp.cost_lat = jerk_cost_lat;
                fp.cost_lon = jerk_cost_lon;
                fp.lat_f_final = lat_f;
                fp.lon_vf_final = lon_vf;

                for (double t = 0; t <= T; t += TIME_GAP_PATH)
                {
                    fp.t.push_back(t);
                    fp.lat_p.push_back(lat_qp.calc_point(t));
                    fp.lat_v.push_back(lat_qp.calc_vel(t));
                    fp.lat_a.push_back(lat_qp.calc_acc(t));
                    fp.lat_j.push_back(lat_qp.calc_jerk(t));

                    fp.lon_p.push_back(lon_qp.calc_point(t));
                    fp.lon_v.push_back(lon_qp.calc_vel(t));
                    fp.lon_a.push_back(lon_qp.calc_acc(t));
                    fp.lon_j.push_back(lon_qp.calc_jerk(t));
                }
                candidate_trajs.push_back(fp);
            }
        }
    }
}

/**
 * @brief The init function for trajectory optimization class.
 * @todo In this function, you need to initialize the planning environment
 * and other variables that matter for your trajectory generation method.
 * @param planning_env global planning environment in the data pool
 */
int Opt::ini(PlanningEnv_S *planning_env)
{
    _env = planning_env;
    traj_best.path.clear();
    traj_best.feasible = false;

    ui_latest_min_obs_dist = 100.0;
    ui_avg_min_obs_dist_10s = 100.0;
    ui_best_cost_total = 0.0;
    ui_best_cost_jerk = 0.0;
    ui_best_cost_target = 0.0;
    ui_best_cost_bound = 0.0;
    ui_best_cost_obs = 0.0;
    ui_best_min_margin_bound = 100.0;
    ui_candidate_total_num = 0;
    ui_candidate_valid_num = 0;
    min_obs_dist_hist_10s.clear();

    return 0;
}

/**
 * @brief The run function for trajectory optimization class.
 * It is called in a loop (see GUI/widgets/3d_display/mainloop.cpp).
 * @todo In this function, you need to implement your trajectory optimization algorithm.
 * You can use the global planning environment `_env` to access the planning environment.
 * You can also use the global trajectory `traj_best` to store the optimal trajectory.
 */
void Opt::run()
{
    if (_env == NULL || _env->refPathVec.size() < 3)
        return;

    traj_best.path.clear();
    traj_best.feasible = false;

    ui_latest_min_obs_dist = 100.0;
    ui_best_cost_total = 0.0;
    ui_best_cost_jerk = 0.0;
    ui_best_cost_target = 0.0;
    ui_best_cost_bound = 0.0;
    ui_best_cost_obs = 0.0;
    ui_best_min_margin_bound = 100.0;
    ui_candidate_total_num = 0;
    ui_candidate_valid_num = 0;

    double now_sec = ros::Time::now().toSec();
    prune_obs_hist(min_obs_dist_hist_10s, now_sec);
    ui_avg_min_obs_dist_10s = calc_obs_hist_avg(min_obs_dist_hist_10s);

    // -------------------------------------------------------------------
    // Step 1: 参考路径样条化
    // -------------------------------------------------------------------
    XM::update_dist(&_env->refPathVec);
    std::vector<double> s_vec, x_vec, y_vec;
    for (size_t i = 0; i < _env->refPathVec.size(); ++i)
    {
        if (i > 0 && _env->refPathVec[i].mileage <= s_vec.back())
            continue;
        s_vec.push_back(_env->refPathVec[i].mileage);
        x_vec.push_back(_env->refPathVec[i].x);
        y_vec.push_back(_env->refPathVec[i].y);
    }
    if (s_vec.size() < 3)
        return;

    tk::spline csp_x, csp_y;
    csp_x.set_points(s_vec, x_vec);
    csp_y.set_points(s_vec, y_vec);

    // -------------------------------------------------------------------
    // Step 2: Cartesian -> Frenet
    // -------------------------------------------------------------------
    int npn_idx = XM::find_NPN(&_env->refPathVec, _env->x, _env->y);
    npn_idx = MAX(0, MIN((int)_env->refPathVec.size() - 1, npn_idx));
    double lon_0 = _env->refPathVec[npn_idx].mileage;

    double xr = csp_x(lon_0), yr = csp_y(lon_0);
    double dx = csp_x.deriv(1, lon_0), dy = csp_y.deriv(1, lon_0);
    double ddx = csp_x.deriv(2, lon_0), ddy = csp_y.deriv(2, lon_0);

    double theta_r = atan2(dy, dx);
    double kr = (dx * ddy - dy * ddx) / pow(dx * dx + dy * dy, 1.5);

    double delta_theta = _env->heading - theta_r;
    double lat_0 = (_env->y - yr) * cos(theta_r) - (_env->x - xr) * sin(theta_r);

    double lon_v0 = _env->speed_x * cos(delta_theta) / (1.0 - kr * lat_0);
    double lat_v0 = _env->speed_x * sin(delta_theta);

    double lon_a0 = _env->acc_x * cos(delta_theta);
    double lat_a0 = _env->acc_x * sin(delta_theta);

    // -------------------------------------------------------------------
    // Step 3: 采样生成候选轨迹集合
    // -------------------------------------------------------------------
    std::vector<double> tf_pcts = {0.0,-0.60, -0.50, -0.45, -0.30, -0.15,-0.05,0.05, 0.15, 0.30, 0.45, 0.60};
    std::vector<double> lat_pcts;
    for (double p = -0.90; p <= 0.91; p += 0.1)
        lat_pcts.push_back(p);
    std::vector<double> lon_pcts = {0.0};
    for (double p = -0.8; p <= 0.81; p += 0.2)
        lon_pcts.push_back(p);

    candidate_trajs.clear();

    double base_time = target_time;
    double max_lat_bound = 2.0;
    double target_lon_v = set_max_speed * 0.85;

    generate_frenet_paths(
        lon_0, lon_v0, lon_a0,
        lat_0, lat_v0, lat_a0,
        base_time, max_lat_bound, target_lon_v,
        set_max_speed, set_max_acc, set_min_acc,
        tf_pcts, lat_pcts, lon_pcts,
        candidate_trajs);

    ui_candidate_total_num = (int)candidate_trajs.size();

    // -------------------------------------------------------------------
    // Step 4: 坐标回转与约束筛选
    // -------------------------------------------------------------------
    double min_cost = 1e9;
    int best_idx = -1;
    double max_s_spline = s_vec.back();

    double circ_r = sqrt(pow(_env->size_x / 4.0, 2) + pow(_env->size_y / 2.0, 2));

    for (size_t i = 0; i < candidate_trajs.size(); ++i)
    {
        FrenetPath &fp = candidate_trajs[i];
        bool is_valid = true;

        fp.cartesian_path.clear();
        fp.valid = false;

        double min_dist_obs = 100.0;
        double min_margin_bound = 100.0;

        for (size_t j = 0; j < fp.lon_p.size(); ++j)
        {
            if (fp.lon_p[j] > max_s_spline)
            {
                is_valid = false;
                break;
            }

            double x_r = csp_x(fp.lon_p[j]);
            double y_r = csp_y(fp.lon_p[j]);
            double dx_r = csp_x.deriv(1, fp.lon_p[j]);
            double dy_r = csp_y.deriv(1, fp.lon_p[j]);
            double yaw_r = atan2(dy_r, dx_r);

            Point_Xd pt;
            pt.ini();
            pt.x = x_r - fp.lat_p[j] * sin(yaw_r);
            pt.y = y_r + fp.lat_p[j] * cos(yaw_r);
            pt.t = fp.t[j];
            pt.v = sqrt(pow(fp.lon_v[j] * (1 - kr * fp.lat_p[j]), 2) + pow(fp.lat_v[j], 2));
            pt.heading = atan2(fp.lat_v[j], fp.lon_v[j] * (1 - kr * fp.lat_p[j])) + yaw_r;

            fp.cartesian_path.push_back(pt);

            if (pt.v > set_max_speed)
            {
                is_valid = false;
                break;
            }

            if (guiSet.flag_checkObstacles && !_env->obstacleVec.empty()) {
                bool collision = false;
                for (const auto &obs : _env->obstacleVec) {
                    double circ1_x = pt.x + circ_r * cos(pt.heading);
                    double circ1_y = pt.y + circ_r * sin(pt.heading);
                    double circ2_x = pt.x - circ_r * cos(pt.heading);
                    double circ2_y = pt.y - circ_r * sin(pt.heading);

                    double dist1 = sqrt(pow(circ1_x - obs.x_local, 2) + pow(circ1_y - obs.y_local, 2));
                    double dist2 = sqrt(pow(circ2_x - obs.x_local, 2) + pow(circ2_y - obs.y_local, 2));

                    min_dist_obs = std::min(min_dist_obs, std::min(dist1, dist2));

                    if (std::min(dist1, dist2) < (obs.radius + circ_r + 0.2)) {
                        collision = true;
                        break;
                    }
                }
                if (collision) {
                    is_valid = false;
                    break;
                }
            }

            if (!_env->refPathVec.empty()) {
                int search_idx = XM::find_NPN(&_env->refPathVec, pt.x, pt.y);
                search_idx = MAX(0, MIN((int)_env->refPathVec.size() - 1, search_idx));
                const auto &ref_pt = _env->refPathVec[search_idx];

                double dist_to_ref = sqrt(pow(pt.x - ref_pt.x, 2) + pow(pt.y - ref_pt.y, 2));
                double veh_half_width = _env->size_y / 2.0;
                double current_bound = std::min(ref_pt.bound_left, ref_pt.bound_right);

                double margin = current_bound - (dist_to_ref + veh_half_width);

                min_margin_bound = std::min(min_margin_bound, margin);

                if (margin < 0.2) {
                    is_valid = false;
                    break;
                }
            }
        }

        if (is_valid)
        {
            double w_jerk = 1.2;
            double w_target = 1.0;
            double w_bound = 2.0;
            double w_obs = 15.0;

            double cost_jerk = fp.cost_lat + fp.cost_lon;
            double cost_target = std::pow(fp.lat_f_final, 2) + 5.0 * std::pow(target_lon_v - fp.lon_vf_final, 2);
            double cost_bound = 1.0 / (min_margin_bound + 0.001);
            double cost_obs = (min_dist_obs < 3.0) ? (1.0 / std::pow(min_dist_obs + 0.001, 2)) : 0.0;

            fp.cost_jerk = cost_jerk;
            fp.cost_target = cost_target;
            fp.cost_bound = cost_bound;
            fp.cost_obs = cost_obs;
            fp.min_dist_obs = min_dist_obs;
            fp.min_margin_bound = min_margin_bound;
            fp.valid = true;

            fp.cost_total = w_jerk * cost_jerk + w_target * cost_target + w_bound * cost_bound + w_obs * cost_obs;

            ui_candidate_valid_num++;

            if (fp.cost_total < min_cost)
            {
                min_cost = fp.cost_total;
                best_idx = (int)i;
            }
        }
    }

    // -------------------------------------------------------------------
    // Step 5: 最优轨迹定型并移交控制
    // -------------------------------------------------------------------
    if (best_idx != -1) {
        traj_best.path = candidate_trajs[best_idx].cartesian_path;
        XM::cal_heading_by_2pts(&traj_best.path, 0.25, 2);
        bool flag_fwd, flag_R;
        XM::cal_curvature_x(&traj_best.path, 0.25, 4.0, flag_fwd, flag_R);
        traj_best.feasible = true;
        pt_goal = traj_best.path.back();

        const FrenetPath &best_fp = candidate_trajs[best_idx];
        ui_latest_min_obs_dist = best_fp.min_dist_obs;
        ui_best_cost_total = best_fp.cost_total;
        ui_best_cost_jerk = best_fp.cost_jerk;
        ui_best_cost_target = best_fp.cost_target;
        ui_best_cost_bound = best_fp.cost_bound;
        ui_best_cost_obs = best_fp.cost_obs;
        ui_best_min_margin_bound = best_fp.min_margin_bound;

        min_obs_dist_hist_10s.push_back(std::make_pair(now_sec, ui_latest_min_obs_dist));
        prune_obs_hist(min_obs_dist_hist_10s, now_sec);
        ui_avg_min_obs_dist_10s = calc_obs_hist_avg(min_obs_dist_hist_10s);
    }
    else
    {
        prune_obs_hist(min_obs_dist_hist_10s, now_sec);
        ui_avg_min_obs_dist_10s = calc_obs_hist_avg(min_obs_dist_hist_10s);
    }

    genControlPath(&traj_best, TIME_GAP_CONTROL);
    return;
}

/**
 * @brief [Example]This is a very simple example of how to generate a trajectory.
 * Here we simply copy future 50 points of refpath to generate a trajectory.
 */
void Opt::copy_generator()
{
    if (_env == NULL)
        return;
    if (_env->refPathVec.size() < 1)
        return;

    traj_best.path.clear();
    traj_best.feasible = false;

    _env->NPN = XM::find_NPN(&_env->refPathVec, _env->x, _env->y);
    _env->NPN = MAX(0, MIN(_env->refPathVec.size() - 1, _env->NPN));

    int N = _env->refPathVec.size();
    int N_max = MIN(N, _env->NPN + 400);
    printf("N_max=%d\n", N_max);

    pt_goal.x = _env->refPathVec.at(N_max - 1).x;
    pt_goal.y = _env->refPathVec.at(N_max - 1).y;
    pt_goal.heading = _env->refPathVec.at(N_max - 1).heading;

    for (int i = _env->NPN; i < N_max; ++i)
    {
        Point_Xd pt;
        pt.x = _env->refPathVec.at(i).x;
        pt.y = _env->refPathVec.at(i).y;
        pt.heading = _env->refPathVec.at(i).heading;
        pt.t = _env->refPathVec.at(i).t - _env->refPathVec.at(_env->NPN).t;
        pt.v = _env->refPathVec.at(i).v;
        pt.cr = _env->refPathVec.at(i).cr;
        pt.bound_left = _env->refPathVec.at(i).bound_left;
        pt.bound_right = _env->refPathVec.at(i).bound_right;
        pt.mileage = _env->refPathVec.at(i).mileage;
        traj_best.path.push_back(pt);
    }
    traj_best.feasible = true;
}
