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

/**
 * @brief The init function for trajectory optimization class.
 * @todo In this function, you need to initialize the planning environment 
 * and other variables that matter for your trajectory generation method.
 * @param planning_env global planning environment in the data pool
 */
int Opt::ini(PlanningEnv_S *planning_env)
{
	_env = planning_env;
    // TODO
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
	if (_env == NULL)
		return;

	
	// YY edit 
	// *****************************************************************************/
    //TODO
	// Here is an example of how to generate a trajectory
	// copy_generator();
	/*****************************************************************************/
	// reformat the trajectory for control 0.04s per point
	// genControlPath(&traj_best, TIME_GAP_CONTROL);
	// return;

	if (_env == NULL)
        return;

    // 清空上一周期的旧数据
    candidate_trajs.clear();
    traj_best.path.clear();
    traj_best.feasible = false;

    // 更新最近参考点索引
    _env->NPN = XM::find_NPN(&_env->refPathVec, _env->x, _env->y);
    _env->NPN = MAX(0, MIN((int)_env->refPathVec.size()-1, _env->NPN));

    // =====================================================
    // step 1：调用采样模块 (队友的工作，这里放个占位)
    // 把生成的轨迹 push_back 到 candidate_trajs 里




	
    // =====================================================
    // teammate_sampling_module(candidate_trajs); 
    
    // 为了测试代码，暂时保留 copy_generator，
    // 把参考路径当成唯一的一条候选轨迹塞进去测试
    if(candidate_trajs.empty()) {
        copy_generator(); 
        candidate_trajs.push_back(traj_best);
    }

    // =====================================================
    // step 2：执行你的约束筛选与代价评估
    // =====================================================
    filterAndEvaluateTrajectories();

    // =====================================================
    // step 3：框架后处理 (平滑、插值计算，不要动)
    // =====================================================
    if (traj_best.feasible) {
        genControlPath(&traj_best, TIME_GAP_CONTROL);
    } else {
        printf("[Warning] No feasible trajectory found! Emergency Brake!\n");
    }

    return;
	// YY edit 
}


/**
 * @brief [Example]This is a very simple example of how to generate a trajectory.
 * Here we simply copy future 50 points of refpath to generate a trajectory.
*/
void Opt::copy_generator()
{
	if(_env == NULL) return;
	if(_env->refPathVec.size()<1) return;
	// clear the old trajectory
	traj_best.path.clear();
	traj_best.feasible = false;

	// update new NPN
    _env->NPN = XM::find_NPN(&_env->refPathVec, _env->x, _env->y);
	_env->NPN = MAX(0, MIN(_env->refPathVec.size()-1, _env->NPN));

	int N = _env->refPathVec.size();
	int N_max = MIN(N, _env->NPN+400);
    printf("N_max=%d\n", N_max);

	// set target point
	pt_goal.x = _env->refPathVec.at(N_max-1).x;
	pt_goal.y = _env->refPathVec.at(N_max-1).y;
	pt_goal.heading = _env->refPathVec.at(N_max-1).heading;
	
	// find and copy the future 50 points
	for (int i = _env->NPN; i < N_max; ++i)
	{
		Point_Xd pt;
		pt.x = _env->refPathVec.at(i).x;
		pt.y = _env->refPathVec.at(i).y;
		pt.heading = _env->refPathVec.at(i).heading;
		pt.t = _env->refPathVec.at(i).t-_env->refPathVec.at(_env->NPN).t;
		pt.v = _env->refPathVec.at(i).v;
		pt.cr = _env->refPathVec.at(i).cr;
		pt.bound_left = _env->refPathVec.at(i).bound_left;
		pt.bound_right = _env->refPathVec.at(i).bound_right;
		pt.mileage = _env->refPathVec.at(i).mileage;
		traj_best.path.push_back(pt);
	}
	traj_best.feasible = true;

}

// YY edit
// =========================================================================
// 约束筛选模块实现
// =========================================================================
// 1. 碰撞检测约束：将车辆简化为外接圆，与障碍物圆形进行快速距离计算。
bool Opt::checkCollision(Trajectory_S &traj)
{
    // 如果 GUI 关掉了避障检测，或者没有障碍物，直接放行
    if (!guiSet.flag_checkObstacles || _env->obstacleVec.empty()) {
        traj.minDObj = 100.0;
        return true;
    }

    double min_dist = 1000.0;
    // 车辆外接圆半径 (长5宽2，对角线的一半)
    double veh_radius = sqrt(pow(_env->size_x / 2.0, 2) + pow(_env->size_y / 2.0, 2));

    for (const auto &pt : traj.path) {
        for (const auto &obs : _env->obstacleVec) {
            // 计算轨迹点到障碍物中心的欧氏距离
            double dist = sqrt(pow(pt.x - obs.x_local, 2) + pow(pt.y - obs.y_local, 2));
            min_dist = std::min(min_dist, dist);

            // 碰撞条件：两圆心距离 < (车辆半径 + 障碍物半径)
            // 额外加 0.5m 作为安全冗余，数值可调
            if (dist < (obs.radius + veh_radius + 0.5)) {
                traj.feasible = false;
                return false;
            }
        }
    }
    traj.minDObj = min_dist;
    return true;
}

// 2. 道路边界约束：检查轨迹点是否超出了参考线的可行驶边界。
bool Opt::checkBoundary(Trajectory_S &traj)
{
    if (_env->refPathVec.empty()) return false;

    double min_bound_dist = 100.0;
    int search_idx = _env->NPN; // 优化：从车辆当前最近点开始往后搜

    for (const auto &pt : traj.path) {
        // 找离当前轨迹点最近的参考路径点 (使用框架自带的简易搜索)
        search_idx = XM::find_NPN(&_env->refPathVec, pt.x, pt.y);
        search_idx = MAX(0, MIN((int)_env->refPathVec.size() - 1, search_idx));

        const auto &ref_pt = _env->refPathVec[search_idx];
        
        // 计算横向偏差
        double dist_to_ref = sqrt(pow(pt.x - ref_pt.x, 2) + pow(pt.y - ref_pt.y, 2));
        
        // 简易边界判断：横向偏差 + 半车宽 > 道路边界
        double veh_half_width = _env->size_y / 2.0;
        double current_bound = std::min(ref_pt.bound_left, ref_pt.bound_right);
        
        double margin = current_bound - (dist_to_ref + veh_half_width);
        min_bound_dist = std::min(min_bound_dist, margin);

        if (margin < 0.2) { // 留 0.2m 压线余量
            traj.feasible = false;
            return false;
        }
    }
    traj.minDbound = min_bound_dist;
    return true;
}

// 3. 动力学约束：检查轨迹的速度、加速度是否超过了物理极限。
bool Opt::checkDynamics(Trajectory_S &traj)
{
    for (const auto &pt : traj.path) {
        // 检查最大速度
        if (pt.v > set_max_speed) {
            traj.feasible = false;
            return false;
        }
        // 检查加减速 (如果采样层计算了的话)
        if (pt.x_dd > set_max_acc || pt.x_dd < set_min_acc) {
            traj.feasible = false;
            return false;
        }
    }
    return true;
}

// 4. 综合筛选与评价总控函数：遍历队友生成的 candidate_trajs，过三关斩六将，最后算分选出最好的一条。
// 这里是一个简易的代价函数，后面用队友更专业的代价函数替代
void Opt::filterAndEvaluateTrajectories()
{
    traj_best.feasible = false;
    double min_cost = 1e9;
    int best_index = -1;

    for (int i = 0; i < candidate_trajs.size(); ++i) {
        auto &traj = candidate_trajs[i];
        traj.feasible = true; // 假设一开始可行

        // 1. 依次经过三大约束“安检”
        if (!checkDynamics(traj)) continue;
        if (!checkBoundary(traj)) continue;
        if (!checkCollision(traj)) continue;

        // 2. 活下来的轨迹，计算代价 (Cost)
        // 代价函数 = 引导车辆靠近中心线 + 引导车辆远离障碍物
        // 横向偏差代价 (这里简写为距离参考线的均值，实际可以让队友在采样时把终点横向偏移量d传过来)
        double cost_deviation = (3.0 - traj.minDbound) * 10.0; 
        
        // 避障代价 (越靠近障碍物，代价呈指数级上升)
        double cost_obstacle = 0.0;
        if (traj.minDObj < 10.0) {
            cost_obstacle = 50.0 / (traj.minDObj + 0.1); 
        }

        traj.Jcost = cost_deviation + cost_obstacle;

        // 3. 记录全场最低分
        if (traj.Jcost < min_cost) {
            min_cost = traj.Jcost;
            best_index = i;
        }
    }

    // 4. 将胜利者赋值给 traj_best
    if (best_index != -1) {
        traj_best = candidate_trajs[best_index];
        traj_best.feasible = true;
    }
}
// YY edit
