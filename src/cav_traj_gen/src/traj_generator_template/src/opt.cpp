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
	
	// *****************************************************************************/
    //TODO
	// Here is an example of how to generate a trajectory
	copy_generator();
	
	/*****************************************************************************/
	// reformat the trajectory for control 0.04s per point
	genControlPath(&traj_best, TIME_GAP_CONTROL);
	
	return;
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
