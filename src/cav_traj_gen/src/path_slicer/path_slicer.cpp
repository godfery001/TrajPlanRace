/*
 * path_slicer.cpp
 * Path slicer can obtain the local reference path from the global reference path.
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE**
 */
#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include "path_slicer.h"
#include "UTM.h"
#include "dataDefine.hpp"
// [Initialization] Bind planning environment
int Path_Slicer::initialize(PlanningEnv_S *planning_env)
{
    _env = planning_env;
    return 0;
}


// [Path Extraction] Generate local reference path segment
void Path_Slicer::get_local_reference_path()
{
    // Initial validation
    const long total_path_points = _env->whole_path.size();
    if (total_path_points < 2) {
        if (total_path_points == 0) {
            _env->refPathVec.clear();
        }
        return;
    }

    // Phase 1: Locate nearest path node
    if (NPN < 0) // Initial positioning
    {
        int candidate_index = -1;
        double min_squared_distance = 0;
        XM::find_NPN_preview(&_env->whole_path, 
                            _env->x, _env->y, _env->heading, 
                            candidate_index, 
                            min_squared_distance);

        if (candidate_index < 0 || candidate_index >= total_path_points) return;
        NPN = candidate_index;
    }

    // Phase 2: Extract complete local path segment
    const long local_path_start = MAX(0, NPN - long(4.0/interval)); // 4m backward
    const long local_path_end = MIN(NPN + long(50.0/interval), total_path_points); // 10m forward

    NPN = XM::find_NPN(&_env->whole_path, _env->x, _env->y);
    _env->refPathVec.clear();
    for (long i = local_path_start; i < local_path_end; ++i) {   
        // Calculate accumulated path distance
        if (i == local_path_start) {
            _env->whole_path[i].mileage = 0;
        } else {
            const double dx = _env->whole_path[i].x - _env->whole_path[i-1].x;
            const double dy = _env->whole_path[i].y - _env->whole_path[i-1].y;
            _env->whole_path[i].mileage = SQR(dx) + SQR(dy) + _env->whole_path[i-1].mileage;
        }
        _env->refPathVec.push_back(_env->whole_path[i]);
    }
}