/*
 * path_slicer.cpp
 * Path slicer can obtain the local reference path from the global reference path.
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE**
 */
#ifndef PATH_SLICER_H
#define PATH_SLICER_H

#include <iostream>
#include <vector>
#include <cmath>
#include "dataDefine.hpp"

class Path_Slicer
{
public:
    Path_Slicer() = default;
    ~Path_Slicer() = default;

    // Initialize planning environment context
    int initialize(PlanningEnv_S *planning_env);

    // Extract local path segment around current position
    int NPN = -1;  // Original NPN
    void get_local_reference_path();

    // Path containers
    std::vector<X_Point> raw_reference_path;     // Original global path

private:
    double interval = 0.1;  // Original 'gap'
    PlanningEnv_S *_env = nullptr; // Environment context
};

#endif
