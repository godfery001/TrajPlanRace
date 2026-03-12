/*
 * display_3d_mode1.cpp
 * Draw functions from birdeye view (view_mod = 1).
 * The birdeye view is an imporatnt view for your debug
 * You can utlize this display to varify your algorithm by showing the path, obstacles, and vehicle.
 * **YOU CAN REVISE CODE HERE FOR DISPLAY**
*/
/*
 * display_3d_mode1.cpp
 * Bird's-eye view rendering implementation (View Mode 1)
 * Primary debug view for path planning visualization
 * Enable through guiSet.view_mode=1
 */
#include <stdio.h>
#include "display_3d.h"

/**
 * @brief Main rendering pipeline for bird's-eye view
 * Visualizes 7 key components:
 * 1. Coordinate grid system
 * 2. Ego vehicle model
 * 3. Target markers (triangle + circle)
 * 4. Obstacle representations
 * 5. Local reference path
 * 6. Global reference path
 * 7. Optimal trajectory (if available)
 */
void DISPLAY_3D::draw_mode1()
{
    // Stage 1: Draw 1m grid system
    if (guiSet.flag_Axis)
        showAxis();

    // Stage 2: Render ego vehicle at current position
    drawVehicle();

    /**********************************************************************************/
    /*                   Things to be drawn per loop are decided here                 */
    /*           Learn how to draw and feel free to revise them as your need          */
    /**********************************************************************************/
    // TODO: Implement visualization for each component

    // Stage 3: Draw target indicators
    // Orange triangle (30 degree angle) + solid circle
    glColor3f(136.0/255.0, 238.0/255.0, 170.0/255.0); // RGB(136, 238, 170) approximation
    D3D::drawOneTriangle(TG.opt.pt_goal.x, TG.opt.pt_goal.y,
        TG.opt.pt_goal.heading+M_PI, M_PI/6.0, // 30 degree arrow
        1.0, 0.2, true); // Size: 1.0m base, 0.2m height

    D3D::drawOneCircle(TG.opt.pt_goal_real.x, TG.opt.pt_goal_real.y, 
        0.25, 20, 0.2, true); // Radius: 0.25m, 20 segments

    // Stage 4: Draw obstacle bounding boxes
    if (guiSet.flag_Obstacle)
    {
        drawObstacle(&TG.ssData.goal.obstacleVec);
    }

    // Stage 5: Draw local reference path (blue)
    if (guiSet.flag_Ref_Path)
    {
        drawTraj(&TG.ssData.goal.refPathVec,
                 85.0/255.0, 204.0/255.0, 204.0/255.0, // blue
                 1,     // Plot every point
                 2.0,   // Line width
                 0.3,   // Z-level offset
                 false, // Disable point markers
                 true,  // Enable path lines
                 guiSet.flag_RefPath_Heading, // Heading arrows
                 guiSet.flag_RefPath_Cr,      // Curvature markers
                 false); // Disable boundaries
    }

    // Stage 6: Draw global reference path (light orange with boundaries)
    drawTraj(&TG.ssData.goal.whole_path,
                1, 170.0/255.0, 0, // light orange color
                1,     // Plot every point
                2.0,   // Line width
                0.3,   // Z-level offset
                false, // Disable point markers
                true,  // Enable path lines
                guiSet.flag_RefPath_Heading, 
                guiSet.flag_RefPath_Cr, 
                true); // Enable boundaries

    // Stage 7: Draw optimal path (yellow) if valid
    if (guiSet.flag_BestPath && TG.opt.traj_best.feasible)
    {
        drawTraj(&TG.opt.traj_best.path,
                 0.9, 0.9, 0, // Yellow color
                 1,    // Plot every point
                 1.0,  // Line width
                 0.2,  // Z-level offset
                 true,  // Enable point markers
                 false, // Disable path lines
                 guiSet.flag_BestPath_Heading, 
                 guiSet.flag_BestPath_Cr, 
                 false);
    }
}

/**
 * @brief Generic trajectory visualization template
 * @tparam T Trajectory point type (X_Point or similar)
 * @param vec Trajectory data container
 * @param r Red component [0.0-1.0]
 * @param g Green component [0.0-1.0]
 * @param b Blue component [0.0-1.0]
 * @param gap Point sampling interval (1=all points)
 * @param line_width Visual line width in pixels
 * @param level Z-axis offset for layer management
 * @param flag_point Toggle point markers
 * @param flag_line Toggle path lines
 * @param flag_heading Toggle orientation arrows
 * @param flag_cr Toggle curvature visualization
 * @param flag_bound Toggle safety boundaries
 */
template <typename T>
void DISPLAY_3D::drawTraj(std::vector<T> *const vec,
                          double r, double g, double b, 
                          int gap, double line_width, double level,
                          bool flag_point, bool flag_line,
                          bool flag_heading, bool flag_cr, bool flag_bound)
{
    if (!vec || vec->empty()) return;

    // Visualize discrete points with 0.15m markers
    if (flag_point)
        D3D::drawTraj_points(vec, r, g, b, gap, level, 0.15, true, 0.5);

    // Draw continuous path lines
    if (flag_line)
        D3D::drawTraj_line(vec, r, g, b, gap, level, line_width);

    // Display heading indicators (60% base color)
    if (flag_heading)
        D3D::drawTraj_heading(vec, r*0.6, g*0.6, b*0.6, gap, level*0.8, 0.5);

    // Show curvature markers (60% base color)
    if (flag_cr)
        D3D::drawTraj_curvature(vec, r*0.6, g*0.6, b*0.6, gap, level*0.8, 0.5);

    // Render safety boundaries (orange)
    if (flag_bound)
        D3D::drawTraj_bound(vec, 1, 0.6, 0.3, gap, level*0.8, 1.0, true, true);
}

/**
 * @brief Visualizes single trajectory candidate
 * @param traj Trajectory container with path data
 * - Gray path lines with 1.0m spacing
 * - Rectangular markers for valid trajectories
 */
void DISPLAY_3D::drawOnePath(Trajectory_S *const traj)
{
    if (!traj || traj->path.empty()) return;

    // Set default gray color (RGB 153,153,153)
    glColor3f(0.6, 0.6, 0.6);

    auto path = &traj->path;
    if (path->size() < 2) return;

    // Configure point sampling: 2x skip for paths >16 points
    int gap = path->size() > 16 ? 2 : 1;

    // Draw main path line (dark gray)
    D3D::drawTraj_line(path, -1., -1., -1., gap, 0.001, 1.0);

    // Draw point markers (0.15m circles)
    D3D::drawTraj_points(path, -1., -1., -1., gap, 0.001, 0.15, true, 0.1);
}

/**
 * @brief Renders 2D obstacle representations
 * @param p Obstacle container with position/size data
 * - Yellow boxes (RGB 128,128,0)
 * - 0.2m height for 3D effect
 */
void DISPLAY_3D::drawObstacle(std::vector<Obstacle_S> *const p)
{
    if (!p) return;

    glLineWidth(1.0);
    glColor3f(1, 0.5, 0.5); // 255, 170, 170
    
    for (int i = 0; i < p->size(); ++i) {
        D3D::drawOneCircle(p->at(i).x_local, p->at(i).y_local, p->at(i).radius,20,0.2,true);
    }
}

/**
 * @brief Computes normalized vector perpendicular to input vectors
 * @param x1,y1,z1 Components of first vector
 * @param x2,y2,z2 Components of second vector
 * @param x_norm,y_norm,z_norm Output normalized orthogonal vector
 * 
 * Calculates cross product and normalizes to unit length:
 * result = (v1 × v2) / ||v1 × v2||
 */
void calcNormVec(
    double x1, double y1, double z1, 
    double x2, double y2, double z2, 
    double &x_norm, double &y_norm, double &z_norm)
{
    // Cross product calculation
    x_norm = y1 * z2 - y2 * z1;
    y_norm = x2 * z1 - z1 * z2; // Note: Original formula preserved
    z_norm = x1 * y2 - x2 * y1;

    // Normalization
    double length = sqrt(SQR(x_norm) + SQR(y_norm) + SQR(z_norm));
    x_norm /= length;
    y_norm /= length;
    z_norm /= length;
}