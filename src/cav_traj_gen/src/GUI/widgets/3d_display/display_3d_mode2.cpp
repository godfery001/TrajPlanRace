/*
 * display_3d_mode1.cpp
 * Draw functions from driver view (view_mod = 2).
 * Actually, driver mode is not heplful for debug, but it is cool to display.
 * So you don't need to revise this file.
 * **YOU CAN REVISE CODE HERE FOR DISPLAY**
*/
#include <math.h>
#include <stdio.h>
#include "display_3d.h"

/**
 * @brief Main rendering pipeline for driver perspective
 * Visualizes 6 key components:
 * 1. Coordinate grid system (optional)
 * 2. Ego vehicle model
 * 3. Global trajectory (translucent green)
 * 4. Global reference path (semi-transparent orange)
 * 5. Historical path trail (solid green)
 * 6. 3D obstacle models
 */
void DISPLAY_3D::draw_mode2()
{
    // Stage 1: Draw coordinate grid if enabled
    if (guiSet.flag_Axis)
        showAxis();

    // Stage 2: Render ego vehicle
    drawVehicle();

    // Enable alpha blending for translucent effects
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Stage 3: Draw optimal trajectory (green, fully transparent)
    auto _traj_best = &TG.opt.traj_best.path;
    drawTraj_Alp(_traj_best,
                  119.0f/255.0f, 170.0f/255.0f, 238.0f/255.0f, 0.0f, 
                  1, 
                  TG.ssData.goal.size_y*1.0,  // Line width scaled by vehicle width
                  0.01, 
                  false);

    // Stage 4: Draw global path (orange, 80% opaque)
    drawTraj_Alp(&TG.ssData.goal.whole_path,
                  1.0f, 170.0/255.0, 0.0f, 0.8f, 
                  1,
                  TG.ssData.goal.size_y*0.3,  // Narrower line width
                  0.005, 
                  true);

    // Stage 5: Draw historical path (solid green)
    drawTraj_Alp(&pastPath,
                  85.0f/255.0f, 136.0f/255.0f, 238.0f/255.0f, 1.0f,  
                  1,
                  TG.ssData.goal.size_y*1.0,
                  0.01, 
                  false);

    // Draw connection marker between history and current path
    glColor4f(119.0f/255.0f, 170.0f/255.0f, 238.0f/255.0f, 1.0f);  // blue
    if (_traj_best->size() > 2)
        D3D::drawOnePT( pastPath.back().x, pastPath.back().y, pastPath.back().heading,
                    _traj_best->at(0).x, 
                    _traj_best->at(0).y, 
                    _traj_best->at(0).heading,
                    TG.ssData.goal.size_y*0.5, 0.01);

    // Stage 6: Render 3D obstacles
    if (guiSet.flag_Obstacle)
        drawObstacle_3d(&TG.ssData.goal.obstacleVec);

    glDisable(GL_BLEND);
}

/**
 * @brief Renders 3D obstacle models with proper transformations
 * @param obstVec Obstacle data container
 * - Applies local coordinate offsets (x_local, y_local)
 * - Rotates models according to global heading
 * - Uses obstacle dimensions (size_x, size_y, size_z)
 * - Renders orange boxes (RGB 230,51,0) with 80% opacity
 */
void DISPLAY_3D::drawObstacle_3d(std::vector<Obstacle_S> *const obstVec)
{
    if (!obstVec) return;
    
    glLineWidth(1.0);
    for (int i = 0; i < obstVec->size(); ++i) {
        auto p = &obstVec->at(i);
        // Apply local coordinate transformation
        glTranslatef(p->x_local, p->y_local, 0);
        // Render 3D box centered at (0,0) with height offset
        D3D::drawOne3DObj_cylinder(
            0.0,0.0,0.5,
            p->radius,1,
            0.9,0.2,0.0,0.8);
        // Restore transformation matrix
        glTranslatef(-p->x_local, -p->y_local, 0);
    }
}

/**
 * @brief Renders trajectory with distance-based alpha attenuation
 * @tparam T Trajectory point type
 * @param vec Trajectory data container
 * @param r Red component [0.0-1.0]
 * @param g Green component [0.0-1.0]
 * @param b Blue component [0.0-1.0]
 * @param alp Base alpha value [0.0-1.0]
 * @param gap Point sampling interval
 * @param width Line width in meters
 * @param level Z-axis offset
 * @param en_bound Toggle boundary visualization
 * 
 * Alpha calculation: alpha = 1.0 - i*(1-alp)/Nmax
 * Creates fade-out effect along trajectory length
 */
template <typename T>
void DISPLAY_3D::drawTraj_Alp(std::vector<T> *const vec,
                          double r, double g, double b, double alp,
                          int gap, double width, double level, bool en_bound)
{
    if (!vec || vec->size() < 3) return;

    const int Nmax = vec->size();
    for (int i = 0; i < Nmax-1; ++i) { 
        // Calculate fading alpha: full opacity at start, decreasing linearly
        const float fadeAlpha = 1.0f - i*(1.0f-alp)/Nmax;
        glColor4f(r, g, b, fadeAlpha);
        
        // Draw line segment between consecutive points
        auto p1 = vec->at(i);
        auto p2 = vec->at(i+1);
        D3D::drawOnePT(p1.x, p1.y, p1.heading,
                      p2.x, p2.y, p2.heading,
                      width*0.5,  // Half width for bilateral rendering
                      level);
    }

    // Draw boundaries with 60% base color intensity
    if (en_bound)
        D3D::drawTraj_bound(vec, r*0.8, g*0.8, b*0.8, 1, level, 2.0, true, true);
}

/**
 * @brief Maintains historical path buffer
 * - Stores recent trajectory points from optimal path
 * - Keeps maximum 500 points (≈25 seconds at 20Hz)
 * - Filters points closer than 0.2m
 * - Clears buffer when jumps >3m detected
 */
void DISPLAY_3D::genPastPath()
{
    if (TG.opt.traj_best.path.size() < 2 || 
        !TG.opt.traj_best.feasible) 
        return;
    
    X_Point pt = {
        TG.opt.traj_best.path[0].x,
        TG.opt.traj_best.path[0].y,
        TG.opt.traj_best.path[0].heading
    };

    if (pastPath.empty()) {
        pastPath.push_back(pt);
    } else {
        const float dist = XM::distFromXY(pt, pastPath.back());
        if (dist > 3.0f) pastPath.clear();  // Reset on large position jumps
        if (dist > 0.2f) pastPath.push_back(pt);  // 0.2m filtering
    }

    XM::cal_heading_by_2pts(&pastPath, 0.6, 1);

    // Maintain circular buffer
    if (pastPath.size() > 500)
        pastPath.erase(pastPath.begin());
}