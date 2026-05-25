/*
 * display_3d.h
 * We use openGL to display the 3D simulation environment.
 * This file contains the class definition for the 3D display system.
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE** 
*/
#ifndef DISPLAY_3D_H
#define DISPLAY_3D_H

#include <QGLWidget>
#include <QTextStream>
#include <QKeyEvent>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <QMessageBox>
#include <QApplication>

#include "gui.h"
#include "dataDefine.hpp"
#include "globalDef.hpp"
#include "draw_3d_base.h"
#include "objloader.h"

/**
 * @class DISPLAY_3D
 * @brief 3D visualization system for autonomous driving simulation
 * 
 * Handles OpenGL rendering pipeline, vehicle model display, and multiple view modes.
 * Integrates with Qt framework for GUI interaction.
 */
class DISPLAY_3D : public QGLWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructs 3D visualization context
     * @param parent Parent widget in Qt's object hierarchy
     */
    explicit DISPLAY_3D(QWidget *parent = 0);
    
    ~DISPLAY_3D();

    void showAxis();

    void iniView();

    void draw_mode1();

    void draw_mode2();

protected:

    void resizeGL(int w, int h) override;

    void loadGLTexture();

    void initializeGL();

    void paintGL();

    void drawOnePath(Trajectory_S *const traj);

    void drawObstacle(std::vector<Obstacle_S> *const obstVec);


    void drawObstacle_3d(std::vector<Obstacle_S> *const obstVec);

    template <typename T>
    void drawTraj_Alp(std::vector<T> *const vec,
                    double r, double g, double b, double alpha,
                    int gap, double width, double level, bool en_bound);

    void genPastPath();

    void drawVehicle();

    template <typename T>
    void drawTraj(std::vector<T> *const p,
                  double r, double g, double b,
                  int gap, double line_width, double level,
                  bool flag_point, bool flag_line,
                  bool flag_heading, bool flag_cr, bool flag_bound);

private:
    /// @brief Lexus vehicle 3D model loader
    ObjLoader lexus_obj;

    /// @brief Dynamic light source position (X,Y,Z,W)
    /// @note W=0: Directional light, W=1: Positional light
    GLfloat lightpos[4];

    /// @brief Frame counter for animation timing
    unsigned long count = 0;

    /// @brief Vehicle X position in world coordinates (meters)
    double move_x = 0;

    /// @brief Vehicle Y position in world coordinates (meters)
    double move_y = 0;

    /// @brief Vehicle heading angle (radians)
    double move_heading = 0;

    /// @brief Historical path storage (last 500 points)
    /// @details Each point contains X/Y position and heading
    std::vector<X_Point> pastPath;
};

#endif