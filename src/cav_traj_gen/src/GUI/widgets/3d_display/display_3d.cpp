/*
 * display_3d.cpp
 * Some basic 3d display dunctions are defined here.
 * If you want to revise the display, go to display_3d_mode1.cpp and display_3d_mode2.cpp
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE**
*/
#include <math.h>
#include <stdio.h>
#include "display_3d.h"

DISPLAY_3D::DISPLAY_3D(QWidget *parent) : QGLWidget(parent)
{
    showNormal();
}

DISPLAY_3D::~DISPLAY_3D()
{
}

/**
 * Loads 3D vehicle model and texture assets
 * - Model path: /opt/cav_deploy/obj_model/Lexus/lexus_hs.obj
 * - Texture path: /opt/cav_deploy/obj_model/Lexus/Lexus.jpg
 * - Original scaling factor: 1 (no scaling)
 */
void DISPLAY_3D::loadGLTexture()
{
    lexus_obj.Load("/opt/cav_deploy/obj_model/Lexus/lexus_hs.obj",
                    "/opt/cav_deploy/obj_model/Lexus/Lexus.jpg");
    lexus_obj.resize(1);
}

/**
 * Handles viewport resizing events
 * @param w New width in pixels
 * @param h New height in pixels
 * - Maintains 45 degree vertical FOV
 * - Near clip plane: 0.5 units
 * - Far clip plane: 1000 units
 */
void DISPLAY_3D::resizeGL(int w, int h)
{
    if (h == 0)
        h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLfloat)w / (GLfloat)h, 0.5f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/**
 * Initializes OpenGL rendering context
 * - Background color: RGB(1,5,43)
 * - Enables depth testing and smooth shading
 * - Configures two light sources:
 *   1. Directional light (Light 0) at (0,100,1000)
 *   2. Positional light (Light 1) following camera
 */
void DISPLAY_3D::initializeGL()
{
    glShadeModel(GL_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glClearColor(181.0 / 255.0f, 227.0 / 255.0f, 229.0 / 255.0f, 0.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    GLfloat mLightPosition[4] = {-10.0f, 0.0f, 15.0f, 1.0f};
    glLightfv(GL_LIGHT1, GL_POSITION, mLightPosition);

    loadGLTexture();

    // Configure directional light (Light 0)
    GLfloat sunlightpos[] = {0, 100, 1000.f, 0.0};
    glLightfv(GL_LIGHT0, GL_POSITION, sunlightpos);
    
    GLfloat light_ambient[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);   
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

    // Configure positional light (Light 1)
    GLfloat lightpos[] = {0, 0, 20, 0.};
    glLightfv(GL_LIGHT1, GL_POSITION, lightpos);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
}

/**
 * Main rendering loop
 * - Updates frame counter (count)
 * - Configures camera view (iniView)
 * - Updates path history (genPastPath)
 * - Delegates to current view mode renderer
 */
void DISPLAY_3D::paintGL()
{
    count++;
    iniView();
    genPastPath();
    
    if (guiSet.view_mode==1) 
        draw_mode1();
    else 
        draw_mode2();
}

/**
 * Configures camera view parameters
 * - View mode 1: Bird's-eye view 8m behind vehicle
 * - View mode 2: Driver view 8m ahead of vehicle
 * - Updates light position (lightpos) based on view mode
 */
void DISPLAY_3D::iniView()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    GLfloat aspect = static_cast<GLfloat>(width()) / static_cast<GLfloat>(height());
    gluPerspective(80.0f, aspect, 0.5f, 500.0f);

    move_x = TG.ssData.goal.x;
    move_y = TG.ssData.goal.y;
    move_heading = TG.ssData.goal.heading;

    float dx=0, dy=0, dz=0;

    if (guiSet.view_mode==1) {
        dx = guiSet.x_gui + 8 *cos(move_heading);
        dy = guiSet.y_gui + 8 *sin(move_heading);
        dz = 15 + guiSet.scale_gui * 0.19;
        gluLookAt(move_x+dx, move_y+dy, dz, move_x+dx, move_y+dy, 1.0, 0.0f, 1.0f, 0.0f);
        lightpos[0] = dx;
        lightpos[1] = dy;
        lightpos[2] = dz;
        lightpos[3] = 0.0;
    }

    if (guiSet.view_mode==2) {
        dx = -8*cos(move_heading);
        dy = -8*sin(move_heading);
        dz = 7 + MAX(0, guiSet.scale_gui*2);
        gluLookAt(move_x + dx, move_y + dy, dz, move_x, move_y, 1.0, 0.0f, 0.0f, 1.0f);
        lightpos[0] = -8;
        lightpos[1] = 0;
        lightpos[2] = dz-1;
        lightpos[3] = 1.0;
    }

    glMatrixMode(GL_MODELVIEW);
}

/**
 * Renders 1m grid in X-Y plane
 * - Grid extends 150 units from current position
 * - Lines drawn at Z=-0.01 to avoid depth fighting
 */
void DISPLAY_3D::showAxis()
{
    double gl_resolution = 1.0;
    int N = int(150 / gl_resolution);

    double x = long(move_x/gl_resolution)*gl_resolution;
    double y = long(move_y/gl_resolution)*gl_resolution;

    glLineWidth(1.0);
    glColor3f(1, 1, 0.8);
    glBegin(GL_LINES);
    for (int i = -N; i <= N; ++i) {
        glVertex3f(-gl_resolution * N + x, i * gl_resolution + y, -0.01);
        glVertex3f(+gl_resolution * N + x, i * gl_resolution + y, -0.01);
        glVertex3f(i * gl_resolution + x, -gl_resolution * N + y, -0.01);
        glVertex3f(i * gl_resolution + x, +gl_resolution * N + y, -0.01);
    }
    glEnd();
}

/**
 * Renders ego vehicle model with lighting
 * - Applies current vehicle position (move_x, move_y)
 * - Rotates model according to heading (move_heading)
 * - Uses preloaded Lexus model dimensions from TG.ssData
 */
void DISPLAY_3D::drawVehicle()
{
    float mkz_x = move_x;
    float mkz_y = move_y;
    float mkz_h = move_heading * 180.0f / PI;

    glTranslatef(mkz_x, mkz_y, 0);
    glRotatef(mkz_h,0,0,1);

    glLightfv(GL_LIGHT1, GL_POSITION, lightpos);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT1);
    lexus_obj.Render_Texture(TG.ssData.goal.size_x, TG.ssData.goal.size_y, TG.ssData.goal.size_z);
    glDisable(GL_LIGHT1);
    glDisable(GL_LIGHTING);

    glRotatef(-mkz_h,0,0,1);
    glTranslatef(-mkz_x, -mkz_y, 0);
}