
// basic 3d drawing
// draw_3d_base.h

#ifndef DRAW_3D_BASE_H
#define DRAW_3D_BASE_H

#include <iostream> 
#include <math.h> 
#include <unistd.h>
#include <stdbool.h>
#include <random>

#include <string>
#include <cstring>


#include <QGLWidget>
#include <QTextStream>
#include <QKeyEvent>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <QMessageBox>
#include <QApplication>

#include "XMath.h"

namespace D3D
{
    inline void drawOneTriangle(double x,double y,double h,double hg,double size, double level, bool filled=true)
    {
        if (filled) glBegin(GL_TRIANGLES);
        else glBegin(GL_LINE_LOOP);
        glVertex3f( x, y, level);
        glVertex3f( x+size*cos(h+hg), y+size*sin(h+hg), level);
        glVertex3f( x+size*cos(h-hg), y+size*sin(h-hg), level);
        glEnd();
    }


    inline void drawOnePT(double x1, double y1, double h1,
                          double x2, double y2, double h2 ,
                          double size, double level)
    {
        double cosv, sinv;
        glBegin(GL_QUADS);
            cosv = cos(h1 - PI*0.5);
            sinv = sin(h1 - PI*0.5);
            glVertex3f( x1 + size*cosv, y1 + size*sinv, level);
            glVertex3f( x1 - size*cosv, y1 - size*sinv, level);

            
            cosv = cos(h2 - PI*0.5);
            sinv = sin(h2 - PI*0.5);
            glVertex3f( x2 - size*cosv, y2 - size*sinv, level);
            glVertex3f( x2 + size*cosv, y2 + size*sinv, level);
        glEnd();

    }

    inline void drawOnePT(double x, double y, double size_x, double size_y, double level)
    {
        double dx = size_x*0.5, dy=size_y*0.5;
        glBegin(GL_QUADS);
            glVertex3f( x + dx, y + dy, level);
            glVertex3f( x + dx, y - dy, level);
            glVertex3f( x - dx, y - dy, level);
            glVertex3f( x - dx, y + dy, level);
        glEnd();

    }


    inline void drawOneCircle(double x, double y, double r, int num, double level, bool filled)
    {
        double theta;
        double delta_angle = 2.0f*M_PI/num;
        if (filled) glBegin(GL_POLYGON);
        else glBegin(GL_LINE_LOOP);

        for (int ii = 0; ii < num; ++ii)
        {                   
            theta =  delta_angle*ii;
            glVertex3f(r * cos(theta) + x, r * sin(theta) + y, level);
        }
        glEnd();
    }


    template <typename T>
    void drawTraj_points(std::vector<T> * vec,
                    double r, double g, double b,
                    int gap, double level, double size, bool filled, double line_width)
    {
        if(r>=0) glColor3f(r, g, b);
        glLineWidth(line_width);
        if (filled) glBegin(GL_QUADS);
        else glBegin(GL_LINE_LOOP);
        for (int i = 0; i < vec->size(); i+=gap)
        {
            auto p = &vec->at(i);
            glVertex3f(p->x+size, p->y+size, level);
            glVertex3f(p->x+size, p->y-size, level);
            glVertex3f(p->x-size, p->y-size, level);
            glVertex3f(p->x-size, p->y+size, level);
        }
        glEnd();
    }

    template <typename T>
    void drawTraj_line(std::vector<T> * vec,
                    double r, double g, double b,
                    int gap, double level, double line_width)
    {
        if(r>=0) glColor3f(r, g, b);
        if(line_width>0) glLineWidth(line_width);
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < vec->size(); i+=gap)
        {
            auto p = &vec->at(i);
            glVertex3f(p->x, p->y, level);
        }
        glEnd();
    }

    template <typename T>
    void drawTraj_heading(std::vector<T> * vec,
                    double r, double g, double b,
                    int gap, double level, double line_width)
    {
        if(r>=0) glColor3f(r, g, b);
        glLineWidth(line_width);
        glBegin(GL_LINES);
        for (int i = 0; i < vec->size(); i+=gap)
        {
            auto p = &vec->at(i);
            glVertex3f( p->x, p->y, level);
            glVertex3f( p->x + 5 * cos(p->heading), 
                        p->y + 5 * sin(p->heading), level);
        }
        glEnd();
    }

    template <typename T>
    void drawTraj_speed(std::vector<T> * vec,
                    double r, double g, double b,
                    int gap, double level, double line_width)
    {
        if(r>=0) glColor3f(r, g, b);
        glLineWidth(line_width);
        glBegin(GL_LINES);
        for (int i = 0; i < vec->size(); i+=gap)
        {
            auto p = &vec->at(i);
            glVertex3f( p->x, p->y, level);
            glVertex3f( p->x + p->x_d*5 * cos(p->heading + PI*0.5), 
                        p->y + p->x_d*5 * sin(p->heading + PI*0.5), level);
        }
        glEnd();
    }

    template <typename T>
    void drawTraj_curvature(std::vector<T> * vec,
                    double r, double g, double b,
                    int gap, double level, double line_width)
    {
        if(r>=0) glColor3f(r, g, b);
        glLineWidth(line_width);
        glBegin(GL_LINES);
        for (int i = 0; i < vec->size(); i+=gap)
        {
            auto p = &vec->at(i);
            glVertex3f( p->x, p->y, level);
            glVertex3f( p->x + p->cr*50 * cos(p->heading + PI*0.5), 
                        p->y + p->cr*50 * sin(p->heading + PI*0.5),level);
        }
        glEnd();
    }

    template <typename T>
    void drawTraj_bound(std::vector<T> * vec,
                    double r, double g, double b,
                    int gap, double level, double line_width,  
                    bool en_left, bool en_right)
    {
        if(r>=0) glColor3f(r, g, b);
        glLineWidth(line_width);

        //left
        if (en_left)
        {
            glBegin(GL_LINE_STRIP);
            for (int i = 0; i < vec->size(); i+=gap)
            {
                auto p = &vec->at(i);
                glVertex3f( p->x + p->bound_left*cos(PI*0.5+p->heading), 
                            p->y + p->bound_left*sin(PI*0.5+p->heading), level);
            }
            glEnd();
        }

        //right
        if (en_right)
        {
            glBegin(GL_LINE_STRIP);
            for (int i = 0; i < vec->size(); i+=gap)
            {
                auto p = &vec->at(i);
                glVertex3f( p->x + p->bound_right*cos(-PI*0.5+p->heading), 
                            p->y + p->bound_right*sin(-PI*0.5+p->heading), level);
            }
            glEnd();
        }
    }

    inline void drawOne3DObj_cylinder(
        double cx, double cy, double cz,
        double radius, double height,
        double r, double g, double b, double alp)
    {
        const int n = 32; // 分段数，控制圆形的平滑度
        double rd = (long(cx * cy) % 100) * 0.003 + 0.7;

        glLineWidth(0.1);
        
        // 绘制顶面圆盘
        glBegin(GL_TRIANGLE_FAN);
        glColor4f(r, g, b, alp);
        glNormal3d(0, 0, 1); // 法线朝上
        glVertex3f(cx, cy, cz+ height/2); // 圆心
        for (int i = 0; i <= n; ++i) {
            double theta = 2 * PI * i / n;
            double x = cx + radius * cos(theta);
            double y = cy + radius * sin(theta);
            glVertex3f(x,y,cz + height/2);
        }
        glEnd();

        // 绘制底面圆盘
        glBegin(GL_TRIANGLE_FAN);
        glColor4f(r, g, b, alp);
        glNormal3d(0, 0, -1); // 法线朝下
        glVertex3f(cx, cy, cz- height/2); // 圆心
        for (int i = n; i >= 0; --i) { // 反向顶点顺序以保证法线正确
            double theta = 2 * PI * i / n;
            double x = cx + radius * cos(theta);
            double y = cy + radius * sin(theta);
            glVertex3f(x, y, cz - height/2);
        }
        glEnd();

        // 绘制侧面
        glBegin(GL_QUAD_STRIP);
        glColor4f(r * rd, g * rd, b * rd, alp); // 侧面颜色调整
        for (int i = 0; i <= n; ++i) {
            double theta = 2 * PI * i / n;
            double cosTheta = cos(theta);
            double sinTheta = sin(theta);
            double x = cx + radius * cosTheta;
            double y = cy + radius * sinTheta;
            
            // 设置法线（沿径向方向）
            glNormal3d(cosTheta, sinTheta,0 );
            // 侧面四边形顶点
            glVertex3f(x, y, cz + height/2); // 顶边点
            glVertex3f(x, y, cz - height/2); // 底边点
        }
        glEnd();
    }
}
#endif //DRAW_3D_BASE_H
