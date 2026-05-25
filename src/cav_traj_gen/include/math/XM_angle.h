

// common math
// XMath.h


#ifndef XM_ANGLE_H
#define XM_ANGLE_H

#include <math.h>
#include <stdio.h>
#include <cstddef>
#include <iostream>
#include <vector>

#include "XM_define.h"



namespace XM
{


    inline double Normalise_PI(double angle)
    {
        while (angle>PI)
            angle -= 2.0*PI;

        while (angle<=-PI)
            angle += 2.0*PI;

        return angle;
    }


    inline double Normalise_2PI(double angle)
    {
        while (angle >= 2.0*PI)
            angle -= 2.0*PI;

        while (angle < 0.0f)
            angle += 2.0*PI;

        return angle;
    }


    inline double meanAngle_weighted(double angle1, double angle2, double w1, double w2)
    {
        double y_part = 0, x_part = 0;
        x_part = cos(angle1)*w1 + cos(angle2)*w2;
        y_part = sin(angle1)*w1 + sin(angle2)*w2;
 
     
        return atan2(y_part, x_part);
    }

    
    inline double ABS_Angle_Dist(double angle1, double angle2)
    {
        angle1 = Normalise_PI(angle1);
        angle2 = Normalise_PI(angle2);
        return ABS(Normalise_PI(angle1-angle2));
    }

    inline double AngleFrom_dX_dY(double dx,double dy)
    {
        return Normalise_PI( atan2(dy,dx) );
    }


    inline double AngleFromXY(double x,double y, double ori, double x2, double y2)
    {
        return Normalise_PI(atan2(y2-y,x2-x) - ori);
    }


    template <typename T>
    inline double angle_3_point(T P3, T P1, T P2) // angle p3-p1-p2 
    {
        double angle = atan2(P3.y - P1.y, P3.x - P1.x) - atan2(P2.y - P1.y, P2.x - P1.x);
        return ABS(Normalise_PI(angle));
    }
    
    inline double approx_atan(double z)
    {
        //const double n1 = 0.97239411f;
        //const double n2 = -0.19194795f;
        //return (n1 + n2*z*z)*z;
        return 0.25*PI*z - z*(fabs(z) - 1)*(0.2447 + 0.0663*fabs(z));
    }

    inline double approx_atan2(double y, double x)
    {
        if (x != 0.0f)
        {
            if (fabsf(x) > fabsf(y))
            {
                const double z = y / x;

                if (x > 0.0) return approx_atan(z); // atan2(y,x) = atan(y/x) if x > 0
                else if (y >= 0.0) return approx_atan(z) + PI; // atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0
                else return approx_atan(z) - PI; // atan2(y,x) = atan(y/x) - PI if x < 0, y < 0
            }
            else // Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1.
            {
                const double z = x / y;
                if (y > 0.0) return -approx_atan(z) + HALFPI; // atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0
                else return -approx_atan(z) - HALFPI; // atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0
            }
        }
        else
        {
            if (y > 0.0f) return HALFPI; // x = 0, y > 0
            else if (y < 0.0f) return -HALFPI; // x = 0, y < 0 
        }

        return 0.0f; // x,y = 0. Could return NaN instead.
    }

}

#endif // XM_ANGLE_H