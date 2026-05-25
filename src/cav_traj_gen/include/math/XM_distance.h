

// common math
// XMath.h


#ifndef XM_DISTANCE_H
#define XM_DISTANCE_H

#include <math.h>
#include <stdio.h>
#include <cstddef>
#include <iostream>
#include <vector>

#include "XM_base.h"


namespace XM
{

    inline double distFromXY(double x, double y, double x2,double y2)
    {
        return sqrt(SQR(x2-x)+SQR(y2-y));
    }

    template <typename T>
    static inline double distFromXY(T p1, T p2)
    {
        return distFromXY(p1.x, p1.y, p2.x, p2.y);
    }



    inline void distFromLine(double x, double y,
                   double ax, double ay ,
                   double bx, double by,
                   double & distanceSegment,
                   double & distanceLine)
    {
        // find the distance from the point (x,y) to the line
        // determined by the points (ax,ay) and (bx,by)
        //
        // distanceSegment = distance from the point to the line segment
        // distanceLine = distance from the point to the line
        //          (assuming infinite extent in both directions)
        //
        // copied from http://www.codeguru.com/forum/printthread.php?t=194400

        double r_numerator = (x-ax)*(bx-ax) + (y-ay)*(by-ay);
        double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
        double r = r_numerator / r_denomenator;

        double s =  ((ay-y)*(bx-ax)-(ax-x)*(by-ay) ) / r_denomenator;

        distanceLine = fabs(s)*sqrtf(r_denomenator);

        if ( (r >= 0) && (r <= 1) )
        {
          distanceSegment = distanceLine;
        }
        else
        {
            double dist1 = (x-ax)*(x-ax) + (y-ay)*(y-ay);
            double dist2 = (x-bx)*(x-bx) + (y-by)*(y-by);

            if (dist1 < dist2)
                distanceSegment = sqrtf(dist1);
            else
                distanceSegment = sqrtf(dist2);
        }
    }

    template <typename T>
    static inline void distFromLine(T p, T p1, T p2,
                   double & distanceSegment,
                   double & distanceLine)
    {
        distFromLine(p.x, p.y,
               p1.x, p1.y, p2.x, p2.y,
               distanceSegment, distanceLine);
    }


    inline double dist2LineWithSign(double x, double y, double x1,double y1,double x2,double y2)
    {
        double A = y2-y1;
        double B = x1-x2;
        double C = (x2-x1)*y1-(y2-y1)*x1;
        double distance = fabs(A*x+B*y+C)/sqrt(A*A+B*B);
        double L = (y-y1)*(x2-x1) - (y2-y1)*(x-x1);
        if( L < 0 ) distance = - distance;
        return distance;
    }
}

#endif // XM_DISTANCE_H