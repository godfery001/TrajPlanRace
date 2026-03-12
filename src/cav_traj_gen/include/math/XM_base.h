

// common math
// XMath.h


#ifndef XM_BASE_H
#define XM_BASE_H

#include <math.h>
#include <stdio.h>
#include <cstddef>
#include <iostream>
#include <vector>
#include "XM_define.h"
#include "XM_point.h"


namespace XM
{

 	// area of triangle
    static inline double area_3points(X_Point p1, X_Point p2, X_Point p3)
    {
        double dArea = ((p2.x - p1.x)*(p3.y - p1.y) - (p3.x - p1.x)*(p2.y - p1.y))*0.5;
        return (dArea > 0.0) ? dArea : -dArea;

    }

    
    // caculate curvature by three points
    static inline double radiusFun(double x1,double y1,double x2,double y2,double x3,double y3)
    {

        double dArea = ((x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1))/2.0;
 
        if (ABS(dArea) < 0.01) return 1e10;

        double m1x = (x2 + x1)/2;  
        double m1y = (y2 + y1)/2;
      
        double m2x = ( x3 + x1)/2;  
        double m2y = ( y3 + y1)/2;  
      
        double k1 = -(x2 - x1)/(y2 - y1);
        if ((y2 - y1) == 0) k1 = 1e10;

        double k2 = -(x3 - x1)/(y3 - y1);
        if ((y3 - y1) == 0) k2 = 1e10;
      
        double center_x = (m2y - m1y - k2*m2x + k1*m1x)/(k1 - k2);
        double center_y = m1y + k1*(m2y - m1y - k2*m2x + k2*m1x)/(k1-k2);
        double radius = sqrtf ( ((center_x - x1)*(center_x - x1) + (center_y - y1)*(center_y - y1)) );

        //determine positive or negative
        if (dArea < 0) radius = -radius;

        return radius;
    }
}

#endif // XM_BASE_H