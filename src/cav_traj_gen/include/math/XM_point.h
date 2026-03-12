// common math
// XMath.h


#ifndef X_POINT_H
#define X_POINT_H

#include <math.h>
#include <stdio.h>
#include <cstddef>
#include <iostream>


class X_Point
{


public:
    double x = 0;
    double y = 0;
    double z = 0;

    double t = 0; //time
    double heading = 0;
    double v = 0;
    double cr = 0;
    double mileage = 0;

    double glat=0;
    double glon=0;

    double bound_left;
    double bound_right;

    X_Point()
    {
    }
    X_Point(double x_, double y_, double z_)
    {x =x_, y=y_, z=z_;}

    void set(double x_, double y_, double z_)
    {x =x_, y=y_, z=z_;}

    double dist2point(X_Point * pt)
    {
        if (pt==NULL) return 0;
        return sqrt(SQR(x-pt->x) + SQR(y-pt->y) + SQR(z-pt->z) );
    }

    double dist2XY(double xin, double yin)
    {
        double d = sqrt(SQR(x-xin)+SQR(y-yin));
        return d;
    }

    X_Point operator-(X_Point &p)
    {
        X_Point xp;

        xp.x = x-p.x;
        xp.y = y-p.y;
        xp.z = z-p.z;

        xp.t = t;//t-p.t;
        xp.heading = heading-p.heading;
        xp.v = v-p.v;
        xp.cr = cr-p.cr;
        xp.mileage = mileage-p.mileage;

        xp.glat = glat-p.glat;
        xp.glon = glon-p.glon;
        return xp;
    }
    void print()
    {
    	printf("Lat:%.7f, Lon:%.7f x：%.3f y：%.3f H:%.3f v:%.1f "
               "mileage=%.1f cr:%.4f time:%.4f",
               glat, glon, x, y, heading, v,
               mileage, cr, t);
    }


};


#endif // X_POINT_H
