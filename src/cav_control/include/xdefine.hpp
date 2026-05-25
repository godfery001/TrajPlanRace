
#ifndef XDEFINE_HPP
#define XDEFINE_HPP
#define _USE_MATH_DEFINES

#include <iostream> 
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

//define gear, keep the same
#define GEAR_NONE (0)
#define GEAR_PARK (1)
#define GEAR_REVERSE (2)
#define GEAR_NEUTRAL (3)
#define GEAR_DRIVE (4)
#define GEAR_LOW (5)


// define Estop
#define ESTOP_NONE (0)
#define ESTOP_LOW (1)
#define ESTOP_MEDIUM (2)
#define ESTOP_HIGH (3)


#define MAX(x,y)    (((x) > (y)) ? (x) : (y))
#define MIN(x,y)    (((x) > (y)) ? (y) : (x))
#define ABS(x)      (((x)>0)?(x):(0-(x)))
#define SQR(x)      ((x) * (x))


namespace XM
{

    inline double Normalise_PI(double angle)
    {
        while (angle>M_PI)
            angle -= 2.0*M_PI;

        while (angle<=-M_PI)
            angle += 2.0*M_PI;

        return angle;
    }


    inline double Normalise_2PI(double angle)
    {
        while (angle >= 2.0*M_PI)
            angle -= 2.0*M_PI;

        while (angle < 0.0f)
            angle += 2.0*M_PI;

        return angle;
    }


    inline double AngleFromXY(double x,double y, double ori, double x2, double y2)
    {
        return Normalise_PI(atan2(y2-y,x2-x) - ori);
    }

    inline double distFromXY(double x, double y, double x2,double y2)
    {
        return sqrt((x2-x)*(x2-x)+(y2-y)*(y2-y));
    }

    template <typename T>
    static inline void update_dist(T * vec)
    {
        //ini
        if (vec==NULL || vec->size() < 1) return;

        //cal distance
        vec->at(0).mileage = 0;
        for (int i = 1; i < vec->size(); ++i)
            vec->at(i).mileage = vec->at(i-1).mileage 
                            + vec->at(i).dist2point(& vec->at(i-1));
    }


    inline double dist2LineWithSign(double x, double y, 
        double x1,double y1,double x2,double y2)
    {
        double A = y2-y1;
        double B = x1-x2;
        double C = (x2-x1)*y1-(y2-y1)*x1;
        double distance = fabs(A*x+B*y+C)/sqrt(A*A+B*B);
        double L = (y-y1)*(x2-x1) - (y2-y1)*(x-x1);
        if( L < 0 ) distance = - distance;
        return distance;
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
    static inline void find_NPN(T const * vec, 
        double x, double y, int & i_tg, double & d_min)
    {
        if (vec == NULL || vec->size() < 1) return;
        int N = vec->size();

        //only one point
        if (N==1)
        {
            i_tg = 0;
            return;
        }

        //more than one point
        d_min = 1e6;
        i_tg = -1;
        double d_seg, d_line;
        for (int i = 0; i < vec->size()-1; ++i)
        {
            XM::distFromLine( x, y,
                vec->at(i).x, vec->at(i).y,
                vec->at(i+1).x, vec->at(i+1).y,
                d_seg, d_line);

            if (d_seg<= d_min && ABS(d_seg - d_line) < 1.0)
            {
                d_min = d_seg;
                i_tg = i;
            }
        }

        return;
    }
}


// vehicle state
class VehState_S
{
public:
    double timestamp = 0;
    std::string RTK_state_string = "";

    double latitude = 42.2995541 * M_PI / 180.0f;
    double longitude = -83.6975026 * M_PI / 180.0f;

    double x = 0;
    double y = 0;
    double z = 0;
    double heading = 0;
    double yaw_rate = 0;
    double speed_x = 0;
    double speed_y = 0;
    double acc_x = 0;
    double acc_y = 0;


    bool by_wire_enabled = false;
    double steer_state_front_wheel = 0;

    double brake_state = 0;
    double throttle_state = 0;
    int gear_pos = 0;

    double size_x = 5.0;
    double size_y = 2.0;
    double size_z = 1.75;
    double wheelbase = 0;
};



// output control commands
class Control_Value_S
{
public:
    double steering = 0.0;
    double speed_cmd = 0;

    int gear = GEAR_NONE;
    int turn_signal = 0;
};


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

    double dist2point(X_Point * pt)
    {
        if (pt==NULL) return 0;
        return sqrt(SQR(x-pt->x) + SQR(y-pt->y) + SQR(z-pt->z) );
    }
};

// path planning and decision results
class Plan_Rlt_S
{
public:
    Plan_Rlt_S(){ini();};

    double timestamp = 0;

    int Estop = ESTOP_NONE;
    bool Go = false;

    double vd = 0.0f;
    double ey = 0.0f;
    double ephi = 0.0f;
    double len = 0.0f;
    double cr = 0.0f;

    //path
    int NPN = -1;
    std::vector<X_Point> path;

    void ini()
    {
        Estop = 0;
        Go = 0;
        vd = 0.0f;
        cr = 0.0f;
    }
};

#endif //XDEFINE_HPP
