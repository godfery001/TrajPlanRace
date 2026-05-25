

// common math
// XMath.h


#ifndef XM_PATH_H

#define XM_PATH_H


#include <math.h>
#include <stdio.h>
#include <cstddef>
#include <iostream>
#include <vector>

#include "XM_define.h"
#include "XM_point.h"
#include "XM_base.h"
#include "XM_distance.h"
#include "XM_angle.h"


namespace XM
{
    template <typename T, typename T1>
    void points_wighted(T1 &pt, T *p1, T *p2, 
        double w1, double w2, bool en_xy)
    {
        if (p1==NULL || p2==NULL) return;
        double k = w2/(w1+w2);

        if (en_xy)
        {
            pt.x = p1->x*(1-k) + p2->x*k;
            pt.y = p1->y*(1-k) + p2->y*k;
        }

        pt.heading = XM::meanAngle_weighted(p1->heading, p2->heading, 1-k, k);
        pt.t  = p1->t*(1-k)  + p2->t*k;
        pt.v  = p1->v*(1-k)  + p2->v*k;
        pt.cr = p1->cr*(1-k) + p2->cr*k;
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

    template <typename T>
    static inline int find_NPN(T const * vec, double x, double y)
    {
        if (vec == NULL || vec->size() < 1) return -1;
        int i_tg = -1;
        double d_min=1e6;
        XM::find_NPN(vec, x, y, i_tg, d_min);
        return i_tg;
    }


    template <typename T>
    static inline void find_NPN_preview(T const * vec, double x, double y,
        double heading, int & i_tg, double & d_min)
    {
        if (vec == NULL || vec->size() < 1) return;
        int N = vec->size();
		int start = MAX(0, i_tg);
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
        for (int i = start; i < vec->size()-1; ++i)
        {
            XM::distFromLine( x, y,
                vec->at(i).x, vec->at(i).y,
                vec->at(i+1).x, vec->at(i+1).y,
                d_seg, d_line);

            if (d_seg< d_min && ABS_Angle_Dist(vec->at(i).heading, heading)<M_PI*0.45 )
            {
                d_min = d_seg;
                i_tg = i;
            }
        }
        return;
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


    template <typename T>
    static inline void cal_heading_by_2pts(T * vec, double dist_gap, int mode)
    {
        //ini
        if (vec==NULL || vec->size() < 2) return;

        //cal distance
        update_dist(vec);

        //set the start and end point
        int N_end = -1;
        for (int i = vec->size()-1; i >=0; --i)
            if (vec->back().mileage - vec->at(i).mileage >= dist_gap)
                {N_end = i;break;}

        if (N_end<1 || vec->back().mileage<2.0 || vec->size()==2)
        {
            double heading = XM::AngleFrom_dX_dY(
                        vec->back().x - vec->at(0).x, 
                        vec->back().y - vec->at(0).y);

            //reset curvature
            for (int i = 0; i < vec->size(); ++i)
                vec->at(i).heading = heading;
            return;
        }

        //cal curvature
        double dx, dy;
        auto p2 = & vec->at(0);
        int Nmax = vec->size()-1;

        for (int i = 0; i <= N_end; ++i)
        {
            auto pi = &vec->at(i);

            p2 = &vec->back();
            for (int k = i; k <=Nmax; ++k)
                if (vec->at(k).mileage-pi->mileage >= dist_gap)
                    { p2 = &vec->at(k); break;}

            dx = p2->x - pi->x;
            dy = p2->y - pi->y;
            if (mode==1) pi->heading = approx_atan2(dy, dx); //use approximation, but faster
            if (mode==2) pi->heading = XM::AngleFrom_dX_dY(dx, dy); //use atan
        }

        for (int i = N_end; i <= Nmax; ++i)
            vec->at(i).heading = vec->at(N_end).heading;
    }


    

    template <typename T>
    static inline void cal_curvature_x(T * vec, double dist_gap, double min_R, 
            bool & flag_forward, bool & flag_R_greater_than_min)
    {
        //ini
        if (vec==NULL) return;

        for (int i = 0; i < vec->size(); ++i)
            vec->at(i).cr = 0;

        if (vec->size() < 3)
            return;


        //cal distance
        update_dist(vec);

        double R=0;
        int N1, N2, Nmax = vec->size()-1;

        //set the start and end point
        int N_start = -1, N_end = -1;
        for (int i = 0; i <=Nmax ; ++i)
            if (vec->at(i).mileage >= dist_gap)
                {N_start = i;break;}

        for (int i = Nmax; i >=0; --i)
            if (vec->back().mileage - vec->at(i).mileage >= dist_gap)
                {N_end = i;break;}

        if (N_start < 0|| N_end<0 || N_start >= N_end || 
            vec->back().mileage<0.5 || vec->size()==3) 
        {
            double m = MAX(1, int(Nmax*0.5 + 0.4));
            
            R = XM::radiusFun(
                vec->at(0).x, vec->at(0).y,
                vec->at(m).x, vec->at(m).y,
                vec->back().x, vec->back().y);

            for (int i = 0; i < vec->size(); ++i)
                vec->at(i).cr = MAX(-1./min_R, MIN(1./min_R, 1.0f/R) );
            return;
        }


        //cal curvature
        for (int i = N_start; i <= N_end; ++i)
        {
            auto pi = &vec->at(i);

            N1=0, N2=Nmax;
            for (int k = i; k >=0; --k)
                if (pi->mileage-vec->at(k).mileage >= dist_gap)
                    { N1 = k; break;}

            for (int k = i; k <=Nmax; ++k)
                if (vec->at(k).mileage-pi->mileage >= dist_gap)
                    { N2 = k; break;}

            double angle = XM::angle_3_point(vec->at(N1), vec->at(i ), vec->at(N2)); // angle p3-p1-p2
            if (angle < M_PI*0.55)//about 110 degrees
                flag_forward = false;

            R = XM::radiusFun(
                vec->at(N1).x, vec->at(N1).y,
                vec->at(i ).x, vec->at(i ).y,
                vec->at(N2).x, vec->at(N2).y);

            if (ABS(R) <min_R)
                flag_R_greater_than_min = false;

            pi->cr = MAX(-1./min_R, MIN(1./min_R, 1.0f/R) );
        }

        for (int i = 0; i < N_start; ++i)
            vec->at(i).cr = vec->at(N_start).cr;

        for (int i = N_end; i <= Nmax; ++i)
            vec->at(i).cr = vec->at(N_end).cr;

        return;
    }
}
#endif // XM_PATH_H
