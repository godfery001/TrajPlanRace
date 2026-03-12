
#ifndef XM_DEFINE_H
#define XM_DEFINE_H

#include <math.h>
#include <stdio.h>
#include <cstddef>
#include <iostream>
#include <vector>


#define PI      (3.14159265)
#define PI_L    (3141592L)

#define TWOPI   (6.2831853)
#define HALFPI  (1.5707963)

#define DEG_TO_RAD(x)       ((double)((x)*PI)/180.0)
#define RAD_TO_DEG(x)       ((double)((x)*180.0)/PI)

#define RAD_TO_MICRO(x)     ((long)((x)*1000000))
#define MICRO_TO_RAD(x)     ((double)(x)/1000000.0)

#define DEG_TO_MICRO(x)     (RAD_TO_MICRO((DEG_TO_RAD((x)))))
#define MICRO_TO_DEG(x)     (RAD_TO_DEG((MICRO_TO_RAD((x)))))

#define DEG_T_RAD           (PI/180.0)
#define RAD_T_DEG           (180.0/PI)

#define SQR(x)              ((x) * (x))
#define CUBE(x)             ((x) * (x)* (x))
#define ABS(x)              (((x)>0)?(x):-1*(x))

#define MAX(x,y)            (((x) > (y)) ? (x) : (y))
#define MIN(x,y)            (((x) > (y)) ? (y) : (x))
#define SIGN(x)             ((x > 0) ? 1: ((x<0) ? -1 : 0))

#define POW_0(x)            (1.0)
#define POW_1(x)            (x)
#define POW_2(x)            ( (x)*(x) )
#define POW_3(x)            ( (x)*(x)*(x) )
#define POW_4(x)            ( (x)*(x)*(x)*(x) )
#define POW_5(x)            ( (x)*(x)*(x)*(x)*(x) )
#define POW_6(x)            ( (x)*(x)*(x)*(x)*(x)*(x) )



#endif // XM_BASE_H