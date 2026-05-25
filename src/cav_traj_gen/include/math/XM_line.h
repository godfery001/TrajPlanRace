

// common math
// XMath.h


#ifndef XM_LINE_H
#define XM_LINE_H

#include <math.h>
#include <stdio.h>
#include <cstddef>
#include <iostream>
#include <vector>

#include "XM_define.h"
#include "XM_point.h"

namespace XM
{
    inline double dot(X_Point c1, X_Point c2)
    {
        return (c1.x * c2.x + c1.y * c2.y + c1.z * c2.z);
    }

     
    inline double norm(X_Point c1)
    {
        return sqrt(dot(c1, c1));
    }
    
}

#endif // XM_LINE_H