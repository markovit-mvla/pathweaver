#pragma once

#include "src/main/include/SwerveDrive.h"
#include <frc/MathUtil.h>

typedef std::pair<double, double> Coordinate;
typedef double Heading;

#define M_PI 3.14159265358979323846

struct PATH_TYPE 
{
    Coordinate coord;
    Heading heading;
    double curvature;
    double distance;
};

class PathWeaver()
{
    public:
        PathWeaver();
        PATH_TYPE* purePursuit(double lookaheadDistance);
};



