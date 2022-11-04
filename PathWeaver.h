#pragma once

#include "src/main/include/SwerveDrive.h"
#include <frc/MathUtil.h>

typedef std::pair<double, double> Coordinate;
typedef double Heading;

struct PATH_TYPE 
{
    Coordinate coord;
    Heading heading;
    double speeds;
    double curvature;
    double distance;
};

class PathWeaver()
{
    public:
        PathWeaver();
        PATH_TYPE* purePursuit(double lookaheadDistance);
};