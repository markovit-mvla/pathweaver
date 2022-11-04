#include "PathWeaver.h"

/**
* @todo
* Consider skidding
* Consider acceleration when following path
*/

PathWeaver::PathWeaver()
{
    /* Can change x to determine how much of an arc the path is */
    double x = 0.0;
    PATH_TYPE path_ = &purePursuit(SwerveDrive::getDistance(SwerveDrive::getRobotGoalAng()) / x);
    /* Add path to network table */
    frc::SmartDashboard::PutData("Path", path_);
}

/**
* @param lookaheadDistance
* @return Closest Arc Path to Goal
* Determine path to take toward goal
* Dependent on lookaheadDistance
*   Bigger lookaheadDistance - More complex path
*   Smaller lookaheadDistance - More straight, basic path
*/
PATH_TYPE* PathWeaver::purePursuit(double lookaheadDistance)
{
    /* Current coordinate */
    Coordinate{ SwerveDrive::getX(), SwerveDrive::getY() } curr;

    /* Current angle to goal */
    Heading heading = SwerveDrive::getRobotGoalAng();

    /* Next coordinate */
    Coordinate{ curr.first + sin(heading) * lookaheadDistance,
        curr.second + cos(heading) * lookaheadDistance } nextCoord;

    /* Path to take */
    PATH_TYPE{ 
        nextCoord,
        heading,
        /* (2*pi*r) / theta to obtain curvature */
        (2 * (M_PI * ((lookaheadDistance * lookaheadDistance)/(2 * curr.first)))
            / (heading * 180 / M_PI),
        lookaheadDistance,
    } closestPath;

    /* Return closest path */
    return *closestPath;
}
