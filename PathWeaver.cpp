#include "PathWeaver.h"

/**
* @todo
* Consider skidding
* Make the arc velocity model more realistic...
*   Uses uniform circular motion because that's all I know so far
*   from AP Physics 1 :)
*/

PathWeaver::PathWeaver()
{
    /* Can change x to determine how much of an arc the path is */
    auto x = 1.0;
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
    
    /* Radius */
    auto radius = (lookaheadDistance * lookaheadDistance)/(2 * curr.first)

    /* Current angle to goal */
    Heading heading = SwerveDrive::getRobotGoalAng();

    /* Next coordinate */
    Coordinate{ curr.first + sin(heading) * lookaheadDistance,
        curr.second + cos(heading) * lookaheadDistance } nextCoord;

    /* Current velocity */
    auto vx = SwerveDrive::getRobotSpeeds.vx;
    auto vy = SwerveDrive::getRobotSpeeds.vy;
    auto speed = sqrt(vx*vx + vy*vy);

    /* Current acceleration */
    auto acceleration = speed / 1_s;

    /* Velocity to take the path to the next point */
    auto arcSpeed = sqrt(acceleration * radius);

    /* Path to take */
    PATH_TYPE{ 
        /* New coordinate */
        nextCoord,
        /* Heading to goal */
        heading,
        /* Velocity to take on path to point */
        arcSpeed,
        /* rtheta to obtain curvature */
        radius * heading * (180.0 / M_PI),
        /* Distance to next path point */
        lookaheadDistance,
    } closestPath;

    /* Return closest path */
    return *closestPath;
}