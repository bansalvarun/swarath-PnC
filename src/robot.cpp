/************************************
* Robot description implementation  *
*    for rabbit follow algorithm    *
* Written for SWARATH Project       *
* @author Nishant Sharma            *
* @version 0.0                      *
* @date 4, March, 2015              *
************************************/

#include <rabbit_follow/robot.h>

/** constructors **/
Robot::Robot()
{
    this->setXLocation(0);
    this->setYLocation(0);
    this->setDirection(0);
    this->setState(followingCarrot);
    this->setCarrotXLocation(0);
    this->setCarrotYLocation(0);
}

Robot::Robot(float x, float y, float radian, State state)
{
    this->setXLocation(x);
    this->setYLocation(y);
    this->setDirection(radian);
    this->setState(state);
    this->setCarrotXLocation(0);
    this->setCarrotYLocation(0);
}

/** destructor **/
Robot::~Robot()
{
    //dtor
}

/** get functions **/

float Robot::getXLocation()
{
    return this->xLocation;
}

float Robot::getYLocation()
{
    return this->yLocation;
}

State Robot::getState()
{
    return this->state;
}

float Robot::getDirection()
{
    return this->direction;
}

float Robot::getCarrotXlocation()
{
    return this->carrotXLocation;
}

float Robot::getCarrotYlocation()
{
    return this->carrotYLocation;
}

/** set functions **/

void Robot::setXLocation(float x)
{
    this->xLocation = x;
}

void Robot::setYLocation(float y)
{
    this->yLocation = y;
}

void Robot::setDirection(float radian)
{
    this->direction = radian;
}

void Robot::setState(State state)
{
    this->state = state;
}

void Robot::setCarrotXLocation(float x)
{
    this->carrotXLocation = x;
}

void Robot::setCarrotYLocation(float y)
{
    this->carrotYLocation = y;
}

/** modifier functions **/

void Robot::changeDirection(float radian)
{
    this->direction += radian;
}

void Robot::moveRobot()
{
    /** add code here **/
}

void Robot::updateState()
{
    /** add code here **/
}

void publishThrottle()
{
    /** add code here **/
}

void publishSteering()
{
    /** add code here **/
}

void Robot::callbackUpdateRobotLocation(const geometry_msgs::Point::ConstPtr& robot)
{
    float radian = atan2(robot->y - this->getYLocation(),robot->x - this->getXLocation());
    this->setXLocation(robot->x);
    this->setYLocation(robot->y);
    this->setDirection(radian);
}

void Robot::callbackUpdateCarrotLocation(const geometry_msgs::Point::ConstPtr& carrot)
{
    this->setCarrotXLocation(carrot->x);
    this->setCarrotYLocation(carrot->y);
}
