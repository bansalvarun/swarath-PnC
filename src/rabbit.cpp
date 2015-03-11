/************************************
* Rabbit description implementation  *
*    for rabbit follow algorithm    *
* Written for SWARATH Project       *
* @author Nishant Sharma            *
* @version 0.0                      *
* @date 4, March, 2015              *
************************************/

#include <rabbit_follow/rabbit.h>

/** constructors **/
Rabbit::Rabbit()
{
    this->setXLocation(0);
    this->setYLocation(0);
    this->setDirection(0);
    this->setState(followingCarrot);
    this->setCarrotXLocation(0);
    this->setCarrotYLocation(0);
}

Rabbit::Rabbit(float x, float y, float radian, RabbitState state)
{
    this->setXLocation(x);
    this->setYLocation(y);
    this->setDirection(radian);
    this->setState(state);
    this->setCarrotXLocation(0);
    this->setCarrotYLocation(0);
}

/** destructor **/
Rabbit::~Rabbit()
{
    //dtor
}

/** get functions **/

float Rabbit::getXLocation()
{
    return this->xLocation;
}

float Rabbit::getYLocation()
{
    return this->yLocation;
}

RabbitState Rabbit::getState()
{
    return this->state;
}

float Rabbit::getDirection()
{
    return this->direction;
}

float Rabbit::getCarrotXlocation()
{
    return this->carrotXLocation;
}

float Rabbit::getCarrotYlocation()
{
    return this->carrotYLocation;
}

/** set functions **/

void Rabbit::setXLocation(float x)
{
    this->xLocation = x;
}

void Rabbit::setYLocation(float y)
{
    this->yLocation = y;
}

void Rabbit::setDirection(float radian)
{
    this->direction = radian;
}

void Rabbit::setState(RabbitState state)
{
    this->state = state;
}

void Rabbit::setCarrotXLocation(float x)
{
    this->carrotXLocation = x;
}

void Rabbit::setCarrotYLocation(float y)
{
    this->carrotYLocation = y;
}

/** modifier functions **/

void Rabbit::changeDirection(float radian)
{
    temp=this->direction;
    temp=temp*((7*M_PI/45));
    
    radian=(radian+(38*M_PI/45))%38*M_PI/45;
    radian=radian-temp;
    radian=radian/((7*M_PI/45));
    this->direction=radian;
    
}

void Rabbit::moveRabbit()
{
    /** add code here **/
}

void Rabbit::updateState()
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

void Rabbit::callbackUpdateRabbitLocation(const geometry_msgs::Point::ConstPtr& rabbit)
{
    float radian = atan2(rabbit->y - this->getYLocation(),rabbit->x - this->getXLocation());
    this->setXLocation(rabbit->x);
    this->setYLocation(rabbit->y);
    this->setDirection(radian);
}

void Rabbit::callbackUpdateCarrotLocation(const geometry_msgs::Point::ConstPtr& carrot)
{
    this->setCarrotXLocation(carrot->x);
    this->setCarrotYLocation(carrot->y);
}
