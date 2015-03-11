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
    this->setState(followingCarrot);
}

/** destructor **/
Rabbit::~Rabbit()
{
    //dtor
}

/** get functions **/

float Rabbit::getXLocation()
{
    return this->rabbit;
}

RabbitState Rabbit::getState()
{
    return this->state;
}

float Rabbit::getSteering()
{
    return this->steering;
}

float Rabbit::getThrottle()
{
    return this->throttle;
}

/** set functions **/

void Rabbit::setRabbitPosition(Position rabbit)
{
    this->rabbit = rabbit;
}

void Rabbit::setState(RabbitState state)
{
    this->state = state;
}

/** modifier functions **/

void Rabbit::changeSteering(float radian)
{
    temp=this->steering;
    temp=temp*((7*M_PI/45));

    radian=(radian+(38*M_PI/45))%38*M_PI/45;
    radian=radian-temp;
    radian=radian/((7*M_PI/45));
    this->steering=radian;
}

void Rabbit::changeThrottle(float radian)
{
    temp=this->steering;
    temp=temp*((7*M_PI/45));

    radian=(radian+(38*M_PI/45))%38*M_PI/45;
    radian=radian-temp;
    radian=radian/((7*M_PI/45));
    this->steering=radian;
}

void Rabbit::moveRabbit()
{
    /** add code here **/
}

void Rabbit::updateState()
{
    /** add code here **/
}


void Rabbit::callbackUpdateRabbitGPSLocation(const geometry_msgs::Point::ConstPtr& rabbit)
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
