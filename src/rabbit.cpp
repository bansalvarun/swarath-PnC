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

Position Rabbit::getRabbitPosition()
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
    float temp=this->steering;
    temp=temp*((7*M_PI/45));

    radian = fmod(radian+(38*(M_PI/45)),38 * (M_PI/45));
    radian=radian-temp;
    radian=radian/((7*M_PI/45));
    this->steering=radian;
}

void Rabbit::moveRabbit(const ros::TimerEvent& event)
{
    float deltaT = 0.1;
    float tempThrottle = ((this->carrotPos.carrotDistance - (this->velocity * deltaT))*2) / (deltaT * deltaT); // s = ut + 1/2 at^2

    if (this->carrotPos.carrotDistance <= MaximumDistanceFromRabbit)
    {
        tempThrottle *= -1;
    }

    tempThrottle /= 5.5;

    this->throttle = tempThrottle;

    publishSteering();
    publishThrottle();

}

void Rabbit::publishSteering()
{
    std_msgs::String temp;
    temp.data = floatToString(this->getSteering());
    steer_publisher.publish(temp);
}

void Rabbit::publishThrottle()
{
    std_msgs::String temp;
    temp.data = floatToString(this->getThrottle());
    throttle_publisher.publish(temp);
}

void Rabbit::callbackUpdateRabbitGPSLocation(const std_msgs::String::ConstPtr& rabbit)
{
    Position tempGPS;
    vector<string> temp = split(rabbit->data,',');
    tempGPS.x = atof(temp[0].c_str());;
    tempGPS.y = atof(temp[1].c_str());;
    tempGPS.z = atof(temp[2].c_str());;

    temp distance = getEuclideanDistance(tempGPS,getRabbitPosition());
    ros::Time time = ros::Time::now();
    this->lastUpdateTime = ros::Time::now();
    float totalTime = endTime.sec - time.sec;
    int nsec = endTime.nsec - time.nsec;
    if(nsec < 0)
    {
        totalTime -= 1;
        nsec += 1000000000;
    }
    totalTime += (nsec/1000000000);

    this->velocity = distance/totalTime;
}

void Rabbit::callbackUpdateRabbitIMULocation(const std_msgs::String::ConstPtr& rabbit)
{
    vector<string> temp = split(rabbit->data,',');
    this->steering = atof(temp[1].c_str());
    this->steering -= 180;
    this->steering *= M_PI / 180; //already set = no change
}

void Rabbit::callbackUpdateCarrotLocation(const rabbit_follow::carrotPosition::ConstPtr& carrot)
{
    this->carrotPos.carrotDistance  = carrot->carrotDistance;
    this->carrotPos.carrotDirection = carrot->carrotDirection;
}
