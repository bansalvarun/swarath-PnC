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
    float temp=this->currentHeading;
    temp=(temp*M_PI)/180;
    if (temp>M_PI)
    temp=temp-M_PI;
    float radian_1=radian+temp;
    if (radian_1>M_PI)
    radian_1=radian-temp-M_PI;





//    //temp=fmod(temp+(38*(M_PI/45)),38 * (M_PI/45));
//
    //radian=radian/M_PI;

    //radian = fmod(radian+(38*(M_PI/45)),7 * (M_PI/45));
    //radian=radian-temp;
    //ROS_INFO("steering %f",radian * 180 / M_PI);
    if (radian_1 >  (7*M_PI/45)) radian_1=(7*M_PI/45);
    else if (radian < - (7*M_PI/45)) radian =-(7*M_PI/45);

    radian_1=radian_1/((7*M_PI/45));
    ROS_INFO("steering %f",radian_1); //This is not getting printed which means this function isnot being called properly
    //if(radian > 1) radian = 1;
    //if(radian < -1) radian = -1;
    this->steering=-radian_1;
}

void Rabbit::moveRabbit(const ros::TimerEvent& event)
{
	changeSteering(this->carrotPos.carrotDirection);
    float deltaT = 0.1;
    float tempThrottle = ((this->carrotPos.carrotDistance - (this->velocity * deltaT))*2) / (deltaT * deltaT); // s = ut + 1/2 at^2

	ROS_INFO("tempTh %f, velocity %f",tempThrottle, velocity);

    if (this->carrotPos.carrotDistance <= MaximumDistanceFromRabbit)
    {
        tempThrottle *= -1;
    }

    tempThrottle /= 55000;

    this->throttle = tempThrottle;


	//ROS_INFO("tempTh %f, velocity %f",tempThrottle, velocity);

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

int counter =50;

void Rabbit::callbackUpdateRabbitGPSLocation(const std_msgs::String::ConstPtr& rabbit)
{
    counter++;
    Position tempGPS;
    vector<string> temp = split(rabbit->data,',');
    tempGPS.x = atof(temp[0].c_str());;
    tempGPS.y = atof(temp[1].c_str());;
    tempGPS.z = atof(temp[2].c_str());;

    float distance = getEuclideanDistance(tempGPS,getRabbitPosition());
    ros::Time time = ros::Time::now();

    float totalTime = lastUpdateTime.sec - time.sec;
    int nsec = lastUpdateTime.nsec - time.nsec;
    if(nsec < 0)
    {
        totalTime -= 1;
        nsec += 1000000000;
    }
    totalTime += (nsec/1000000000.0);
//    if (counter > 12)
//    {
//        this->currentHeading = atan2((int)tempGPS.x - (int)this->rabbit.x,(int)tempGPS.z - (int)this->rabbit.z);
//        ROS_INFO("direct %f",this->currentHeading);
//        counter =0;
//    }
    this->velocity = distance/totalTime;
    this->rabbit = tempGPS;
    this->lastUpdateTime = ros::Time::now();
}

void Rabbit::callbackUpdateRabbitIMULocation(const std_msgs::String::ConstPtr& rabbit)
{
    vector<string> temp = split(rabbit->data,',');
    this->currentHeading = atof(temp[1].c_str());
//    this->steering -= 180;
//    ROS_INFO("direct %f",this->steering);
//    this->steering *= M_PI / 180; //already set = no change
}

void Rabbit::callbackUpdateCarrotLocation(const rabbit_follow::carrotPosition::ConstPtr& carrot)
{
    this->carrotPos.carrotDistance  = carrot->carrotDistance;
    this->carrotPos.carrotDirection = carrot->carrotDirection;
}
