/*************************************
* Rabbit description implementation  *
*    for rabbit follow algorithm     *
* Written for SWARATH Project        *
* @author Nishant Sharma             *
* @version 1.1                       *
* @date 13, March, 2015              *
*************************************/

#include <rabbit_follow/rabbit.h>

/** constructors **/
Rabbit::Rabbit()
{
    this->rabbitState = FollowingCarrot;
}

/** destructor **/
Rabbit::~Rabbit()
{
    //dtor
}

/** get functions **/

//Position Rabbit::getRabbitPosition()
//{
//    return this->rabbit;
//}
//
//RabbitState Rabbit::getState()
//{
//    return this->state;
//}
//
//float Rabbit::getSteering()
//{
//    return this->steering;
//}
//
//float Rabbit::getThrottle()
//{
//    return this->throttle;
//}

/** set functions **/

//void Rabbit::setRabbitPosition(Position rabbit)
//{
//    this->rabbit = rabbit;
//}
//
//void Rabbit::setState(RabbitState state)
//{
//    this->state = state;
//}

/** modifier functions **/

void Rabbit::MoveRabbit(const ros::TimerEvent& event)
{

#ifdef debugRabbit
    ROS_INFO("Entering Move Rabbit Function");
#endif // debugRabbit

    CalculateVelocity(this->rabbit);
	UpdateSteering();
    UpdateThrottle();
    PublishSteering();
    PublishThrottle();

#ifdef debugRabbit
    ROS_INFO("Exiting Move Rabbit Function");
#endif // debugRabbit

}

void Rabbit::CallbackUpdateRabbitGPSLocation(const std_msgs::String::ConstPtr& rabbit)
{

#ifdef debugRabbit
    ROS_INFO("Entering Callback Update Rabbit GPS Location Function");
#endif // debugRabbit

    Position newRabbitGPSLocation;
    vector<string> temp = split(rabbit->data,',');
    newRabbitGPSLocation.x = atof(temp[0].c_str());;
    newRabbitGPSLocation.y = atof(temp[1].c_str());;
    newRabbitGPSLocation.z = atof(temp[2].c_str());;
    this->rabbit = newRabbitGPSLocation;

#ifdef debugRabbit
    ROS_INFO("Robot New Location: x = %f, y = %f, z = %f", this->rabbit.x, this->rabbit.y, this->rabbit.z);
#endif // debugRabbit

#ifdef debugRabbit
    ROS_INFO("Exiting Callback Update Rabbit GPS Location Function");
#endif // debugRabbit

}

void Rabbit::CallbackUpdateRabbitIMULocation(const std_msgs::String::ConstPtr& rabbit)
{
    vector<string> temp = split(rabbit->data,',');
    this->rabbitCurrentHeading = atof(temp[1].c_str());

#ifdef debugRabbit
    ROS_INFO("Current Updated Rabbit Heading is %f", this->rabbitCurrentHeading);
#endif // debugRabbit

}

void Rabbit::CallbackUpdateCarrotLocation(const rabbit_follow::carrotPosition::ConstPtr& carrot)
{
    this->carrotPosition.carrotDistance  = carrot->carrotDistance;
    this->carrotPosition.carrotDirection = carrot->carrotDirection;

#ifdef debugRabbit
    ROS_INFO("Carrot Distance = %f, Carrot Direction = %f", this->carrotPosition.carrotDistance, this->carrotPosition.carrotDirection);
#endif // debugRabbit

}


void Rabbit::UpdateSteering()
{

#ifdef debugRabbit
    ROS_INFO("Entering Update Steering Method");
#endif // debugRabbit

    /** set desired and current heading **/
    float desiredHeading = this->carrotPosition.carrotDirection;
    float currentHeading = this->rabbitCurrentHeading;

    /** convert current heading from degrees to radians **/
    currentHeading = ToRadians(currentHeading);

#ifdef debugRabbit
    ROS_INFO("Desired heading = %f (rad), current Heading = %f (rad)",desiredHeading, currentHeading);
#endif // debugRabbit

    /** if current heading is greater than PI then subtract 2 PI to make come in the range -pi to 0**/
    if(currentHeading>M_PI)
        currentHeading=currentHeading - (M_PI * 2);

    /** setting required turning angle **/
    float requiredTurningAngle = desiredHeading + currentHeading;
    if(requiredTurningAngle > M_PI)
        requiredTurningAngle = desiredHeading - currentHeading - M_PI;

    /** bring down required turn angle in the range -maximumAllowedTurn to maximumAllowedTurn **/
    if(requiredTurningAngle >  MaximumAllowedTurnValue)
    {
        requiredTurningAngle = MaximumAllowedTurnValue;
    }
    else if(requiredTurningAngle < ( -1 * MaximumAllowedTurnValue))
    {
        requiredTurningAngle = -1 * MaximumAllowedTurnValue;
    }

    /** normalizing the turn from -1 to 1 **/
    requiredTurningAngle = requiredTurningAngle / MaximumAllowedTurnValue;

    this->steering = -1 * requiredTurningAngle;

#ifdef debugRabbit
    ROS_INFO("Updated Steering = %f", this->steering);
#endif // debugRabbit

#ifdef debugRabbit
    ROS_INFO("Exiting Update Steering Method");
#endif // debugRabbit
}

void Rabbit::UpdateThrottle()
{

#ifdef debugRabbit
    ROS_INFO("Entering Update Throttle Function");
#endif // debugRabbit

    this->throttle = 0.3;

#ifdef debugRabbit
    ROS_INFO("Exiting Update Throttle Function");
#endif // debugRabbit

}

void Rabbit::CalculateVelocity(Position position)
{

#ifdef debugRabbit
    ROS_INFO("Entering Calculate Veloocity Function");
#endif // debugRabbit

    /** get displacement between robot's last and current location **/
    float displacement = GetEuclideanDistance(position, this->rabbitLastLocation);

#ifdef debugRabbit
    ROS_INFO("displacement = %f", displacement);
#endif // debugRabbit

    /**
        get current time
        get difference between last update time and current time
    **/
    ros::Time currentTime = ros::Time::now();
    float totalTimeElapsed = this->lastVelocityUpdateTime.sec - currentTime.sec;
    float nanoSeconds = this->lastVelocityUpdateTime.nsec - currentTime.nsec;
    if(nanoSeconds < 0)
    {
        totalTimeElapsed -= 1;
        nanoSeconds += NanoSecondsInOneSecond;
    }
    totalTimeElapsed += (nanoSeconds / NanoSecondsInOneSecond);

#ifdef debugRabbit
    ROS_INFO("time elapsed = %f", totalTimeElapsed);
#endif // debugRabbit

    /** velocity is displacemnt upon time **/
    this->currentVelocity = displacement / totalTimeElapsed;

#ifdef debugRabbit
    ROS_INFO("calculated Velocity = %f", this->currentVelocity);
#endif // debugRabbit

    /** updating last update time and rabbit last location **/
    this->lastVelocityUpdateTime = currentTime;
    this->rabbitLastLocation = position;

#ifdef debugRabbit
    ROS_INFO("Exiting Calculate velocity function");
#endif // debugRabbit

}

void Rabbit::PublishSteering()
{
    std_msgs::String steeringMessage;
    steeringMessage.data = floatToString(this->steering);
    steer_publisher.publish(steeringMessage);

#ifdef debugRabbit
    ROS_INFO("Publishing Steering to Unity");
#endif // debugRabbit

}

void Rabbit::PublishThrottle()
{
    std_msgs::String throttleMessage;
    throttleMessage.data = floatToString(this->throttle);
    throttle_publisher.publish(throttleMessage);

#ifdef debugRabbit
    ROS_INFO("Publishing Steering to Unity");
#endif // debugRabbit

}



