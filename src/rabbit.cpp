/*************************************
* Rabbit description implementation  *
*    for rabbit follow algorithm     *
* Written for SWARATH Project        *
* @author Nishant Sharma             *
* @version 1.1                       *
* @date 13, March, 2015              *
*************************************/

#include <rabbit_follow/rabbit.h>
#include <geometry_msgs/Point.h>
#include <fstream>

/** constructors **/
Rabbit::Rabbit()
{
    this->rabbitState = FollowingCarrot;
    string filename;
    cin >> filename;
    this->ReadWayPointsFromFile(filename);
    this->currentWayPointID_rab = 1;
}

/** destructor **/
Rabbit::~Rabbit()
{
    //dtor
}
int flag=1;

void Rabbit::InitializeMarker()
{
 //init headers
	rabbitMarker.header.frame_id    = rabbitCurrentHeadingMarker.header.frame_id    = rabbitDesiredHeadingMarker.header.frame_id    = "rabbit_follow";
	rabbitMarker.header.stamp       = rabbitCurrentHeadingMarker.header.stamp       = rabbitDesiredHeadingMarker.header.stamp       = ros::Time::now();
	rabbitMarker.ns                 = rabbitCurrentHeadingMarker.ns                 = rabbitDesiredHeadingMarker.ns                 = "rabbit_follow_rabbit";
	rabbitMarker.action             = rabbitCurrentHeadingMarker.action             = rabbitDesiredHeadingMarker.action             = visualization_msgs::Marker::ADD;
	rabbitMarker.pose.orientation.w = rabbitCurrentHeadingMarker.pose.orientation.w = rabbitDesiredHeadingMarker.pose.orientation.w = 1.0;

    //setting id for each marker
    rabbitMarker.id    = 2;
	rabbitCurrentHeadingMarker.id      = 3;
	rabbitDesiredHeadingMarker.id  = 4;

	//defining types
	rabbitCurrentHeadingMarker.type   = visualization_msgs::Marker::LINE_STRIP;
	rabbitDesiredHeadingMarker.type  = visualization_msgs::Marker::LINE_STRIP;
	rabbitMarker.type    = visualization_msgs::Marker::CUBE;

	//setting scale
	rabbitMarker.scale.x   = 1;
    rabbitMarker.scale.y   = 1;
    rabbitMarker.scale.z   = 1;

    rabbitCurrentHeadingMarker.scale.x = 0.21;
    rabbitDesiredHeadingMarker.scale.x = 0.21;

    //assigning colors
	rabbitMarker.color.g   = 1.0f;
	rabbitCurrentHeadingMarker.color.g     = 1.0f;

	rabbitDesiredHeadingMarker.color.b = 0.8f;
	rabbitDesiredHeadingMarker.color.b = 1.0f;

	rabbitMarker.color.a = rabbitDesiredHeadingMarker.color.a = rabbitCurrentHeadingMarker.color.a = 1.0f;
}

/** modifier functions **/

Position Rabbit::GetWayPoint(int index)
{
    return this->wayPointPath[index];
}
int Rabbit::GetCurrentWayPointID()
{
    return this->currentWayPointID_rab;
}
void Rabbit::IncrementCurrentWayPointID()
{
	this->currentWayPointID_rab += 1;
}
void Rabbit::MoveRabbit(const ros::TimerEvent& event)
{

//#ifdef debugRabbit
//    ROS_INFO("Entering Move Rabbit Function");
//#endif // debugRabbit

    CalculateVelocity(this->rabbit);
	UpdateSteering();
    UpdateThrottle();
    PublishSteering();
    PublishThrottle();

//#ifdef debugRabbit
//    ROS_INFO("Exiting Move Rabbit Function");
//#endif // debugRabbit

}

void Rabbit::CallbackUpdateRabbitGPSLocation(const std_msgs::String::ConstPtr& rabbit)
{
//
//#ifdef debugRabbit
//    ROS_INFO("Entering Callback Update Rabbit GPS Location Function");
//#endif // debugRabbit

    Position newRabbitGPSLocation;
    vector<string> temp = split(rabbit->data,',');
    newRabbitGPSLocation.x = atof(temp[0].c_str());;
    newRabbitGPSLocation.y = atof(temp[1].c_str());;
    newRabbitGPSLocation.z = atof(temp[2].c_str());;
    this->rabbit = newRabbitGPSLocation;

//#ifdef debugRabbit
//    ROS_INFO("Robot New Location: x = %f, y = %f, z = %f", this->rabbit.x, this->rabbit.y, this->rabbit.z);
//#endif // debugRabbit
//
//#ifdef debugRabbit
//    ROS_INFO("Exiting Callback Update Rabbit GPS Location Function");
//#endif // debugRabbit

}

void Rabbit::CallbackUpdateRabbitIMULocation(const std_msgs::String::ConstPtr& rabbit)
{
    vector<string> temp = split(rabbit->data,',');
    this->rabbitCurrentHeading = atof(temp[1].c_str());

//#ifdef debugRabbit
//    ROS_INFO("Current Updated Rabbit Heading is %f", this->rabbitCurrentHeading);
//#endif // debugRabbit

}

void Rabbit::CallbackUpdateCarrotLocation(const rabbit_follow::carrotPosition::ConstPtr& carrot)
{
    if(carrot->rabbitState == 2)
{
        this->throttle = 0;
        PublishThrottle();
    exit(1);
}
    this->carrotPosition.carrotDistance  = carrot->carrotDistance;
    this->carrotPosition.carrotDirection = carrot->carrotDirection;
    this->carrotPosition.rabbitState = carrot->rabbitState;
    this->carrotPosition.rabbitDistanceToWaypoint = carrot->rabbitDistanceToWaypoint;
    this->carrotPosition.carrotx=this->carrot.x;
    this->carrotPosition.carrotz=this->carrot.z;
//#ifdef debugRabbit
//    ROS_INFO("Carrot Distance = %f, Carrot Direction = %f", this->carrotPosition.carrotDistance, this->carrotPosition.carrotDirection);
//#endif // debugRabbit

}


void Rabbit::UpdateSteering()
{

//#ifdef debugRabbit
//    ROS_INFO("Entering Update Steering Method");
//#endif // debugRabbit
    //float boundary=0.8;




    float desiredHeading = this->carrotPosition.carrotDirection;
    float currentHeading = this->rabbitCurrentHeading;
    float psi_new;
    //float q22_gain=0.8;
     currentHeading = ToRadians(currentHeading);
     currentHeading=(-1)*ang_wrap(currentHeading);
    float vel=this->currentVelocity;
    float theta=GetAngle(GetWayPoint(currentWayPointID_rab), GetWayPoint(currentWayPointID_rab-1));
    float DistFmWp = GetEuclideanDistance(this->rabbit,GetWayPoint(currentWayPointID_rab-1));
     float q22_gain;
    float boundary;
    float theta_m=GetAngle(this->rabbit, GetWayPoint(currentWayPointID_rab-1));
     float  ct_err=DistFmWp*sin(theta-theta_m);
    //int flag=1;
    /** set desired and current heading **/
    if (GetEuclideanDistance(this->rabbit,GetWayPoint(currentWayPointID_rab))< (8)&&(flag==1))
    {IncrementCurrentWayPointID();
    flag=0;
    }
    else flag=1;

    //else{
     q22_gain=0.8;
     boundary=0.8;
    //}
   // float DistFmWp=sqrt( pow( carrotPosition.carrotx- GetWayPoint(currentWayPointID-1).x ,2) + pow( carrotPosition.carrotz- GetWayPoint(currentWayPointID-1).z  ,2) );

 //  float theta_m=atan2((carrotPosition.carrotz - GetWayPoint(currentWayPointID-1).z), (carrotPosition.carrotx- GetWayPoint(currentWayPointID-1).x));

     ROS_INFO("current way point %d \n waypointdist %f \n crosstrack %f", GetCurrentWayPointID(), this->carrotPosition.rabbitDistanceToWaypoint,ct_err);


    /** ct_errdot **/
    float vd=vel*sin(ang_wrap(currentHeading-theta));
    //float q11=abs(2/(2-abs(ct_err)));
    float q11=abs(boundary/(boundary-abs(ct_err)));
    ct_err=-ct_err;
    float u=-(sqrt(q11)*ct_err + sqrt(2*sqrt(q11)+q22_gain)*vd);
    float temp=GetEuclideanDistance(this->rabbit,GetWayPoint(currentWayPointID_rab));
    float temp1=GetEuclideanDistance(GetWayPoint(currentWayPointID_rab -1),GetWayPoint(currentWayPointID_rab));
//    if (u>0.07) u=0.07;
//    if (u< -0.07) u=-0.07;
     if (DistFmWp<=6&&(temp1-temp<6))
 {// ct_err=-ct_err;
    //{
     psi_new=ang_wrap(desiredHeading-currentHeading);
    }

   else  psi_new=0.1*(u/vel);

    /** bring down required turn angle in the range -maximumAllowedTurn to maximumAllowedTurn **/
    if(psi_new >  MaximumAllowedTurnValue)
    {
        psi_new = MaximumAllowedTurnValue;
    }
    else if(psi_new < ( -1 * MaximumAllowedTurnValue))
    {
        psi_new = -1 * MaximumAllowedTurnValue;
    }

    /** normalizing the turn from -1 to 1 **/
    psi_new = psi_new / MaximumAllowedTurnValue;

    this->steering = -1 * psi_new;









    /** convert current heading from degrees to radians **/


    /** if current heading is greater than PI then subtract 2 PI to make come in the range -pi to 0**/
//    if(currentHeading>M_PI)
//        currentHeading -= (M_PI * 2);
//
//    currentHeading *= -1;
//
//    /** convert current heading from clockwise to anticlockwise **/
//
//
//#ifdef debugRabbit
//    ROS_INFO("Desired heading = %f (rad), current Heading = %f (rad)",desiredHeading, currentHeading);
//#endif // debugRabbit
//
//    /** setting required turning angle **/
//    float requiredTurningAngle = desiredHeading - currentHeading;
//    if(requiredTurningAngle > M_PI)
//        requiredTurningAngle -= (2* M_PI);
//    if(requiredTurningAngle < (-1 * M_PI))
//        requiredTurningAngle += (2 * M_PI);
//
//    /** bring down required turn angle in the range -maximumAllowedTurn to maximumAllowedTurn **/
//    if(requiredTurningAngle >  MaximumAllowedTurnValue)
//    {
//        requiredTurningAngle = MaximumAllowedTurnValue;
//    }
//    else if(requiredTurningAngle < ( -1 * MaximumAllowedTurnValue))
//    {
//        requiredTurningAngle = -1 * MaximumAllowedTurnValue;
//    }
//
//    /** normalizing the turn from -1 to 1 **/
//    requiredTurningAngle = requiredTurningAngle / MaximumAllowedTurnValue;
//
//    this->steering = -1 * requiredTurningAngle;

    geometry_msgs::Point position;

    position.x = this->rabbit.x;
    position.y = this->rabbit.z;
    position.z = 0;

    rabbitCurrentHeadingMarker.points.clear();

    rabbitCurrentHeadingMarker.points.push_back(position);
    position.x = position.x + 2*cos(currentHeading );
    position.y = position.y + 2*sin(currentHeading );

    rabbitCurrentHeadingMarker.points.push_back(position);

/**#ifdef debugRabbit
    ROS_INFO("Updated Steering = %f", this->steering);
#endif**/ // debugRabbit

//#ifdef debugRabbit
//    ROS_INFO("Exiting Update Steering Method");
//#endif // debugRabbit
}

void Rabbit::UpdateThrottle()
{
    float deltaTime = 0.1;
    float tempThrottle = 0;
    //float distanceRabbitToWayPoint = 10;//GetEuclideanDistance(this->rabbit,wayPointPath[currentWayPointID]);


    //if (this->carrotPosition.rabbitState == rabbit_follow::carrotPosition::NearWayPoint)
    if (this->carrotPosition.rabbitDistanceToWaypoint < (8))
    {
        //breaking
        //float currentTimeToCarrot = this->carrotPosition.carrotDistance / this->currentVelocity;
        ROS_INFO("Breaking");
        float velocitySquare = this->currentVelocity * this->currentVelocity;
        float requiredAcceleration = -1 * velocitySquare / (2 * 8);
        tempThrottle = requiredAcceleration;
        tempThrottle *=(0.75);
        if(this->currentVelocity  < 0.25)
                tempThrottle = 0.035;
    }
    else
    {
        ROS_INFO("Normal");
        //accelerate
        tempThrottle = (((0.1 * this->carrotPosition.carrotDistance) - (this->currentVelocity * deltaTime))*2) / (deltaTime * deltaTime); // s = ut + 1/2 at^2
        if(tempThrottle < 0) tempThrottle = 0;
    }
    //ROS_INFO("Throttle %f - %f",this->throttle, tempThrottle);
    if(tempThrottle > 0.07) tempThrottle = 0.07;
    if(tempThrottle < -0.07) tempThrottle = -0.07;

    tempThrottle /= 0.07;

    if(currentVelocity > MaximumAllowedVelocity) tempThrottle = 0;

    this->throttle = tempThrottle;
   // ROS_INFO("Throttle %f - %f",this->throttle, tempThrottle);
}

void Rabbit::CalculateVelocity(Position position)
{

//#ifdef debugRabbit
//    ROS_INFO("Entering Calculate Veloocity Function");
//#endif // debugRabbit

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
    float totalTimeElapsed = currentTime.sec - this->lastVelocityUpdateTime.sec;

    int nanoSeconds = currentTime.nsec - this->lastVelocityUpdateTime.nsec;


   if(nanoSeconds < 0)
    {
        totalTimeElapsed -= 1;
        nanoSeconds += NanoSecondsInOneSecond;
    }
    totalTimeElapsed += (nanoSeconds / NanoSecondsInOneSecond);
     this->currentVelocity = displacement / totalTimeElapsed;

#ifdef debugRabbit
    ROS_INFO("time elapsed = %f", totalTimeElapsed);
#endif // debugRabbit

    /** velocity is displacemnt upon time **/


#ifdef debugRabbit
    ROS_INFO("calculated Velocity = %f", this->currentVelocity);
#endif // debugRabbit

    /** updating last update time and rabbit last location **/
    this->lastVelocityUpdateTime = currentTime;
    this->rabbitLastLocation = position;
//
//#ifdef debugRabbit
//    ROS_INFO("Exiting Calculate velocity function");
//#endif // debugRabbit

}

void Rabbit::PublishSteering()
{
    std_msgs::String steeringMessage;
    steeringMessage.data = floatToString(this->steering);
    steer_publisher.publish(steeringMessage);

    geometry_msgs::Point position;

    position.x = this->rabbit.x;
    position.y = this->rabbit.z;
    position.z = 0;

    rabbitMarker.pose.position = position;

    rabbitDesiredHeadingMarker.points.clear();


    rabbitDesiredHeadingMarker.points.push_back(position);


    position.x = position.x + 3*cos(this->carrotPosition.carrotDirection);
    position.y = position.y + 3*sin(this->carrotPosition.carrotDirection);

    rabbitDesiredHeadingMarker.points.push_back(position);

    publisher_rabbit_location_rviz.publish(rabbitMarker);
    publisher_rabbit_location_rviz.publish(rabbitDesiredHeadingMarker);
    publisher_rabbit_location_rviz.publish(rabbitCurrentHeadingMarker);

//#ifdef debugRabbit
//    ROS_INFO("Publishing Steering to Unity");
//#endif // debugRabbit

}

void Rabbit::PublishThrottle()
{
    std_msgs::String throttleMessage;
    throttleMessage.data = floatToString(this->throttle);
    throttle_publisher.publish(throttleMessage);

//#ifdef debugRabbit
//    ROS_INFO("Publishing Steering to Unity");
//#endif // debugRabbit

}

void Rabbit::ReadWayPointsFromFile(string filename)
{

#ifdef debugCarrot
 ROS_INFO("Starting read Way points from file");
#endif // debugCarrot

 if(filename[0] == '\0') return;
 Position position;
 geometry_msgs::Point posi;
 this->wayPointPath.clear();
 string line;

 ifstream myfile (filename.c_str());
 if (myfile.is_open())
 {
 while ( getline (myfile,line) )
 {
 vector<string> temp = split(line,',');
 posi.x = position.x = atof(temp[0].c_str());
 position.y = atof(temp[1].c_str());
 posi.y = position.z = atof(temp[2].c_str());
 posi.z =00;

 this->wayPointPath.push_back(position);
 }
 myfile.close();
 }
 else
 {
 ROS_INFO("Unable to read File");
 exit(1);
 }
#ifdef debugCarrot
 ROS_INFO("Reading way points from file complete");
#endif // debugCarrot
}
