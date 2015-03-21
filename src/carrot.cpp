/*********************************************************
* Carrot implementation file for rabbit follow algorithm *
* Written for SWARATH Project                            *
* @author Nishant Sharma                                 *
* @version 1.1                                           *
* @date 13, March, 2015                                  *
*********************************************************/

#include <rabbit_follow/carrot.h>
#include <fstream>

/** Default constructor */
Carrot::Carrot(char* filename)
{
    this->ReadWayPointsFromFile(filename);
    this->carrot.x = 0;
    this->carrot.z = 0;
    this->currentWayPointID = 1;
    this->carrotState = MovingOnLine;

#ifdef debugCarrot
    ROS_INFO("Initialized Carrot Object");
#endif // debugCarrot

}

/** Default destructor */
Carrot::~Carrot()
{

}

void Carrot::InitializeMarker()
{
    //init headers
	carrotMarker.header.frame_id    = wayPointMarker.header.frame_id = "rabbit_follow";
	carrotMarker.header.stamp       = wayPointMarker.header.stamp = ros::Time::now();
	carrotMarker.ns                 = wayPointMarker.ns = "rabbit_follow_carrot";
	carrotMarker.action             = wayPointMarker.action = visualization_msgs::Marker::ADD;
	carrotMarker.pose.orientation.w = wayPointMarker.pose.orientation.w  = 1.0;

    //setting id for each marker
    carrotMarker.id = 0;
    wayPointMarker.id = 1;
	//defining types
	carrotMarker.type = visualization_msgs::Marker::SPHERE;
    wayPointMarker.type = 4;
	//setting scale
	carrotMarker.scale.x = 1;
    carrotMarker.scale.y = 1;
    carrotMarker.scale.z = 1;

    wayPointMarker.scale.x = 0.3;
    //assigning colors
	carrotMarker.color.b   = 1.0f;
	carrotMarker.color.a   = 1.0f;

    wayPointMarker.color.r = 0;
    wayPointMarker.color.g = 0;
    wayPointMarker.color.b = 0;
    wayPointMarker.color.a = 1;

}


/** get functions **/
//Position Carrot::getCarrotLocation()
//{
//    return this->carrot;
//}

//Position Carrot::getRabbitLocation()
//{
//    return this->rabbit;
//}

//CarrotState Carrot::getState()
//{
//    return this->state;
//}

Position Carrot::GetWayPoint(int index)
{
    return this->wayPointPath[index];
}

int Carrot::GetCurrentWayPointID()
{
    return this->currentWayPointID;
}

/** set functions **/
//void Carrot::setCarrotLocation(Position carrot)
//{
//    this->carrot = carrot;
//}
//
//void Carrot::setRabbitLocation(Position rabbit)
//{
//    this->rabbit = rabbit;
//}
//
//void Carrot::setState(CarrotState state)
//{
//    this->state = state;
//}

void Carrot::IncrementCurrentWayPointID()
{
	this->currentWayPointID += 1;
}

void Carrot::SetCurrentWayPointID(int index)
{
	this->currentWayPointID = index;
}

void Carrot::SetCarrotRabbitPosition(float distance, float direction, float rabbitDistanceToWaypoint, int rabbitState)
{
    this->carrotPosition.rabbitDistanceToWaypoint = rabbitDistanceToWaypoint;
    this->carrotPosition.carrotDirection = direction;
    this->carrotPosition.carrotDistance = distance;
    this->carrotPosition.rabbitState = rabbitState;

#ifdef debugCarrot
    //ROS_INFO("carrot Direction = %f, carrot Distance = %f", direction, distance);
#endif // debugCarrot

}


/** modifier functions **/

void Carrot::UpdateCarrotWhenReachedWaypoint()
{

    float carrotHeadingAngle = GetAngle(GetWayPoint(currentWayPointID), GetWayPoint(currentWayPointID-1));
    Position carrotNewPosition;
    carrotNewPosition.x = GetWayPoint(currentWayPointID-1).x + ((MaximumDistanceOnTurn) * cos(carrotHeadingAngle));
    carrotNewPosition.z = GetWayPoint(currentWayPointID-1).z + ((MaximumDistanceOnTurn) * sin(carrotHeadingAngle));
    this->carrot = carrotNewPosition;

    float carrotDistance = GetEuclideanDistance(this->carrot,this->rabbit);
    float carrotDirection = GetAngle(this->carrot,this->rabbit);
    SetCarrotRabbitPosition(carrotDistance, carrotDirection, carrotDistance, rabbit_follow::carrotPosition::NearWayPoint);

    float rabbitToCurrentWaypointDistance  = GetEuclideanDistance(this->rabbit, GetWayPoint(currentWayPointID-1));


    float waypointdistance = GetEuclideanDistance(this->rabbit,GetWayPoint(currentWayPointID-1));
    /** if carrot distance from rabbit is less than the specified minimum distance
            then change carrot state to moving on line
        else
            update current distance and direction of rabbit from carrot
    **/
    if(carrotDistance < MinAllowedDistanceCarrotToRabbit)
    {



        this->carrotState = MovingOnLine;
        ROS_INFO("current way point %d \n waypoint size %ld", GetCurrentWayPointID(), wayPointPath.size());
        /** if this was the last waypoint, carrotState will be reached end **/
        if(GetCurrentWayPointID() >= wayPointPath.size())
        {
            this->carrotState = ReachedEndDestination;
            SetCarrotRabbitPosition(carrotDistance, carrotDirection, rabbitToCurrentWaypointDistance, rabbit_follow::carrotPosition::ReachedEnd);
            PublishCarrotPosition();
            ros::Duration(1).sleep();
            exit(1);
        }
#ifdef debugCarrot
     //   ROS_INFO("Changing carrot state to moving to line");
#endif // debugCarrot

    }
    else
    {
        //carrotDirection = GetAngle(this->carrot,this->rabbit);
//             Position robotLineIntersectionPoint;
//        GetPerpendicularLineIntersection(GetWayPoint(currentWayPointID-1), GetWayPoint(currentWayPointID), this->rabbit, robotLineIntersectionPoint);
//
//        float carrotHeadingAngle = GetAngle(GetWayPoint(currentWayPointID), GetWayPoint(currentWayPointID-1));
//        Position carrotNewPosition;
//        carrotNewPosition.x = robotLineIntersectionPoint.x + ((MaximumDistanceFromRabbit-carrotDistance) * cos(carrotHeadingAngle));
//        carrotNewPosition.z = robotLineIntersectionPoint.z + ((MaximumDistanceFromRabbit-carrotDistance) * sin(carrotHeadingAngle));
//        this->carrot = carrotNewPosition;
//
//        float carrotDistance = GetEuclideanDistance(this->carrot,this->rabbit);
//        float carrotDirection = GetAngle(this->carrot,this->rabbit);
        SetCarrotRabbitPosition(carrotDistance, carrotDirection, rabbitToCurrentWaypointDistance, rabbit_follow::carrotPosition::NearWayPoint);

#ifdef debugCarrot
        ROS_INFO("Carrot Distance = %f, Carrot Direction = %f", carrotDistance, carrotDirection);
        ros::Duration(1).sleep();
#endif // debugCarrot

    }

//#ifdef debugCarrot
//    ROS_INFO("Exiting Update Carrot When Reached Waypoint Function");
//#endif // debugCarrot

}

void Carrot::UpdateCarrotWhenMovingOnLine()
{

//#ifdef debugCarrot
//    ROS_INFO("Entering Update Carrot When Moving On Line Function");
//#endif // debugCarrot

    /** get robot projection on the current line that is being followed **/
    Position robotLineIntersectionPoint;
    GetPerpendicularLineIntersection(GetWayPoint(currentWayPointID-1), GetWayPoint(currentWayPointID), this->rabbit, robotLineIntersectionPoint);

#ifdef debugCarrot
   // ROS_INFO("Got robot Line Intersection Point: x = %f, y = %f, z = %f", robotLineIntersectionPoint.x, robotLineIntersectionPoint.y, robotLineIntersectionPoint.z);
#endif // debugCarrot

    /** place carrot a fixed distance ahead of robot on that line **/
    float carrotHeadingAngle = GetAngle(GetWayPoint(currentWayPointID), GetWayPoint(currentWayPointID-1));
    Position carrotNewPosition;
    carrotNewPosition.x = robotLineIntersectionPoint.x + ((MaximumDistanceFromRabbit) * cos(carrotHeadingAngle));
    carrotNewPosition.z = robotLineIntersectionPoint.z + ((MaximumDistanceFromRabbit) * sin(carrotHeadingAngle));

#ifdef debugCarrot
    //ROS_INFO("Updated carrot position on the line");
#endif // debugCarrot

    /** check if carrot is still on the line segment or not **/
    float carrotToLastWaypointDistance, intersectionToLastWaypointDistance;
    carrotToLastWaypointDistance  = GetEuclideanDistance(carrotNewPosition, GetWayPoint(currentWayPointID-1));
    float rabbitToCurrentWaypointDistance  = GetEuclideanDistance(this->rabbit, GetWayPoint(currentWayPointID));
    intersectionToLastWaypointDistance = GetEuclideanDistance(GetWayPoint(currentWayPointID-1), GetWayPoint(currentWayPointID));

#ifdef debugCarrot
 //   ROS_INFO("carrot ot wapnt = %f",carrotToCurrentWaypointDistance);
#endif // debugCarrot

    /** if carrot is crossing the line segment
            then carrot location is waypoint value and carrot state is reached waypoint
        else
            carrot location is the new carrot location
    **/
    //if (carrotToLastWaypointDistance > intersectionToLastWaypointDistance )//&&
    if(rabbitToCurrentWaypointDistance < 6)
    {
        this->carrot = GetWayPoint(currentWayPointID);
        this->carrotState = ReachedWaypoint;
        IncrementCurrentWayPointID();
    }
    else
    {
        this->carrot = carrotNewPosition;
    }

    float carrotDistance = GetEuclideanDistance(this->carrot, this->rabbit);
    float carrotDirection = GetAngle(this->carrot, this->rabbit);

    SetCarrotRabbitPosition(carrotDistance, carrotDirection, GetEuclideanDistance(robotLineIntersectionPoint,GetWayPoint(currentWayPointID)), rabbit_follow::carrotPosition::FollowingCarrot);

//#ifdef debugCarrot
//    ROS_INFO("Exiting Update Carrot When Moving On Line Function");
//#endif // debugCarrot

}

void Carrot::MoveCarrot(const ros::TimerEvent& event)
{

//#ifdef debugCarrot
//    ROS_INFO("Entering Move Carrot Function");
//#endif // debugCarrot

    switch(this->carrotState)
    {
        case(ReachedEndDestination):
            ROS_INFO("Carrot State: ReachedEndDestination");
            exit(1); /** If carrot is at the endwaypoint with rabbit at that point, end carrot**/
        case(ReachedWaypoint):
            ROS_INFO("Carrot State: On Waypoint");
            UpdateCarrotWhenReachedWaypoint();
            break;
        case(MovingOnLine):
        ROS_INFO("Carrot State: Moving on Line");
            UpdateCarrotWhenMovingOnLine();
            break;
        default:;
    }

#ifdef debugCarrot
    //ROS_INFO("Publishing Carrot distance and direction");
#endif // debugCarrot

    PublishCarrotPosition();
    lidarSensor.setRvizMarkerData(this->rabbit, this->rabbitCurrentHeading);
//#ifdef debugCarrot
//    ROS_INFO("Exiting Move Carrot Function");
//#endif // debugCarrot

}

void Carrot::PublishCarrotPosition()
{
    carrotMarker.pose.position.x = this->carrot.x;
    carrotMarker.pose.position.y = this->carrot.z;
    carrotMarker.pose.position.z = 0;
    this->publisher_carrot_location_rviz.publish(carrotMarker);
    this->publisher_carrot_location_rviz.publish(wayPointMarker);

    this->publisher_carrot_robot.publish(carrotPosition);
}

void Carrot::CallbackUpdateRabbitLocation(const std_msgs::String::ConstPtr& rabbit)
{

//#ifdef debugCarrot
//    ROS_INFO("Entering Callback Update Rabbit Location Function");
//#endif // debugCarrot

    /** assigning received information to rabbit location **/
    vector<string> temp = split(rabbit->data,',');
    this->rabbit.x = atof(temp[0].c_str());;
    this->rabbit.y = atof(temp[1].c_str());;
    this->rabbit.z = atof(temp[2].c_str());;

//#ifdef debugCarrot
//    ROS_INFO("New Rabbit Values are: x = %f, y = %f, z = %f", this->rabbit.x, this->rabbit.y, this->rabbit.z);
//#endif // debugCarrot
//
//#ifdef debugCarrot
//    ROS_INFO("Exiting Callback Update Rabbit Location Function");
//#endif // debugCarrot

}

void Carrot::ReadWayPointsFromFile(char* filename)
{

#ifdef debugCarrot
    ROS_INFO("Starting read Way points from file");
#endif // debugCarrot

    if(filename[0] == '\0') return;
    Position  position;
    geometry_msgs::Point posi;
    this->wayPointPath.clear();
    string line;
    wayPointMarker.points.clear();
    ifstream myfile (filename);
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            vector<string> temp = split(line,',');
            posi.x = position.x = atof(temp[0].c_str());
            position.y = atof(temp[1].c_str());
            posi.y = position.z = atof(temp[2].c_str());
            posi.z =00;
            wayPointMarker.points.push_back(posi);
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

void Carrot::GetPerpendicularLineIntersection(Position linePointOne, Position linePointTwo, Position robotLocation, Position &intersectionPoint)
{
    // first convert line to normalized unit vector
    double dx = linePointTwo.x - linePointOne.x;
    double dz = linePointTwo.z - linePointOne.z;
    double mag = sqrt(dx*dx + dz*dz);
    dx /= mag;
    dz /= mag;

    // translate the point and get the dot product
    double lambda = (dx * (robotLocation.x - linePointOne.x)) + (dz * (robotLocation.z - linePointOne.z));
    intersectionPoint.x = (dx * lambda) + linePointOne.x;
    intersectionPoint.z = (dz * lambda) + linePointOne.z;
}

void Carrot::CallbackUpdateRabbitIMULocation(const std_msgs::String::ConstPtr& rabbit)
{
    vector<string> temp = split(rabbit->data,',');
    this->rabbitCurrentHeading = atof(temp[1].c_str());
    /** convert current heading from degrees to radians **/
    this->rabbitCurrentHeading = ToRadians(this->rabbitCurrentHeading);

    /** if current heading is greater than PI then subtract 2 PI to make come in the range -pi to 0**/
    if(this->rabbitCurrentHeading>M_PI)
        this->rabbitCurrentHeading -= (M_PI * 2);

    this->rabbitCurrentHeading *= -1;
}
