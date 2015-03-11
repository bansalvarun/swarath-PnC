#include <rabbit_follow/carrot.h>
#include <ros/ros.h>
#include <fstream>

/** Default constructor */
Carrot::Carrot(string filename)
{
    this->readWayPointsFromFile(filename);
    this->carrot.x = this->getWayPoint(0).x;
    this->carrot.z = this->getWayPoint(0).z;
    this->currentWayPointID=0;
    this->state = MovingOnLine;
}

/** Default destructor */
Carrot::~Carrot()
{

}

/** get functions **/
Position Carrot::getCarrotLocation()
{
    return this->carrot;
}

Position Carrot::getRabbitLocation()
{
    return this->rabbit;
}

CarrotState Carrot::getState()
{
    return this->state;
}

Position Carrot::getWayPoint(int id)
{
    return this->wayPointPath[id];
}

int Carrot::getCurrentWayPointID()
{
    this->currentWayPointID;
}

/** set functions **/
void Carrot::setCarrotLocation(Position carrot)
{
    this->carrot = carrot;
}

void Carrot::setRabbitLocation(Position rabbit)
{
    this->rabbit = rabbit;
}

void Carrot::setState(CarrotState state)
{
    this->state = state;
}

void Carrot::incrementCurrentWayPointID()
{
	this->currentWayPointID += 1;
}

void Carrot::setCurrentWayPointID(int id)
{
	this->currentWayPointID = id;
}

void Carrot::setCarrotRabbitPosition(float distance, float direction)
{
    this->carrotPos.carrotDirection = direction;
    this->carrotPos.carrotDistance = distance;
}


/** modifier functions **/
void Carrot::moveCarrot(const ros::TimerEvent& event)
{
    float crDirection;
    if(getState() == ReachedEnd)
    {
    	return;
    }
    else if(getState() == ReachedWaypoint)
    {
		float crDistance=getEuclideanDistance(getRabbitLocation(), getCarrotLocation());
    	if(crDistance<1)
    		setState(MovingOnLine);
        else
        {
            crDirection = atan2(getCarrotLocation().z - getRabbitLocation().z, getCarrotLocation().x - getRabbitLocation().x);
            setCarrotRabbitPosition(crDistance, crDirection);
        }
    }
    else if(getState() == MovingOnLine)
    {
    	Position intersectionPoint;
    	float dist,dist1;
    	getLineIntersection(getWayPoint(currentWayPointID), getWayPoint(currentWayPointID-1), getRabbitLocation(), intersectionPoint);
    	float carrotHeadingAngle = atan2(getWayPoint(currentWayPointID).z-getWayPoint(currentWayPointID-1).z,getWayPoint(currentWayPointID).x-getWayPoint(currentWayPointID-1).x);
    	Position temp;
    	temp.x = intersectionPoint.x + ((MaximumDistanceFromRabbit) * cos(carrotHeadingAngle));
    	temp.z = intersectionPoint.z + ((MaximumDistanceFromRabbit) * sin(carrotHeadingAngle));
    	dist  = getEuclideanDistance(temp, getWayPoint(currentWayPointID-1));
    	dist1 = getEuclideanDistance(getWayPoint(currentWayPointID-1), getWayPoint(currentWayPointID));
    	if (dist > dist1)
    	{
    	    carrot=getWayPoint(currentWayPointID);
    	}
    	crDirection = atan2(getCarrotLocation().z - getRabbitLocation().z, getCarrotLocation().x - getRabbitLocation().x);
    	float crDistance=getEuclideanDistance(getRabbitLocation(), getCarrotLocation());
        setCarrotRabbitPosition(crDistance, crDirection);
    }
    publishCarrotPosition();
}

void Carrot::publishCarrotPosition()
{
    this->publisher_carrot_robot.publish(carrotPos);
}

void Carrot::callbackUpdateRabbitLocation(const std_msgs::String::ConstPtr& rabbit)
{
    vector<string> temp = split(rabbit->data,',');
    this->rabbit.x = atof(temp[0].c_str());;
    this->rabbit.y = atof(temp[1].c_str());;
    this->rabbit.z = atof(temp[2].c_str());;
}

void Carrot::readWayPointsFromFile(string filename)
{
    if(filename[0] == '\0') return;
    Position  position;
    this->wayPointPath.clear();
    string line;
    ifstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            vector<string> temp = split(line,',');
            position.x = atof(temp[0].c_str());
            position.y = atof(temp[1].c_str());
            position.z = atof(temp[1].c_str());
            this->wayPointPath.push_back(position);
        }
        myfile.close();
    }
    else
    {
        ROS_INFO("Unable to read File");
        exit(1);
    }
}


void Carrot::getLineIntersection(Position linePointOne, Position linePointTwo, Position robotLocation, Position &intersectionPoint)
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
