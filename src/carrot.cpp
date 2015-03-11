#include <rabbit_follow/carrot.h>
#include <ros/ros.h>
#include <fstream>

/** Default constructor */
Carrot::Carrot(string filename)
{
    this->readWayPointsFromFile(filename);
    this->carrot.x = this->getWayPoint(0).x;
    this->carrot.y = this->getWayPoint(0).y;
    this->rabbitLocationUpdate = false;
    this->currentWayPointID=0;
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

float Carrot::getHeadingAngle()
{
    return this->carrotHeadingAngle;
}

Position Carrot::getWayPoint(int id)
{
    return this->wayPointPath[id];
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

void Carrot::setHeadingAngle(float radian)
{
    this->carrotHeadingAngle = radian;
}

void Carrot::setState(CarrotState state)
{
    this->state = state;
}

void rabbitLocationUpdated(bool state);

/** modifier functions **/
void Carrot::changeHeadingAngle(float radian)
{
    this->carrotHeadingAngle += radian;
}


void Carrot::moveCarrot()
{
    getLineIntersection()
}

void Carrot::updateState()
{

}

void Carrot::callbackUpdateRabbitLocation(const geometry_msgs::Point::ConstPtr& rabbitMsg)
{
    this->rabbit.x = rabbitMsg->x;
    this->rabbit.y = rabbitMsg->y;
    this->rabbit.z = rabbitMsg->z;
    this->rabbitLocationUpdate = true;
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
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

float Carrot::getEuclideanDistance(Position one, Position two)
{
    return sqrt( pow( one.x - two.x ,2) + pow( one.y - two.y  ,2) );
}

void Carrot::getLineIntersection(Position linePointOne, Position linePointTwo, Position robotLocation, Position &intersectionPoint)
{
    // first convert line to normalized unit vector
    double dx = linePointTwo.x - linePointOne.x;
    double dy = linePointTwo.y - linePointOne.y;
    double mag = sqrt(dx*dx + dy*dy);
    dx /= mag;
    dy /= mag;

    // translate the point and get the dot product
    double lambda = (dx * (robotLocation.x - linePointOne.x)) + (dy * (robotLocation.y - linePointOne.y));
    intersectionPoint.x = (dx * lambda) + linePointOne.x;
    intersectionPoint.y = (dy * lambda) + linePointOne.y;
}
