#include "rabbit_follow/Bug.h"
int Bug::bugIDCounter = 0;

Bug::Bug(State state, float x, float y, float dist, int lastPTID, int boundID)
{
	this->state = state;
	this->x = x;
	this->y = y;
    this->ID = Bug::bugIDCounter++;
    this->lastX = -1;
    this->lastY = -1;
    this->lastPointID = lastPTID;
    this->boundaryID = dist;
    this->distance = dist;
}

Bug::~Bug()
{
    //dtor
}

/**
    Set Functions
*/

void Bug::setState(State state)
{
	this->state = state;
}

void Bug::setXY(float i_x, float i_y)
{
    this->x = x;
    this->y = y;
    //if(i_x == )
}

void Bug::setX(float x)
{
    this->lastX = this->x;
    this->x = x;
}

void Bug::setY(float y)
{
    this->lastY = this->y;
    this->y = y;
}

void Bug::setLastX(float x)
{
    this->lastX = x;
}

void Bug::setLastY(float y)
{
    this->lastY = y;
}

void Bug::setLastPointID(int Id)
{
    this->lastPointID = Id;
}

void Bug::setBoundaryID(int boundaryId)
{
    this->boundaryID = boundaryId;
}

void Bug::setDistance(float dist)
{
        this->distance = dist;
}

void Bug::addDistance(float dist)
{
    this->distance += dist;
}


/**
    Get Functions
*/

State Bug::getState()
{
    return this->state;
}

int Bug::getID()
{
    return this->ID;
}

float Bug::getX()
{
    return this->x;
}

float Bug::getY()
{
    return this->y;
}

float Bug::getLastX()
{
    return this->lastX;
}

float Bug::getLastY()
{
    return this->lastY;
}

int Bug::getBoundaryID()
{
    return this->boundaryID;
}

float Bug::getDistance()
{
    return this->distance;
}

int Bug::getLastPointID()
{
    return this->lastPointID;
}

/**
    Bug Functions
*/

bool Bug::checkInsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, float goalX, float goalY)
{
    float theta = atan2(goalY - this->getY(),goalX - this->getX());
    float newX = this->getX() + ( 0.5 * cos(theta));
    float newY = this->getY() + ( 0.5 * sin(theta));
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        AB = (pow(obstArray[i][0].x - obstArray[i][1].x,2) + pow(obstArray[i][0].y - obstArray[i][1].y,2));
        AD = (pow(obstArray[i][0].x - obstArray[i][3].x,2) + pow(obstArray[i][0].y - obstArray[i][3].y,2));
        AMAB = (((newX - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + ((newY - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
        AMAD = (((newX - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + ((newY - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));
         //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return true;
        }
    }
    return false;
}

int Bug::checkIfOnOtherLines(vector < obstacleLine > &obstacleLines, int &index)
{
    for(int i=0; i< obstacleLines.size(); i++)
    {
        if(i == this->boundaryID)
        {
            continue;
        }
        if(obstacleLines[i].point[0].x == this->getX() && obstacleLines[i].point[0].y == this->getY())
        {
            index = 1;
            return i;
        }
        if(obstacleLines[i].point[1].x == this->getX() && obstacleLines[i].point[1].y == this->getY())
        {
         //   ROS_INFO("check other lines, %d || %d", 0, i);
            index = 0;
            return i;
        }
    }
    index = 0;
    return -1;
}

vector<intersectingPoint> Bug::getLineIntersections(float goalX, float goalY, vector < obstacleLine > &obstacleLines)
{
    float p0_x = this->getX();
    float p0_y = this->getY();
    float p1_x = goalX;
    float p1_y = goalY;

    float p2_x;
    float p2_y;
    float p3_x;
    float p3_y;

    float i_x,i_y;
    vector<intersectingPoint> intersectingLinesID;

    intersectingPoint iPoint;

    for(int i=0; i< obstacleLines.size();i++)
    {
        p2_x = obstacleLines[i].point[0].x;
        p2_y = obstacleLines[i].point[0].y;
        p3_x = obstacleLines[i].point[1].x;
        p3_y = obstacleLines[i].point[1].y;
        float s1_x, s1_y, s2_x, s2_y;
        s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
        s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

        float s, t;
        s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
        t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            // Collision detected
            i_x = p0_x + (t * s1_x);
            i_y = p0_y + (t * s1_y);

            if(i_x == p0_x && i_y == p0_y) continue;

            iPoint.ID = i;
            iPoint.x = i_x;
            iPoint.y = i_y;
            intersectingLinesID.push_back(iPoint);
        }
    }
    return intersectingLinesID; // No collision
}

float Bug::getEuclideanDistance(float p0_x, float p0_y, float p1_x, float p1_y)
{
    return sqrt(pow(p1_x - p0_x,2) + pow(p1_y - p0_y,2));
}
