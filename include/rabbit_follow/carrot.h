/******************************************************
* Carrot description file for rabbit follow algorithm  *
* Written for SWARATH Project                         *
* @author Nishant Sharma                              *
* @version 1.0                                        *
* @date 11, March, 2015                                *
******************************************************/

#ifndef CARROT_H
#define CARROT_H

#include <geometry_msgs/Point.h>
#include <vector>

using namespace std;

enum CarrotState{MovingOnLine, ReachedWaypoint, ReachedEnd};

struct Position
{
    float x;
    float y;
    float z;
};

class Carrot
{
    public:
        const static float MaximumDistanceFromRabbit = 5;

        /** Default constructor */
        Carrot(string filename);

        /** Default destructor */
        virtual ~Carrot();

        /** get functions **/
        Position getCarrotLocation();
        Position getRabbitLocation();
        CarrotState getState();
        Position getWayPoint(int id);
        bool isRabbitLocationUpdated();
        int getCurrentWayPointID();

        /** set functions **/
        void setCarrotLocation(Position carrot);
        void setRabbitLocation(Position rabbit);
        void setState(CarrotState state);
        void rabbitLocationUpdated(bool state);
        void incrementCurrentWayPointID();
        void setCurrentWayPointID(int id);

        /** modifier functions **/
        void moveCarrot();
        void updateState();

        void callbackUpdateRabbitLocation(const geometry_msgs::Point::ConstPtr& rabbit);

        void readWayPointsFromFile(string filename);

    protected:
    private:

        vector <Position> wayPointPath;
        bool rabbitLocationUpdate;
        Position carrot;
        Position rabbit;
        float carrotHeadingAngle;
        CarrotState state;
        int currentWayPointID;
        float getEuclideanDistance(Position one, Position two);
        void getLineIntersection(Position linePointOne, Position linePointTwo, Position robotLocation, Position &intersectionPoint);
};

//float Carrot::MaximumDistanceFromRabbit = 5;

#endif // CARROT_H
