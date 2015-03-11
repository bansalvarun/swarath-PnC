/******************************************************
* Carrot description file for rabbit follow algorithm  *
* Written for SWARATH Project                         *
* @author Nishant Sharma                              *
* @version 1.0                                        *
* @date 11, March, 2015                                *
******************************************************/

#ifndef CARROT_H
#define CARROT_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include "rabbit_follow/GlobalDeclaration.h"
#include <rabbit_follow/carrotPosition.h>
#include <vector>

using namespace std;

enum CarrotState{MovingOnLine, ReachedWaypoint, ReachedEnd};

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
        int getCurrentWayPointID();

        /** set functions **/
        void setCarrotLocation(Position carrot);
        void setRabbitLocation(Position rabbit);
        void setState(CarrotState state);
        void incrementCurrentWayPointID();
        void setCurrentWayPointID(int id);
        void setCarrotRabbitPosition(float distance, float direction);

        /** modifier functions **/
        void moveCarrot(const ros::TimerEvent& event);
        void publishCarrotPosition();

        void callbackUpdateRabbitLocation(const std_msgs::String::ConstPtr& rabbit);

        void readWayPointsFromFile(string filename);

        ros::Publisher publisher_carrot_robot;

    protected:
    private:

        vector <Position> wayPointPath;

        Position carrot;
        Position rabbit;

        rabbit_follow::carrotPosition carrotPos;

        CarrotState state;

        int currentWayPointID;

        float getEuclideanDistance(Position one, Position two);
        void getLineIntersection(Position linePointOne, Position linePointTwo, Position robotLocation, Position &intersectionPoint);
};

#endif // CARROT_H
