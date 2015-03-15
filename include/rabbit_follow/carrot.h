/******************************************************
* Carrot description file for rabbit follow algorithm *
* Written for SWARATH Project                         *
* @author Nishant Sharma                              *
* @version 1.1                                        *
* @date 13, March, 2015                               *
******************************************************/

#ifndef CARROT_H
#define CARROT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <rabbit_follow/GlobalDeclaration.h>
#include <rabbit_follow/carrotPosition.h>
#include <vector>

#define debugCarrot

using namespace std;

enum CarrotState{MovingOnLine, ReachedWaypoint, ReachedEndDestination};

class Carrot
{
    public:

        /** Default constructor */
        Carrot(char* filename);

        /** Default destructor */
        virtual ~Carrot();

        /** modifier functions **/

        /** Main algorithm for maving carrot on the waypoint course**/
        void MoveCarrot(const ros::TimerEvent& event);

        /** ros callback function for updating rabbit location**/
        void CallbackUpdateRabbitLocation(const std_msgs::String::ConstPtr& rabbitLocation);

        /** "publisher_carrot_robot" -> left public for ROS instantiation **/
        ros::Publisher publisher_carrot_robot;

        ros::Publisher publisher_carrot_location_rviz;

        /** initialize carrot rviz marker**/
        void InitializeMarker();

        const static int minAllowedDistanceCarrotToRabbit = 1;
    protected:
    private:
        visualization_msgs::Marker carrotMarker;
        visualization_msgs::Marker wayPointMarker;

        /** member for storing the waypoint path to be followed **/
        vector <Position> wayPointPath;

        /** member for storing the carrot location **/
        Position carrot;

        /** member for storing the rabbit's current location **/
        Position rabbit;

        /** member for storing the carrot's distance and direction from the rabbit **/
        rabbit_follow::carrotPosition carrotPosition;

        /** member for storing the current state of the carrot **/
        CarrotState carrotState;

        /** member for storing the current waypoint that's is being followed **/
        int currentWayPointID;

        /** get functions **/

        /** returns one waypoint form the waypoint path **/
        Position GetWayPoint(int index);

        /** return index of the current waypoint **/
        int GetCurrentWayPointID();

        /** set functions **/

        /** increments the followed waypoint index to the next waypoint **/
        void IncrementCurrentWayPointID();

        /** to reset the current way point index to a specific value **/
        void SetCurrentWayPointID(int index);

        /** to set the carrot's distance and direction from the robot **/
        void SetCarrotRabbitPosition(float distance, float direction);

        /** modifier functions **/

        /** carrot behaviour when on a waypoint **/
        void UpdateCarrotWhenReachedWaypoint();

        /** carrot behaviour when on a line **/
        void UpdateCarrotWhenMovingOnLine();

        /** function for publishing carrot distance and direction to the rabbit node**/
        void PublishCarrotPosition();

        /** Read WayPoints information from a File **/
        void ReadWayPointsFromFile(char* filename);

        /** function with calculates the perpendicular intersection point from the current followed line **/
        void GetPerpendicularLineIntersection(Position linePointOne, Position linePointTwo, Position robotLocation, Position &intersectionPoint);
};

#endif // CARROT_H
