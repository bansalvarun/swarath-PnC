/******************************************************
* Rabbit description file for rabbit follow algorithm *
* Written for SWARATH Project                         *
* @author Nishant Sharma                              *
* @version 1.1                                        *
* @date 13, March, 2015                               *
******************************************************/

#ifndef Rabbit_H
#define Rabbit_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rabbit_follow/GlobalDeclaration.h>
#include <rabbit_follow/carrotPosition.h>
#include <visualization_msgs/Marker.h>
#include <vector>

#define debugRabbit

using namespace std;

enum RabbitState{FollowingCarrot, ReachedEnd};

class Rabbit
{
    public:
        /** Default constructor */
        Rabbit();

        /** Default destructor */
        virtual ~Rabbit();

        /** function implementing rabbit moving **/
        void MoveRabbit(const ros::TimerEvent& event);

        /** ros callback function for updating rabbit location**/
        void CallbackUpdateRabbitGPSLocation(const std_msgs::String::ConstPtr& rabbit);

        /** ros callback function for updating rabbit heading angle**/
        void CallbackUpdateRabbitIMULocation(const std_msgs::String::ConstPtr& rabbit);

        /** ros callback function for updating carrot location**/
        void CallbackUpdateCarrotLocation(const rabbit_follow::carrotPosition::ConstPtr& carrot);

        /** ros publisher for publishing steering value to unity **/
        ros::Publisher steer_publisher;

        /** ros publisher for publishing throttle value to unity **/
        ros::Publisher throttle_publisher;
        ros::Publisher publisher_rabbit_location_rviz;

        /** initialize carrot rviz marker**/
        void InitializeMarker();
    protected:
    private:
        visualization_msgs::Marker rabbitMarker;
        visualization_msgs::Marker rabbitCurrentHeadingMarker;
        visualization_msgs::Marker rabbitDesiredHeadingMarker;

        /** rabbit location holder member **/
        Position rabbit;
        Position carrot;
        vector <Position> wayPointPath;
        int currentWayPointID;
        void ReadWayPointsFromFile(string filename);

        /** rabbit's current heading member **/
        float rabbitCurrentHeading;

        /** member to store carrot's distance and direction from robot **/
        rabbit_follow::carrotPosition carrotPosition;

        /** member to store required steering value by car in unity **/
        float steering;

        /** member to store required throttle value by car in unity **/
        float throttle;

        /** member to store current velocity of the rabbit**/
        float currentVelocity;



        /** member to store last Velocity update time **/
        ros::Time lastVelocityUpdateTime;

        /** member to store last rabbit's last location at Velocity update time **/
        Position rabbitLastLocation;

        /** member to store current state of the rabbit **/
        RabbitState rabbitState;


        /** modifier functions **/

        /** function to change rabbit's steering value so that it can turn properly in unity**/
        void UpdateSteering();

        /** function to change rabbit's throttle value so that it can move properly in unity**/
        void UpdateThrottle();

        /** function to calculate current velocity of the rabbit **/
        void CalculateVelocity(Position position);

        /** function to publish steering value to unity **/
        void PublishSteering();

        /** function to publish throttle value to unity **/
        void PublishThrottle();
        Position GetWayPoint(int index);
};

#endif // Rabbit_H
