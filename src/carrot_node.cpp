/*********************************************************
* Carrot node file for rabbit follow algorithm *
* Written for SWARATH Project                            *
* @author Nishant Sharma                                 *
* @version 1.0                                           *
* @date 13, March, 2015                                  *
*********************************************************/

#include <ros/ros.h>
#include <rabbit_follow/carrotPosition.h>
#include <rabbit_follow/carrot.h>
#include <cmath>
#include <vector>
#include <string>

using namespace std;

int main (int passedArgumentCount,char** passedArgumentValues)
{
    if(passedArgumentCount != 2)// Check the value of passedArgumentCount. if filename is not passed
    {
        std::cout << "usage -> rosrun rabbit_follow carrot_node <filename>\n"; // Inform the user of how to use the program
        exit(0);
    }

    //passing empty remapping
    ros::M_string remappings;

    //initializing ros node
    ros::init(remappings,"rabbit_follow_carrot");

    //creating node handle for ros node
    ros::NodeHandle nodeHandle;

    //initializing carrot object with filename to read waypoint information from
    Carrot carrot(passedArgumentValues[1]);

    //initializing publisher of carrot to rabbit distance and direction
    carrot.publisher_carrot_robot = nodeHandle.advertise<rabbit_follow::carrotPosition>("carrot_location_update",10);
    carrot.publisher_carrot_location_rviz = nodeHandle.advertise<visualization_msgs::Marker>("carrot_location_rviz_update",10);

    //initializing subscriber for updating rabbit location
    ros::Subscriber subscriber = nodeHandle.subscribe("gps_unity", 1000, &Carrot::CallbackUpdateRabbitLocation, &carrot);

	//sleep to let code register punlisher and subscribers
	ros::Duration(1).sleep();

    //creating rostimer to call Movecarrot every 100 millisecond
    ros::Timer timer = nodeHandle.createTimer(ros::Duration(0.067),&Carrot::MoveCarrot, &carrot);

    carrot.InitializeMarker();

    //starting spin to start processing incoming messages
    ros::spin();
}
