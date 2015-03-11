#include <ros/ros.h>
#include "rabbit_follow/carrotPosition.h"
#include "rabbit_follow/carrot.h"
#include <cmath>
#include <vector>
#include <string>

using namespace std;

int main (int argc,char** argv)
{
    ros::init(argc,argv,"rabbit_follow_carrot");
    ros::NodeHandle n;
    string filename;
	cin>> filename;
    Carrot carrot(filename);

    carrot.publisher_carrot_robot = n.advertise<rabbit_follow::carrotPosition>("carrot_location_update",10);
    ros::Subscriber sub = n.subscribe("gps_unity", 1000, &Carrot::callbackUpdateRabbitLocation, &carrot);
	ros::Duration(1).sleep();
    ros::Timer timer = n.createTimer(ros::Duration(0.1),&Carrot::moveCarrot, &carrot);

    ros::spin();
}
