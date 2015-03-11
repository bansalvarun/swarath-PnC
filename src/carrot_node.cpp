#include <ros/ros.h>
#include "rabbit_follow/carrotPosition.msg"
#include <cmath>
#include <vector>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"rabbit_follow_carrot");
    ros::NodeHandle n;
    Carrot carrot;

    carrot.publisher_carrot_robot = n.advertise<rabbit_follow::carrotPosition>("carrot_location_update",10);
    ros::Subscriber sub = n.subscribe("gps_unity", 1000, &Rabbit::callbackUpdateRabbitLocation, &carrot);

    ros::Timer timer = n.createTimer(ros::Duration(0.1),&Carrot::moveCarrot,$carrot);

    ros::Spin();
}
