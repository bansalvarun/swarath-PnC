#include <ros/ros.h>
#include "rabbit_follow/carrotPosition.msg"
#include <cmath>
#include <vector>

int currentD=0;

bool moveCarrotFlag = true;





int main (int argc,char** argv)
{
    ros::init(argc,argv,"rabbit_follow_carrot");
    ros::NodeHandle n;
    Carrot carrot;
    Rabbit rabbit;
    ros::Publisher steer_pub = n.advertise<rabbit_follow::carrotPosition>("carrot_location_update",10);
    ros::Subscriber sub = n.subscribe("gps_unity", 1000, &Rabbit::callbackUpdateRabbitLocation, &rabbit);
    ros::Timer timer = n.createTimer(ros::Duration(0.1),&Carrot::moveCarrot,$carrot);
    //ros::Rate r(100);

    

    //ros::Duration(3).sleep();

    while (ros::ok())
    {
       ros::spinOnce();
    }

}
