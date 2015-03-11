/***********************************************
* rabbit main file for rabbit follow algorithm  *
* Written for SWARATH Project                  *
* @author Nishant Sharma                       *
* @version 0.0                                 *
* @date 4, March, 2015                         *
***********************************************/
#include <ros/ros.h>
#include <rabbit_follow/rabbit.h>
#include <rabbit_follow/carrotPosition.h>
#include <std_msgs/String.h>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"rabbit_follow_rabbit");
    ros::NodeHandle n;

    Rabbit rabbit;

    rabbit.steer_publisher = n.advertise<std_msgs::String>("steering_unity",10);
    rabbit.throttle_publisher = n.advertise<std_msgs::String>("throttle_unity",10);

    ros::Subscriber sub1 = n.subscribe("carrot_location_update", 1000, &Rabbit::callbackUpdateCarrotLocation, &rabbit);
    ros::Subscriber sub2 = n.subscribe("gps_unity", 1000, &Rabbit::callbackUpdateRabbitGPSLocation, &rabbit);
    ros::Subscriber sub3 = n.subscribe("imu_unity", 1000, &Rabbit::callbackUpdateRabbitIMULocation, &rabbit);

    ros::Timer timer = n.createTimer(ros::Duration(0.1), &Rabbit::moveRabbit, &rabbit);
    ros::spin();
}
