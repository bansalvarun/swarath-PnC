/***********************************************
* rabbit main file for rabbit follow algorithm *
* Written for SWARATH Project                  *
* @author Nishant Sharma                       *
* @version 1.0                                 *
* @date 13, March, 2015                        *
***********************************************/

#include <ros/ros.h>
#include <rabbit_follow/rabbit.h>
#include <rabbit_follow/carrotPosition.h>
#include <std_msgs/String.h>

int main (int argc,char** argv)
{
    //initializing ros node
    ros::init(argc,argv,"rabbit_follow_rabbit");

    //creating node handle for ros node
    ros::NodeHandle n;

    //creating rabbit object
    Rabbit rabbit;

    //initializing publisher of carrot to rabbit distance and direction
    rabbit.steer_publisher = n.advertise<std_msgs::String>("steering_unity",10);

    //initializing publisher of carrot to rabbit distance and direction
    rabbit.throttle_publisher = n.advertise<std_msgs::String>("throttle_unity",10);

    //initializing subscriber for updating carrot location
    ros::Subscriber carrotLocationSubscriber = n.subscribe("carrot_location_update", 1000, &Rabbit::CallbackUpdateCarrotLocation, &rabbit);

    //initializing subscriber for updating rabbit location
    ros::Subscriber rabbitLocationSubscriber = n.subscribe("gps_unity", 1000, &Rabbit::CallbackUpdateRabbitGPSLocation, &rabbit);

    //initializing subscriber for updating rabbit heading
    ros::Subscriber rabbitheadingSubscriber = n.subscribe("imu_unity", 1000, &Rabbit::CallbackUpdateRabbitIMULocation, &rabbit);

    //sleep to let code register punlisher and subscribers
	ros::Duration(1).sleep();

    //creating rostimer to call Move Rabbit every 100 millisecond
    ros::Timer timer = n.createTimer(ros::Duration(0.1), &Rabbit::MoveRabbit, &rabbit);

    //starting spin to start processing incoming messages
    ros::spin();
}
