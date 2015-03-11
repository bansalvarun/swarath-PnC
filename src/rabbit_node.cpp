/***********************************************
* rabbit main file for rabbit follow algorithm  *
* Written for SWARATH Project                  *
* @author Nishant Sharma                       *
* @version 0.0                                 *
* @date 4, March, 2015                         *
***********************************************/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rabbit_follow/rabbit.h>
#include <std_msgs/String.h>
#include <cmath>
#include <vector>

int currentD=0;

/** callback function in rabbitNew.cpp **/

void initializeMarkers(visualization_msgs::Marker &rabbit)
{
    rabbit.header.frame_id  = "/rabbit_follow";
    rabbit.header.stamp     = ros::Time::now();
    rabbit.ns               = "rabbit_follow_rabbit";
    rabbit.id               = 0;
    rabbit.action           = visualization_msgs::Marker::ADD;
    rabbit.type             = visualization_msgs::Marker::SPHERE;

    rabbit.scale.x=1.0;
    rabbit.scale.y=1.0;
    rabbit.scale.z=1.0;

    rabbit.color.g = 1.0f;
    rabbit.color.a = 1.0;
}

int main (int argc,char** argv)
{
    ros::init(argc,argv,"rabbit_follow_rabbit");
    ros::NodeHandle n;

    Rabbit rabbit;

    ros::Publisher steer_pub = n.advertise<visualization_msgs::Marker>("steering_unity",10);
    ros::Publisher throttle_pub = n.advertise<visualization_msgs::Marker>("throttle_unity",10);

    ros::Subscriber sub = n.subscribe("carrot_location_update", 1000, &Rabbit::callbackUpdateRabbitLocation, &rabbit);
    ros::Subscriber sub = n.subscribe("gps_unity", 1000, &Rabbit::callbackUpdateRabbitLocation, &rabbit);
    ros::Subscriber sub = n.subscribe("imu_unity", 1000, &Rabbit::callbackUpdateRabbitLocation, &rabbit);

    visualization_msgs::Marker rabbitMarker;
    initializeMarkers(rabbitMarker);

    while (ros::ok())
    {
        ros::spinOnce();
        rabbit.moveRabbit();
        steer_pub.publish(to_string(rabbit.getDirection()));

        //
        //


    }

}
