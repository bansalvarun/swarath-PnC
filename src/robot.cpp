/***********************************************
* Robot main file for rabbit follow algorithm  *
* Written for SWARATH Project                  *
* @author Nishant Sharma                       *
* @version 0.0                                 *
* @date 4, March, 2015                         *
***********************************************/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rabbit_follow/robot.h>
#include <cmath>
#include <vector>

int currentD=0;

/** callback function in robotNew.cpp **/

void initializeMarkers(visualization_msgs::Marker &robot)
{
    robot.header.frame_id  = "/rabbit_follow";
    robot.header.stamp     = ros::Time::now();
    robot.ns               = "rabbit_follow_robot";
    robot.id               = 0;
    robot.action           = visualization_msgs::Marker::ADD;
    robot.type             = visualization_msgs::Marker::SPHERE;

    robot.scale.x=1.0;
    robot.scale.y=1.0;
    robot.scale.z=1.0;

    robot.color.g = 1.0f;
    robot.color.a = 1.0;
}

int main (int argc,char** argv)
{
    ros::init(argc,argv,"rabbit_follow_robot");
    ros::NodeHandle n;

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("rabbit_follow",10);

    visualization_msgs::Marker robot;


    while (ros::ok())
    {
        ros::Spin();
    }

}
