///***********************************************
//* Robot main file for rabbit follow algorithm  *
//* Written for SWARATH Project                  *
//* @author Nishant Sharma                       *
//* @version 0.0                                 *
//* @date 4, March, 2015                         *
//***********************************************/
//
//#include <ros/ros.h>
//#include <visualization_msgs/Marker.h>
//#include <rabbit_follow/robot.h>
//#include <cmath>
//#include <vector>
//
//int currentD=0;
//
///** callback function in robotNew.cpp **/
//
//void initializeMarkers(visualization_msgs::Marker &robot)
//{
//    robot.header.frame_id  = "/rabbit_follow";
//    robot.header.stamp     = ros::Time::now();
//    robot.ns               = "rabbit_follow_robot";
//    robot.id               = 0;
//    robot.action           = visualization_msgs::Marker::ADD;
//    robot.type             = visualization_msgs::Marker::SPHERE;
//
//    robot.scale.x=1.0;
//    robot.scale.y=1.0;
//    robot.scale.z=1.0;
//
//    robot.color.g = 1.0f;
//    robot.color.a = 1.0;
//}
//
//int main (int argc,char** argv)
//{
//    ros::init(argc,argv,"rabbit_follow_robot");
//    ros::NodeHandle n;
//
//    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("rabbit_follow",10);
//
//    visualization_msgs::Marker robot;
//
//
//    while (ros::ok())
//    {
//        ros::Spin();
//    }
//
//}



#include <ros/ros.h>
#include <rabbit_follow/location.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>

int currentD=0;
bool moveCarrotFlag = true;

rabbit_follow::location circle1;

void callback(const rabbit_follow::location::constPtr &msg)
{
	circle1.x = msg->x;
    circle1.y = msg->y;
    circle1.z = msg->z;
}

visualization_msgs::Marker moveRobot(visualization_msgs::Marker robot)
{

	float dist=pow(pow(robot.pose.position.x-circle1.x,2)+pow(robot.pose.position.y-circle1.y,2),0.5);
    if(dist<1)
    {
       exit(1);
    }
    double theta = atan2(circle1.y-robot.pose.position.y,circle1.x - robot.pose.position.x);
    robot.pose.position.x+= 0.8*cos(theta);
    robot.pose.position.y+= 0.8*sin(theta);
    return robot;
}

int main (int argc,char** argv)
{
    ros::init(argc,argv,"robot");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("carloc",10,callback);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("path",10);
    ros::Publisher loc_pub = n.advertise<rabbit_follow::location>("rabloc",10);

    ros::Publisher carloc_pub = n.advertise<rabbit_follow::location>("carrabloc",10);

    visualization_msgs::Marker robot;
    rabbit_follow::location robot1;
    ros::Rate r(100);

	robot.header.frame_id= "/my_path";
    robot.header.stamp= ros::Time::now();
    robot.ns="path";
    robot.id=3;
    robot.action= visualization_msgs::Marker::ADD;
    robot.type= visualization_msgs::Marker::SPHERE;

    robot.scale.x=1.0;
    robot.scale.y=1.0;
    robot.scale.z=1.0;

    robot.color.g = 1.0f;
    robot.color.a = 1.0;

    robot.pose.position.x = -15;
    robot.pose.position.y = -18;

    robot1.x = -15;
    robot1.y = -18;


    while (ros::ok())
    {
        robot = moveRobot(robot);
        robot1.x = robot.pose.position.x;
        robot1.y = robot.pose.position.y;
        marker_pub.publish(robot);
        loc_pub.publish(robot1);
        carloc_pub.publish(circle1);
	    ros::Duration(.2).sleep();
    }



}
