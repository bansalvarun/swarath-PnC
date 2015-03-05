#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>

int currentD=0;

bool moveCarrotFlag = true;

visualization_msgs::Marker make_line_one (visualization_msgs::Marker line_strip)
{
            geometry_msgs::Point p;
            p.x = -7;
            p.y = -14;
            line_strip.points.push_back(p);
            p.x = -7;
            p.y = 16;
            line_strip.points.push_back(p);
            p.x = 8;
            p.y = 16;
            line_strip.points.push_back(p);
            p.x = 8;
            p.y = 46;
            line_strip.points.push_back(p);
           // marker_pub.publish(line_strip);
            return line_strip;
}

visualization_msgs::Marker moveCarrot(visualization_msgs::Marker circle, visualization_msgs::Marker line_strip1)
{
    float dist;


    dist=pow(pow(line_strip1.points[currentD].x-circle.pose.position.x,2)+pow(line_strip1.points[currentD].y-circle.pose.position.y,2),0.5);
    if(dist<1)
    {
        currentD++;
        if(currentD>=4)
        {
            moveCarrotFlag = false;
            return circle;
        }
    }
    double theta = atan2(line_strip1.points[currentD].y-circle.pose.position.y,line_strip1.points[currentD].x-circle.pose.position.x);
    circle.pose.position.x+= 1*cos(theta);
    circle.pose.position.y+= 1*sin(theta);
    return circle;
}

visualization_msgs::Marker moveRobot(visualization_msgs::Marker robot, visualization_msgs::Marker circle)
{
    float dist=pow(pow(robot.pose.position.x-circle.pose.position.x,2)+pow(robot.pose.position.y-circle.pose.position.y,2),0.5);
    if(dist<1)
    {
       exit(1);
    }
    double theta = atan2(circle.pose.position.y-robot.pose.position.y,circle.pose.position.x - robot.pose.position.x);
    robot.pose.position.x+= 0.8*cos(theta);
    robot.pose.position.y+= 0.8*sin(theta);
    return robot;
}

int main (int argc,char** argv)
{
    ros::init(argc,argv,"path");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("path",10);
    visualization_msgs::Marker line_strip1,circle,robot;
    ros::Rate r(100);

    line_strip1.header.frame_id=circle.header.frame_id=robot.header.frame_id= "/my_path";
    line_strip1.header.stamp=circle.header.stamp=robot.header.stamp= ros::Time::now();
    line_strip1.ns=circle.ns= robot.ns="path";
    line_strip1.id =1;
    circle.id=2;
    robot.id=3;
    line_strip1.action=circle.action= robot.action= visualization_msgs::Marker::ADD;
    line_strip1.pose.orientation.w=circle.pose.orientation.w= 1.0;

    line_strip1.type= visualization_msgs::Marker::LINE_STRIP;
    circle.type= visualization_msgs::Marker::SPHERE;
    robot.type= visualization_msgs::Marker::SPHERE;

    circle.scale.x=1.0;
    circle.scale.y=1.0;
    circle.scale.z=1.0;


    robot.scale.x=1.0;
    robot.scale.y=1.0;
    robot.scale.z=1.0;

    line_strip1.scale.x= 1;
    line_strip1.scale.y= 1;

    circle.color.r = 0.0f;
    circle.color.g = 1.0f;
    circle.color.b = 0.0f;
    circle.color.a = 1.0;

    robot.color.g = 1.0f;
    robot.color.a = 1.0;

    line_strip1.color.b=1.0;
    line_strip1.color.a=1.0;

    robot.pose.position.x = -15;
    robot.pose.position.y = -18;

    circle.pose.position.x = -7;
    circle.pose.position.y = -14;
    circle.pose.position.z = 0.0;

    line_strip1 = make_line_one(line_strip1);
    marker_pub.publish(line_strip1);
   // marker_pub.publish(circle);
    //std::cout<<line_strip1.points.size();

    ros::Duration(3).sleep();

    while (ros::ok())
    {
        if(moveCarrotFlag)
            circle = moveCarrot(circle,line_strip1);
        robot = moveRobot(robot,circle);
        marker_pub.publish(circle);
        marker_pub.publish(robot);
        marker_pub.publish(line_strip1);
        ros::Duration(0.2).sleep();
    }

}
