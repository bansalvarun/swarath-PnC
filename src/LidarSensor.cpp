#include "rabbit_follow/LidarSensor.h"


LidarSensor::LidarSensor()
{
    //ctor
}

LidarSensor::~LidarSensor()
{
    //dtor
}

void LidarSensor::initializeLidarSensorData(int increment, int range)
{
    LidarSensor::increment = increment;
    LidarSensor::range = range;
}

void LidarSensor::callbackUpdateLidarData(const rabbit_follow::lidarData::ConstPtr& lidarSensor)
{
    int lidarSensorIndex = 0;
    for(int i =0 ;i < lidarReadingsRange;i++)
    {
        for(int j=0;j < lidarReadingsRange;j++)
        {
            LidarSensor::sensorData[i][j] = lidarSensor->sensorData[lidarSensorIndex++];
        }
    }
    LidarSensor::sensorDataUpdated = true;
    publishDataToRviz();
    //ROS_INFO("Lidar Sensor Updated");
}

//bool LidarSensor::isObstacleInFront(vector<> )
//{
//
//
//}

void LidarSensor::setRvizMarkerData(Position rabbit, float addedHeading)
{
    rvizData.points.clear();
    geometry_msgs::Point point;
    float angleZ = (-1 * M_PI/3), angleY;
    for(int i =0 ;i < lidarReadingsRange;i++)
    {
        angleY = -1 * M_PI/3;
        for(int j=0;j < lidarReadingsRange;j++)
        {
            point.z = rabbit.y + (LidarSensor::sensorData[i][j] * cos (angleZ + addedHeading) * sin (angleY));
            point.y = rabbit.z + (LidarSensor::sensorData[i][j] * sin (angleZ + addedHeading));
            point.x = rabbit.x + (LidarSensor::sensorData[i][j] * cos (angleZ + addedHeading) * cos (angleY));

            //if(point.z < -0.5 || point.z > 1) continue;
            //if(sqrt((point.x*point.x)+(point.y*point.y)+(point.z*point.z))>10) continue;

            rvizData.points.push_back(point);
            angleY += 0.034906585;
        }
        angleZ += 0.034906585;
    }
    publishDataToRviz();
}

void LidarSensor::publishDataToRviz()
{
    LidarSensor::toRviz.publish(rvizData);
}

void LidarSensor::initializeRvizMarker()
{
 //init headers
	rvizData.header.frame_id    = "rabbit_follow";
	rvizData.header.stamp       = ros::Time::now();
	rvizData.ns                 = "lidar_sensor_rabbit";
	rvizData.action             = visualization_msgs::Marker::ADD;
	rvizData.pose.orientation.w = 1.0;

    //setting id for each marker
    rvizData.id = 10;
	//defining types
	rvizData.type   = visualization_msgs::Marker::POINTS;
	//setting scale
	rvizData.scale.x   = 0.2;
    rvizData.scale.y   = 0.2;
    rvizData.scale.z   = 0.2;

    //assigning colors
	rvizData.color.r = 1.0f;
	rvizData.color.a = 1.0f;
}
