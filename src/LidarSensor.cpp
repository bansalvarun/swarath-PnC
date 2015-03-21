#include "rabbit_follow/LidarSensor.h"


int LidarSensor::verticalReadings;
int LidarSensor::horizontalReadings;
int LidarSensor::range;
int LidarSensor::increment;
float LidarSensor::sensorData[lidarReadingsRange][lidarReadingsRange];
bool LidarSensor::sensorDataUpdated;
visualization_msgs::Marker LidarSensor::rvizData;
ros::Publisher LidarSensor::toRviz;


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

void LidarSensor::publishDataToRviz()
{
    rvizData.points.clear();
    geometry_msgs::Point point;
    float angleZ = (-1 * M_PI/3), angleY;
    for(int i =0 ;i < lidarReadingsRange;i++)
    {
        angleY = -1 * M_PI/3;
        for(int j=0;j < lidarReadingsRange;j++)
        {
            point.z = (LidarSensor::sensorData[i][j] * cos (angleZ) * sin (angleY));
            point.y = (LidarSensor::sensorData[i][j] * sin (angleZ));
            point.x = (LidarSensor::sensorData[i][j] * cos (angleZ) * cos (angleY));
            rvizData.points.push_back(point);
            angleY += 0.034906585;
        }
        angleZ += 0.034906585;
    }
    LidarSensor::toRviz.publish(rvizData);
}

void LidarSensor::initializeRvizMarker()
{
 //init headers
	rvizData.header.frame_id    = "lidar_sensor";
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
