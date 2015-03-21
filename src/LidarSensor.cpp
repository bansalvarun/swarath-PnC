#include "rabbit_follow/LidarSensor.h"


int LidarSensor::verticalReadings;
int LidarSensor::horizontalReadings;
int LidarSensor::range;
int LidarSensor::increment;
float LidarSensor::sensorData[lidarReadingsRange][lidarReadingsRange];
bool LidarSensor::sensorDataUpdated;

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
    //ROS_INFO("Lidar Sensor Updated");
}

bool LidarSensor::isObstacleInFront(vector<> )
{


}
