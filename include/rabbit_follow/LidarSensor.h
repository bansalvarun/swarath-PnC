#ifndef LIDARSENSOR_H
#define LIDARSENSOR_H

#include <ros/ros.h>
#include <rabbit_follow/lidarData.h>
#include <rabbit_follow/GlobalDeclaration.h>
#include <visualization_msgs/Marker.h>
#include <vector>

using namespace std;

class LidarSensor
{
    public:
        /** Default constructor */
        LidarSensor();
        void initializeRvizMarker();
        void setRvizMarkerData(Position rabbit, float addedHeading);
        void publishDataToRviz();
        void initializeLidarSensorData(int increment, int range);
        void callbackUpdateLidarData(const rabbit_follow::lidarData::ConstPtr& lidarSensor);
        ros::Publisher toRviz;
        /** Default destructor */
        virtual ~LidarSensor();
    protected:
    private:
        int verticalReadings;
        int horizontalReadings;
        int range;
        int increment;
        float sensorData[lidarReadingsRange][lidarReadingsRange];
        bool sensorDataUpdated;

        visualization_msgs::Marker rvizData;


};

#endif // LIDARSENSOR_H
