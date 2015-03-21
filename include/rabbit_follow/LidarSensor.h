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
        static int verticalReadings;
        static int horizontalReadings;
        static int range;
        static int increment;
        static float sensorData[lidarReadingsRange][lidarReadingsRange];
        static bool sensorDataUpdated;

        static visualization_msgs::Marker rvizData;
        static ros::Publisher toRviz;

        static void initializeRvizMarker();
        static void publishDataToRviz();
        static void initializeLidarSensorData(int increment, int range);
        static void callbackUpdateLidarData(const rabbit_follow::lidarData::ConstPtr& lidarSensor);
        //static void isObstacleInFront(vector<int> );

        /** Default destructor */
        virtual ~LidarSensor();
    protected:
    private:
    /** Default constructor */
        LidarSensor();
};

#endif // LIDARSENSOR_H
