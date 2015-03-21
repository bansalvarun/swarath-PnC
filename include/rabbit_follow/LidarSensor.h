#ifndef LIDARSENSOR_H
#define LIDARSENSOR_H

#include <ros/ros.h>
#include <rabbit_follow/lidarData.h>
#include <rabbit_follow/GlobalDeclaration.h>

class LidarSensor
{
    public:
        static int verticalReadings;
        static int horizontalReadings;
        static int range;
        static int increment;
        static float sensorData[lidarReadingsRange][lidarReadingsRange];
        static bool sensorDataUpdated;


        static void initializeLidarSensorData(int increment, int range);
        static void callbackUpdateLidarData(const rabbit_follow::lidarData::ConstPtr& lidarSensor);

        /** Default destructor */
        virtual ~LidarSensor();
    protected:
    private:
    /** Default constructor */
        LidarSensor();
};

#endif // LIDARSENSOR_H
