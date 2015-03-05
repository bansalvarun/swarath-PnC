/******************************************************
* Robot description file for rabbit follow algorithm  *
* Written for SWARATH Project                         *
* @author Nishant Sharma                              *
* @version 0.0                                        *
* @date 4, March, 2015                                *
******************************************************/

#ifndef ROBOT_H
#define ROBOT_H

#include <geometry_msgs/Point.h>

enum State{followingCarrot, reachedEnd};

class Robot
{
    public:
        /** Default constructor */
        Robot();
        Robot(float x, float y, float radian, State state);

        /** Default destructor */
        virtual ~Robot();

        /** get functions **/
        float getXLocation();
        float getYLocation();
        State getState();
        float getDirection();
        float getCarrotXlocation();
        float getCarrotYlocation();

        /** set functions **/
        void setXLocation(float x);
        void setYLocation(float y);
        void setDirection(float radian);
        void setState(State state);
        void setCarrotXLocation(float x);
        void setCarrotYLocation(float y);

        /** modifier functions **/
        void changeDirection(float radian);
        void callbackUpdateRobotLocation(const geometry_msgs::Point::ConstPtr& robot);
        void callbackUpdateCarrotLocation(const geometry_msgs::Point::ConstPtr& carrot);

    protected:
    private:
        float xLocation;
        float yLocation;
        float carrotXLocation;
        float carrotYLocation;
        float direction;
        State state;
};

#endif // ROBOT_H
