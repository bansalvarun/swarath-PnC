/******************************************************
* Rabbit description file for rabbit follow algorithm  *
* Written for SWARATH Project                         *
* @author Nishant Sharma                              *
* @version 0.0                                        *
* @date 4, March, 2015                                *
******************************************************/

#ifndef Rabbit_H
#define Rabbit_H

#include <geometry_msgs/Point.h>
#include <rabbit_follow/GlobalDeclaration.h>

enum RabbitState{followingCarrot, reachedEnd};

class Rabbit
{
    public:
        /** Default constructor */
        Rabbit();
        Rabbit(float x, float y, float radian, RabbitState state);

        /** Default destructor */
        virtual ~Rabbit();

        /** get functions **/
        float getRabbitPosition();
        RabbitState getState();
        float getSteering();
        float getCarrotXlocation();
        float getCarrotYlocation();

        /** set functions **/
        void setXLocation(float x);
        void setYLocation(float y);
        void setDirection(float radian);
        void setState(RabbitState state);
        void setCarrotXLocation(float x);
        void setCarrotYLocation(float y);

        /** modifier functions **/
        void changeDirection(float radian);
        void moveRabbit();
        void updateState();

        void publishThrottle();
        void publishSteering();

        void callbackUpdateRabbitLocation(const geometry_msgs::Point::ConstPtr& rabbit);
        void callbackUpdateCarrotLocation(const geometry_msgs::Point::ConstPtr& carrot);

    protected:
    private:

        Position rabbit;
        float carrotDirection;
        float carrotDistance;
        float steering;
        float throttle;
        RabbitState state;
};

#endif // Rabbit_H
