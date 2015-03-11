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
#include <rabbit_follow/carrotPosition.h>

enum RabbitState{followingCarrot, reachedEnd};

class Rabbit
{
    public:
        /** Default constructor */
        Rabbit();

        /** Default destructor */
        virtual ~Rabbit();

        /** get functions **/
        float getRabbitPosition();
        RabbitState getState();
        float getSteering();
        float getThrottle();
        float getCarrotPosition();

        /** set functions **/
        void setRabbitPosition(Position rabbit);
        void setState(RabbitState state);

        /** modifier functions **/
        void changeSteering(float radian);
        void changeThrottle(float throttle);
        void moveRabbit(const ros::TimerEvent& event);
        void updateState();

        void callbackUpdateRabbitGPSLocation(const std_msgs::String::ConstPtr& rabbit);
        void callbackUpdateRabbitIMULocation(const std_msgs::String::ConstPtr& rabbit);
        void callbackUpdateCarrotLocation(const rabbit_follow::carrotPosition::ConstPtr& carrot);

        ros::Publisher steer_pub;
        ros::Publisher throttle_pub;

    protected:
    private:

        Position rabbit;

        rabbit_follow::carrotPosition carrotPos;

        float steering;
        float throttle;

        float velocity;

        RabbitState state;
};

#endif // Rabbit_H
