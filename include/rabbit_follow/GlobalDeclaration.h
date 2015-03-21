#ifndef GLOBALDECLARATION_H
#define GLOBALDECLARATION_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

struct Position
{
    float x;
    float y;
    float z;
};

std::vector<std::string> split(const std::string &s, char delim);
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
float GetAngle(Position destination, Position source);
float GetEuclideanDistance(Position one, Position two);
std::string floatToString(float value);

float ToRadians(float degrees);
float ToDegrees(float radians);

const float MaximumDistanceFromRabbit = 4;
const float MaximumDistanceOnTurn = 4;
const float MaximumDistanceFromRabbitToWayPoint = 5;
const float MaximumAllowedVelocity = 5.5;
const float NanoSecondsInOneSecond = 1000000000;
const float MaximumAllowedTurnValue = ToRadians(30); // value 30 is allowed max turn in unity
const float RabbitMaxAcceleration = 0.07; //units per sec^2
const int MinAllowedDistanceCarrotToRabbit = 3;

const int lidarReadingsRange = 60;

#endif // GLOBALDECLARATION_H
