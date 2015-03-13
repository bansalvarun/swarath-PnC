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

const float MaximumDistanceFromRabbit = 5;
const float NanoSecondsInOneSecond = 1000000000;
const float MaximumAllowedTurnValue = ToRadians(28); // value 28 is allowed max turn in unity
#endif // GLOBALDECLARATION_H
