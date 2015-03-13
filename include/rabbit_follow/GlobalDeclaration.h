#ifndef GLOBALDECLARATION_H
#define GLOBALDECLARATION_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

const float MaximumDistanceFromRabbit = 5;

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

#endif // GLOBALDECLARATION_H
