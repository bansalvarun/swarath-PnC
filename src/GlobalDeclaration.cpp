#include "rabbit_follow/GlobalDeclaration.h"
#include <cmath>


std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

std::string floatToString(float value)
{
    std::ostringstream ss;
    ss << value;
    std::string s(ss.str());
    return s;
}

float GetEuclideanDistance(Position one, Position two)
{
    return sqrt( pow( one.x - two.x ,2) + pow( one.z - two.z  ,2) );
}

float GetAngle(Position destination, Position source)
{
    return atan2(destination.z - source.z , destination.x - source.x);
}

float ToRadians(float degrees)
{
    return degrees * M_PI / 180;
}

float ToDegrees(float radians)
{
    return radians * 180 / M_PI;
}
int count1=0;
int count2=0;
float ang_wrap(float radians)
{
    while ((radians > M_PI) && (count1<100))
    {radians=radians-(2*M_PI);
     count1++;
    }
    count1=0;
    while (radians < (-1*M_PI) && count2<100)
   { radians=radians+(2*M_PI);
     count2++;
   }
   count2=0;
    return radians;
}
