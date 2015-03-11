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

float getEuclideanDistance(Position one, Position two)
{
    return sqrt( pow( one.x - two.x ,2) + pow( one.z - two.z  ,2) );
}
