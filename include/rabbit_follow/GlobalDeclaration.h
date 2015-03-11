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

#endif // GLOBALDECLARATION_H
