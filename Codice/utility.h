#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <string>

double getdRand(double, double);
int getRand(int, int);
int roundUp(int, int);
int roundDown(int, int);
double getDistance(int, int, int, int);
bool isIntersection(double, double, double, double, double, double);
bool isInRange(int, int, int, int, int);
bool printClusterJob(std::string,std::string,std::string,std::string,std::string,std::string, std::string,int, std::string);
#endif
