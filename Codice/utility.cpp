#include "utility.h"
#include <chrono>
#include <random>


double getdRand(double inf, double sup)
{
	unsigned int seed1 = std::chrono::system_clock::now().time_since_epoch().count();

	std::mt19937 generator(seed1);   // mt19937 is a standard mersenne_twister_engine
	std::uniform_real_distribution<double> distribution(inf, sup);
	return distribution(generator);
}

//generatore pseudo-casuale di numeri int nel range inf - sup
int getRand(int inf, int sup)
{
	// obtain a seed from the system clock:
	unsigned int seed1 = std::chrono::system_clock::now().time_since_epoch().count();

	std::mt19937 generator(seed1);   // mt19937 is a standard mersenne_twister_engine
	std::uniform_int_distribution<int> distribution(inf, sup);
	return distribution(generator);
}

int roundUp(int numToRound, int multiple)
{
	if (multiple == 0)
		return numToRound;

	int remainder = abs(numToRound) % multiple;
	if (remainder == 0)
		return numToRound;

	if (numToRound < 0)
		return -(abs(numToRound) - remainder);
	else
		return numToRound + multiple - remainder;
}

int roundDown(int numToRound, int multiple)
{
	if (multiple == 0)
		return numToRound;

	int remainder = abs(numToRound) % multiple;
	if (remainder == 0)
		return numToRound;

	return numToRound - remainder;
}

double getDistance(int px, int py, int qx, int qy)
{
	return sqrt((pow(px - qx, 2)) + pow(py - qy, 2));
}

bool isInRange(int px, int py, int centerx, int centery, int radius)
{
	if (abs(getDistance(px, py, centerx, centery)) <= radius)
		return true;
	else
		return false;
}

bool isIntersection(double x0, double y0, double r0, double x1, double y1, double r1)
{
	//(R0-R1)^2 <= (x0-x1)^2+(y0-y1)^2 <= (R0+R1)^2
	double diffCoords = pow(x0 - x1, 2) + pow(y0 - y1, 2);
	if (diffCoords >= pow(r0 - r1, 2) || diffCoords <= pow(r0 + r1, 2))
		return true;
	else
		return false;
}
