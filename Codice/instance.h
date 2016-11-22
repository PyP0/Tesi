#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#include "cpxmacro.h"
#include <vector>
#include <unordered_map>
#include <map>

// #define DISCONNECTED_COST CPX_INFBOUND NOCOST
//#define INSTANCE_PATH "../instances/" DATASET
#define INSTANCE_PATH "../dataset/"
#define MASTER_FILE "master.txt"

//test coordinate discrete su griglia
struct nodesCoordinates_t
{
	int x;
	int y;
	bool isUser;
	int id;
};

//Global data

//costo deployment drone
extern std::vector<double> deployCost; 

//matrice dei costi
//extern std::vector< std::vector< std::vector<double> > > c;

//matrice di capacità degli archi
extern std::vector< std::vector<double> > u;

//matrice bilanciamento flussi
extern std::vector< std::vector<double> > b;

extern std::map<int, nodesCoordinates_t> mapGrid;

int getUsrsNum();
int getDrnsNum();
//int getPosNum();
int getCommsNum();
int getGridL();
int getGridH();
int getGridStep();
int getNodeRadius();
int getEpsilonNodeRadius();
int getTXCapacity();
int getRXCapacity();
int getThreshold();
int getMaxConnections();
int getTotalPotentialNodes();
//int getTotalNodes();
double getDeployCost(int);
int getRequestedTraffic(int,int);


double getWCInterferencePercentage();
double getReductionFactor();
double getInterferenceCoef(int, int);
double getInterference(int, int); //related to the matrix
//double getCost(int,int,int); NOCOST

//bool isCEmpty(); NOCOST

double getInterferenceFactor(int, int);   

void shadowingTest(int); 
void interferenceModelTest();
void printNodesIOFlow();
void printGrid();
void printTrafficMatrix();
void printBuckets(std::unordered_map<int,int>);
int saveInstance(const char *);
int loadInstance(const char *);
int createBatchInstances(int,std::vector<int>,std::vector<int>,std::vector<int>,std::vector<int>,std::vector<int>,int,int,double,double,double,double,std::string, std::string);

#endif
