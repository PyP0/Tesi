#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#include "cpxmacro.h"
#include <vector>
#include <unordered_map>

#define DISCONNECTED_COST CPX_INFBOUND
#define INSTANCE_PATH "../instances/"
#define MASTER_FILE "../instances/master.txt"

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
extern std::vector< std::vector< std::vector<double> > > c;

//matrice di capacità degli archi
extern std::vector< std::vector<double> > u;

//matrice bilanciamento flussi
extern std::vector< std::vector<double> > b;

int getUsrsNum();
int getDrnsNum();
int getPosNum();
int getCommsNum();
int getGridL();
int getGridH();
int getGridStep();
int getNodeRadius();
int getTXCapacity();
int getRXCapacity();
int getThreshold();
int getMaxConnections();
int getTotalPotentialNodes();
int getTotalNodes();

double getDistanceCoef(int,int);
double getInterferenceFactor(int, int);
void printNodesIOFlow();
void printGrid();
void printBuckets(std::unordered_map<int,int>);
int saveInstance(const char *);
int loadInstance(const char *);
int createBatchInstances(int,std::vector<int>,std::vector<int>,std::vector<int>,std::vector<int>,std::vector<int>,std::vector<int>,int,int,double,double,double,double,std::string);

#endif
