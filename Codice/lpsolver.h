#ifndef _LP_SOLVER_H
#define _LP_SOLVER_H

#include "cpxmacro.h"
#include <vector>

#define DISCONNECTED_COST CPX_INFBOUND

// error status and message buffer
static int status;
static char errmsg[BUF_SIZE];

static const int NAME_SIZE = 512;
static char name[NAME_SIZE];

//Global data

//numero di utenti
extern int n = 0;
// numero di droni
extern int d = 0;
//numero di posizioni potenziali dei droni
extern int P = 0;

//numero di commodities, ovvero numero di coppie distinte sorgente-destinazione
extern int K = n*(n - 1);

extern int totalNodes = n + d;
extern int totalPotentialNodes = n + P;

static const int s = 10; //numero massimo connessioni sostenibili da un drone

static const int bigM = 1000; // TODO: bigM deve essere maggiore della capacità massima dei link

const int threshold = 100; //soglia oltre la quale un costo viene considerato infinito

//indici iniziali per la memorizzazione delle variabili di CPLEX
int f_index = 0;
int y_index = 0;
int x_index = 0;
int w_index = 0;

//double c[n+d][n+d][K];
//matrice dei costi
std::vector< std::vector< std::vector<double> > > c(totalPotentialNodes, std::vector< std::vector<double> >(totalPotentialNodes, std::vector<double>(K, DISCONNECTED_COST)));

//matrice di capacità degli archi
std::vector< std::vector<double> > u(totalPotentialNodes, std::vector<double>(totalPotentialNodes));

//matrice bilanciamento flussi
std::vector< std::vector<double> > b(totalPotentialNodes, std::vector<double>(K, 0));


const double zero = 0.0;
const double uno = 1.0;


bool areOutOfSight(int, int);
void setupLP(CEnv,Prob);

#endif _LP_SOLVER_H
