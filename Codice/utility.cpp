#include "utility.h"
#include <chrono>
#include <random>
#include <iostream>
#include <string>
#include <fstream>

#include <cerrno>
#include <cstring>

using namespace std;

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

//generatore pseudo-casuale di numeri int nel range inf - sup
int getLoadedRand(int inf, int sup)
{
	// obtain a seed from the system clock:
	unsigned int seed1 = std::chrono::system_clock::now().time_since_epoch().count();

	std::mt19937 generator(seed1);   // mt19937 is a standard mersenne_twister_engine
	std::uniform_int_distribution<int> distribution(inf, sup);
	return distribution(generator);
}

// arrotonda numToRound al multiplo di multiple successivo. Es: 10, 7 -> 14
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
	if (getDistance(px, py, centerx, centery) <= (double)radius)
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

bool printClusterJob(std::string fName,std::string path, std::string exename, std::string err_out_name, std::string command, std::string variation, std::string home, int time)
{
	ofstream file;
	cout << path << endl;
	string fileName(path + fName);
	cout << fileName << endl;
	file.open(fileName, ios::out);
		if (file.is_open())
		{
			
			file << "#!/bin/sh" <<endl;
			file <<	"### Set the job name" << endl;
			file <<	"#PBS -N " << exename << endl;
			file << endl;
			file <<	"### Declare myprogram non-rerunable" << endl;
			file <<	"#PBS -r n"<< endl;
			file << endl;
			file <<	"### Uncomment to send email when the job is completed:" << endl;
			file <<	"#PBS -m ae" << endl;
			file <<	"#PBS -M filippo.gamberoni@studenti.unipd.it" << endl;
			file << endl;
			file <<	"### Optionally specifiy destinations for your myprogram's output" << endl;
			file <<	"### Specify localhost and an NFS filesystem to prevent file copy errors." << endl;
			file << endl;
			file <<	"#PBS -e localhost:${HOME}/Cluster/"<<err_out_name<<".err" << endl;
			file <<	"#PBS -o localhost:${HOME}/Cluster/"<<err_out_name<<".out" << endl;
			file << endl;
			file <<	"### Set the queue to \"cluster_long\"" << endl;
			file <<	"#PBS -q cluster_long" << endl;
			file << endl;
			file <<	"### Specify the number of cpus for your job.  This example will run on 1 cpus " << endl;
			file <<	"### using 1 nodes with 1 process per node.  " << endl;
			file <<	"### You MUST specify some number of nodes or Torque will fail to load balance." << endl;
			file <<	"#PBS -l nodes=1:ppn=8:cluster_intel:ubuntu14" << endl;
			file << endl;
			file <<	"### You should tell PBS how much memory you expect your job will use.  mem=1g or mem=1024m" << endl;
			file <<	"#PBS -l mem=32g" << endl;
			file << endl;
			file <<	"### You can override the default 1 hour real-world time limit.  -l walltime=HH:MM:SS" << endl;
			file <<	"### Jobs on the public clusters are currently limited to 10 days walltime." << endl;
			file <<	"#PBS -l walltime="<<time<<":00:00" << endl;
			file << endl;
			file <<	"### Switch to the working directory; by default Torque launches processes from your home directory." << endl;
			file <<	"### Jobs should only be run from /home, /project, or /work; Torque returns results via NFS." << endl;
			file <<	"cd ${HOME}/Cluster/"<< home <<"/TESIv1.0" << endl;
			file << endl;
			file <<	"### Run some informational commands." << endl;
			file <<	"echo Running on host `hostname`" << endl;
			file <<	"echo Time is `date`" << endl;
			file <<	"echo Directory is `pwd`" << endl;
			file <<	"echo This jobs runs on the following processors:" << endl;
			file <<	"echo `cat $PBS_NODEFILE`" << endl;
			file <<	"echo PBS_WORKDIR=$PBS_WORKDIR" << endl;
			file << endl;
			file <<	"### Define number of processors" << endl;
			file <<	"NPROCS=`wc -l < $PBS_NODEFILE`" << endl;
			file <<	"echo This job has allocated $NPROCS cpus" << endl;
			file << endl;
			file << command << variation << "/" << endl;
			//file <<	"./main -a ../"<<variation<<"/";
			return true;
		}
		else
		{
			cerr << __FUNCTION__ << "(): Impossibile stampare file main.job" <<endl;
			return false;
		}
	
	
	
}
