#include "instance.h"
#include "utility.h"
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <chrono>

#include <map> 

using namespace std;
using namespace std::chrono;

//numero di utenti
int n = 0;
// numero di droni
int d = 0;
//numero di posizioni potenziali dei droni
int P = 0;

int totalNodes = n + d;
int totalPotentialNodes = n + P;

//numero di commodities, ovvero numero di coppie distinte sorgente-destinazione
int K = n*(n - 1);

int threshold = 100; //soglia oltre la quale un costo viene considerato infinito

int s = 10; //numero massimo connessioni sostenibili da un drone

int droneTXCapacity = 20000;
int droneRXCapacity = 10000;

int nodeRadius = 3;

//parametri griglia
int length = 0;
int height = 0;
int step = 0;

std::vector< nodesCoordinates_t > grid;

map<int, nodesCoordinates_t> mapGrid;

std::vector< std::vector< std::vector<double> > > c(totalPotentialNodes, std::vector< std::vector<double> >(totalPotentialNodes, std::vector<double>(K, DISCONNECTED_COST)));

//matrice di capacità degli archi
std::vector< std::vector<double> > u(totalPotentialNodes, std::vector<double>(totalPotentialNodes));

//matrice bilanciamento flussi
std::vector< std::vector<double> > b(totalPotentialNodes, std::vector<double>(K, 0));

std::vector<double> deployCost(P, 50); //costo deployment drone

std::vector< std::vector<int> > t(n, std::vector<int>(n, -1)); //matrice di traffico

int getUsrsNum()
{
	return n;
}

int getDrnsNum()
{
	return d;
}

int getPosNum()
{
	return P;
}

int getCommsNum()
{
	return K;
}

int getGridL()
{
	return length;
}

int getGridH()
{
	return height;
}

int getGridStep()
{
	return step;
}

int getNodeRadius()
{
	return nodeRadius;
}

int getTXCapacity()
{
	return droneTXCapacity;
}

int getRXCapacity()
{
	return droneRXCapacity;
}

int getThreshold()
{
	return threshold;
}

int getMaxConnections()
{
	return s;
}

int getTotalPotentialNodes()
{
	return totalPotentialNodes;
}

int getTotalNodes()
{
	return totalNodes;
}

double getDistanceCoef(int i, int j)
{
	return 1.0; //TODO
}

double getInterferenceFactor(int i, int j)
{
	return 1.0; //TODO
}

//costruisce la matrice b partendo dalla matrice di traffico 
//la matrice b e' pre-inizializzata al valore 0
static void convertIntoBMatrix()
{
	if (t.size() > 0 && b.size() > 0)
	{
		int k = 0; //commodity counter
		for (unsigned int i = 0; i < t.size(); i++)
		{
			for (unsigned int j = 0; j < t.size(); j++)
			{
				if (i != j)
				{
					//copia i valori dalla matrice t alla b. I valori vengono resi negativi perche' il traffico e'"uscente" dai nodi
					b[i][k] = t[i][j] * (-1);

					//il traffico uscente dal nodo i al nodo j sara' visto come traffico entrante dal nodo j
					b[j][k] = t[i][j];
					k++;
				}
			}
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}

static int placeUsersSere()
{
	if (grid.size() > 0 && n > 0)
	{
		unordered_map<int, int> mapPool;
		int size = length * height;
		
		mapPool.reserve(size);
		for (int i = 0; i < size; i++)
		{
			mapPool[i] = i;
			grid[i].isUser = false;
			grid[i].id = -1;
		}

		cout<< "start"<<endl;

		for (int i = 0; i < n; i++)
		{
			//int chosenIndex = getRand(0, (int)mapPool.size()-1);
			//int chosenValue = mapPool.at(chosenIndex);


			int chosenIndex = 0;
			do //TODO: renderla deterministica
			{
				chosenIndex = getRand(0, mapPool.bucket_count() - 1);
			} 
			while (mapPool.bucket_size(chosenIndex) == 0);

			//alternativa, deterministica ma piu' lenta
			//else: auto random_it = std::next(std::begin(edges), rand_between(0, edges.size()));
			//auto element = next(begin(mapPool),getRand(0,mapPool.size()) );

			//printBuckets(mapPool);
			//getRand(0, mapPool.bucket_count() - 1);

			//chosenValue = std::advance(mapPool.begin(chosenIndex), 1);
			auto element = mapPool.begin(chosenIndex);
			//advance(element, 0); non causa modifiche
			int chosenValue = element->second;

			
			//randomize by bucket 

			grid[chosenValue].isUser = true;
			grid[chosenValue].id = i;

			cout << chosenValue << " " << grid[chosenValue].id << " " << grid[chosenValue].x << " " << grid[chosenValue].y << endl;
			
			//cout << "Ho scelto il nodo: " << chosenIndex << " " << chosenValue << " " << grid[chosenValue].x << " " << grid[chosenValue].y << endl;
			if (nodeRadius >= step)
			{
				int startX = max(0, roundUp(grid[chosenValue].x - nodeRadius, step)), startY = max(0, roundUp(grid[chosenValue].y - nodeRadius, step));

				int endX = min(roundDown(grid[chosenValue].x + nodeRadius, step), length - 1), endY = min(roundDown(grid[chosenValue].y + nodeRadius, step), height - 1);

				//cout << "start(" << startX << " " << startY << ")" << endl;
				//cout << "end(" << endX << " " << endY << ")" << endl;
				for (int y = startY; y <= endY; y = y + step)
				{
					for (int x = startX; x <= endX; x = x + step)
					{
						//eliminazione dei nodi
						double dst = getDistance(x, y, grid[chosenValue].x, grid[chosenValue].y);
						//cout << "dist: " << "(" << x << "," << y << ") " << "(" << grid[chosenValue].x << "," << grid[chosenValue].y << ")" << " : " << dst << endl;
						if (dst <= nodeRadius)
						{
							if (mapPool.size() == 0)
							{
								cerr << __FUNCTION__ << "(): Impossibile posizionare gli utenti." << endl;
								return 1;
							}
							else
							{
								int nodeId = (y / step) * length + (x / step);

								//int counter=0, flag=0; unsigned int i=0;
								/*while(i<mapPool.size() && flag==0)
								{
								if(mapPool[i][1] == nodeId)
								flag=1;
								else
								counter++;
								i++;
								}*/
								//cout << "sto per cancellare id: " << mapPool[nodeId][1] <<endl;
								//cout << "sto per cancellare id: " << counter <<endl;
								//mapPool.erase(mapPool.begin() + nodeId); 
								//cout << "Sto per cancellare il nodo " << nodeId << endl;
								mapPool.erase(nodeId);

								if (mapPool.size() == 0)
								{
									cerr << __FUNCTION__ << "(): Impossibile posizionare gli utenti." << endl;
									return 1;
								}

								//cout << "nodeId " << nodeId << " size: " << mapPool.size() << endl;

								//int check = 0;
								/*for (auto& x : mapPool)
								{
									if (nodeId == x.first)
										check = 1;
									cout << x.first << " ";
								}*/
								/*cout << endl;
								if (check == 0)
									cout << "Il nodo: " << nodeId << " e' stato eliminato." << endl;*/
							}
						}
					}
				}
				//cout << " cancello il nodo " << chosenIndex << endl;
				mapPool.erase(chosenIndex);
			}
		}

		//assegnazione id ai nodi P
		int progressiveId = n;  
		for(unsigned int i=0; i < grid.size(); i++)
		{
			if(grid[i].isUser == false)
			{
				grid[i].id = progressiveId;
				progressiveId++;
			}
		}

		
		//filling the map
		for(unsigned int i=0; i < grid.size(); i++)
		{
			int gridIndex = grid[i].id;

			mapGrid[gridIndex].id = i;
			mapGrid[gridIndex].isUser = grid[i].isUser; 
			mapGrid[gridIndex].x = grid[i].x; 
			mapGrid[gridIndex].y = grid[i].y; 
		}

		/*for(unsigned int i=0; i< mapGrid.size();i++)
		{
			cout<< i <<")"<<endl;
			cout<< mapGrid[i].id <<endl;
			cout<< mapGrid[i].isUser <<endl;
			cout<< mapGrid[i].x <<endl;
			cout<< mapGrid[i].y <<endl;
		}*/

		return 0;
	}
	else
	{
		cerr << __FUNCTION__ << "(): Uno o piu' parametri sono errati." << endl;
		return 1;
	}
}

//alloca e popola un vettore per ospitare le coordinate di tutti i punti 
static int createGrid()
{
	if (length >0 && height > 0 && step >0)
	{
		int xCoord = 0, yCoord = 0;
		int rows = 0;
		int index = 0;
		int status = 0;
		try
		{
			grid.resize(length*height);
			for (int a = 0; a<height; a++)
			{
				xCoord = 0;

				for (int b = 0; b< length; b++)
				{
					grid[index].isUser = false;

					grid[index].x = xCoord;
					xCoord += step;

					grid[index].y = yCoord;

					index++;
				}
				rows++;
				yCoord += step;
			}
			

			return 0;
		}
		catch (exception &e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
			return 1;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Uno o piu' parametri sono errati.";
		//return empty vector
		return 1;
	}
}

static int initDataStructures(int n, int d, int P)
{
	int success = 0;
	if (n > 0 && d > 0 && P > 0)
	{
		totalNodes = n + d;
		totalPotentialNodes = n + P;
		K = n*(n - 1);

		try
		{
			//matrice nxn, init=-1
			t.resize(n);
			for (int i = 0; i < n; i++)
			{
				t[i].resize(n, -1);
			}

			//vettore P, init=-1
			deployCost.resize(P, -1);

			//matrice 3D (n+P)x(n+P)x(K), init=DISCONNECTED_COST
			c.resize(totalPotentialNodes);
			for (int i = 0; i < totalPotentialNodes; i++)
			{
				c[i].resize(totalPotentialNodes);
				for (int j = 0; j < totalPotentialNodes; j++)
				{
					c[i][j].resize(K, DISCONNECTED_COST);
				}
			}

			//matrice (n+P)x(n+P), init=-1
			u.resize(totalPotentialNodes);
			for (int i = 0; i < totalPotentialNodes; i++)
			{
				u[i].resize(totalPotentialNodes, -1);
			}

			//matrice (n+P)x(K), init=0
			b.resize(totalPotentialNodes);
			for (int i = 0; i < totalPotentialNodes; i++)
			{
				b[i].resize(K, 0);
			}

			success = createGrid();
		}
		catch (exception& e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
			success = 1;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile allocare strutture con dimensione 0." << endl;
		success = 1;
	}
	return success;
}

static void randomizeTraffic(int minVal, int maxVal)
{
	if (t.size() > 0)
	{
		for (unsigned int i = 0; i < t.size(); i++)
		{
			for (unsigned int j = 0; j < t[0].size(); j++)
			{
				if (i != j)
					t[i][j] = getRand(minVal, maxVal);
			}
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Struttura dati non allocata." << endl;
	}
}

static void randomizeCosts(double minCVal, double maxCVal, int radius)
{
	if (c.size() > 0 && grid.size() > 0)
	{
		for (unsigned int i = 0; i < c.size(); i++)
		{
			for (unsigned int j = 0; j < c[0].size(); j++)
			{
				for (unsigned int k = 0; k< c[0][0].size(); k++)
				{
					if (i != j && !(i < (unsigned)n && j < (unsigned)n))
					{
						//cout<<"i: "<<i<<" j: "<<j<<endl;
						if( isInRange(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y, radius) )
							c[i][j][k] = getdRand(minCVal, maxCVal);
						else
							c[i][j][k] = DISCONNECTED_COST;
					}
				}
			}
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}

static void randomizeDeployCost(double minVal, double maxVal)
{
	if (deployCost.size() > 0)
	{
		for (unsigned int i = 0; i < deployCost.size(); i++)
		{
			deployCost[i] = getdRand(minVal, maxVal);
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}

}

static int createRandomInstance(int users, int drones, int positions, int tInf, int tSup, double cInf, double cSup, double dInf, double dSup, int gridLength, int gridHeight, int gridStep, string filename)
{
	int result=0;
	
	if(users > 0 && drones > 0 && positions > 0 && gridLength > 0 && gridHeight >0 && gridStep >0)
	{
		n=users;
		d=drones;
		P=positions;  
		length = gridLength;
		height = gridHeight;
		step = gridStep; 
		result= initDataStructures(users,drones,positions);
		if(result==0)
		{
			//TODO
			int trials = 50;
			do
			{
				result = placeUsersSere();
				trials--;
			}
			while(result != 0 && trials > 0 );
			if(result != 0)
			{
				cerr << __FUNCTION__ << "(): Impossibile posizionare gli utenti sulla griglia." << endl;
				result = 1;
			}

			if(result == 0)
			{
				randomizeTraffic(tInf,tSup);
				randomizeCosts(cInf,cSup,nodeRadius);
				randomizeDeployCost(dInf,dSup);

				//TODO: per l'amor d'iddio rimuovere assolutamente!!!!!
				for(unsigned int i=0;i<u.size();i++)
				{
					for(unsigned int j=0; j<u[0].size();j++)
					{
						u[i][j]=1;
					}
				}

				convertIntoBMatrix();
				saveInstance(filename.c_str());
			}
			else
			{
				cerr << __FUNCTION__ << "(): Impossibile creare la nuova istanza." << endl;
			}
		}
		else
		{
			cerr << __FUNCTION__ << "(): Impossibile allocare le strutture dati necessarie alla nuova istanza." << endl;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): I parametri passati sono errati." << endl;
		result = 1;
	}
	return result;
}


int createBatchInstances(int instQuantity, vector<int> users, vector<int> drones, vector<int> positions, vector<int> gridLength, vector<int> gridHeight, vector<int> gridStep, int tInf, int tSup, double cInf, double cSup, double dInf, double dSup, string instRootName)
{
	if ((instQuantity > 0)) //&& (users.size() == drones.size() && drones.size() == positions.size() && positions.size() == gridLength.size() && gridLength.size() == gridHeight.size() && gridHeight.size() == gridStep.size()))
	{
		int result = 0;
		ofstream file;
		file.open(MASTER_FILE, ios::out);
		if (file.is_open())
		{
			for (unsigned int i = 0; i<users.size(); i++) //TODO: completare con gli altri cicli annidati
			{
				for (int j = 0; j<instQuantity; j++)
				{
					string composeName = INSTANCE_PATH + instRootName + "_i" + to_string(j) + "_u" + to_string(users[i]) + "_d" + to_string(drones[i]) + "_p" + to_string(positions[i]) + ".txt";
					result = createRandomInstance(users[i], drones[i], positions[i], tInf, tSup, cInf, cSup, dInf, dSup, gridLength[i], gridHeight[i], gridStep[i], composeName);
					if (result != 0)
					{
						cerr << __FUNCTION__ << "(): Errore nella creazione di una istanza." << endl;
						file.close();
						return 1;
					}
					file << composeName << endl;
				}
			}
			file.close();
			return 0;
		}
		else
		{
			cerr << __FUNCTION__ << "(): Impossibile aprire il file: " << file << endl;
			return 1;
		}

	}
	else
	{
		cerr << __FUNCTION__ << "(): Uno o piu' parametri errati." << endl;
		return 1;
	}

}

int saveInstance(const char *filename)
{
	int status = 0;
	if (n > 0 && d > 0 && P>0)
	{
		ofstream file;
		file.open(filename, ios::out);
		if (file.is_open())
		{
			file << n << endl;
			file << d << endl;
			file << P << endl;
			file << s << endl;
			file << threshold << endl;
			file << droneTXCapacity << endl;
			file << droneRXCapacity << endl;
			file << nodeRadius << endl;
			file << length << endl;
			file << height << endl;
			file << step << endl;

			for (unsigned int i = 0; i < deployCost.size(); i++)
			{
				file << deployCost[i] << " ";
			}
			file << endl;

			for (unsigned int i = 0; i < t.size(); i++)
			{
				for (unsigned int j = 0; j < t[0].size(); j++)
				{
					file << t[i][j] << " ";
				}
				file << endl;
			}

			for (unsigned int i = 0; i < u.size(); i++)
			{
				for (unsigned int j = 0; j < u[0].size(); j++)
				{
					file << u[i][j] << " ";
				}
				file << endl;
			}

			for (unsigned int i = 0; i < c.size(); i++)
			{
				for (unsigned int j = 0; j < c[0].size(); j++)
				{
					for (unsigned int k = 0; k < c[0][0].size(); k++)
					{
						if(c[i][j][k] >= threshold)
							file << -1 << " ";
						else
							file << c[i][j][k] << " ";
					}
					file << endl;
				}
			}


			file << mapGrid.size() << endl;
			for(unsigned int i = 0; i< mapGrid.size(); i++)
			{
				file << mapGrid[i].id << " ";
				file << mapGrid[i].isUser << " ";
				file << mapGrid[i].x << " ";
				file << mapGrid[i].y << endl;
			}

			file.close();
			cout << "Creato file: " << filename << endl;
		}
		else
		{
			cerr << __FUNCTION__ << "(): Impossibile aprire il file: " << filename << endl;
			status = 1;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile salvare l'istanza vuota. " << endl;
		status = 1;
	}
	return status;

}

int loadInstance(const char *filename) //TODO: controlli su possibile istanza corrotta (getline)
{
	int result = 0;
	ifstream file;

	n = 0, d = 0, P = 0;

	file.open(filename, ios::in);
	if (file.is_open())
	{
		try
		{
			file >> n >> d >> P;
			file >> s >> threshold >> droneTXCapacity;
			file >> droneRXCapacity >> nodeRadius;
			file >> length >> height >> step;
			if (n > 0 && d > 0 && P > 0)
			{
				if (initDataStructures(n, d, P) == 0)
				{
					for (unsigned int i = 0; i < deployCost.size(); i++)
					{
						file >> deployCost[i];
					}

					for (unsigned int i = 0; i < t.size(); i++)
					{
						for (unsigned int j = 0; j < t[0].size(); j++)
						{
							file >> t[i][j];
						}
					}

					for (unsigned int i = 0; i < u.size(); i++)
					{
						for (unsigned int j = 0; j < u[0].size(); j++)
						{
							file >> u[i][j];
						}
					}

					for (unsigned int i = 0; i < c.size(); i++)
					{
						for (unsigned int j = 0; j < c[0].size(); j++)
						{
							for (unsigned int k = 0; k < c[0][0].size(); k++)
							{
								file >> c[i][j][k];
								if(c[i][j][k] == -1)
									c[i][j][k]= DISCONNECTED_COST;
							}
						}
					}

					int mapSize = 0;
					file >> mapSize;
					for(int i = 0; i< mapSize; i++)
					{
						file >> mapGrid[i].id;
						file >> mapGrid[i].isUser;
						file >> mapGrid[i].x;
						file >> mapGrid[i].y;
					}

					convertIntoBMatrix();
				}
				else
				{
					cerr << __FUNCTION__ << "(): Allocazione strutture fallita." << endl;
					result = 1;
				}
			}
			else
			{
				cerr << __FUNCTION__ << "(): Parametri dell'istanza " << filename << " errati." << endl;
				result = 1;
			}
		}
		catch (exception &e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << filename << ": " << e.what() << endl;
			result = 1;
		}
		file.close();
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile aprire il file " << filename << endl;
		result = 1;
	}
	return result;
}

static void setCompleteGraph(double stdCost, int stdCap)
{
	if (c.size() > 0 && u.size() > 0)
	{
		for (unsigned int i = n; i < c.size(); i++)
		{
			for (unsigned int j = n; j < c[0].size(); j++)
			{
				if (i != j)
				{
					for (unsigned int k = 0; k < c[0][0].size(); k++)
					{
						c[i][j][k] = stdCost;
					}
					u[i][j] = stdCap;
				}
			}
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}

static void setCostsByRadius(int xPos, int yPos, int xMax, int yMax, int radius, float cost)
{
	int yStart, yEnd, xStart, xEnd;
	
	if(xPos-radius <0)
		xStart= xPos;
	else
		xStart= xPos-radius;

	if(yPos-radius <0)
		yStart= yPos;
	else
		yStart= yPos-radius;

	if(xPos+radius >xMax)
		xEnd= xPos;
	else
		xEnd= xPos+radius;

	if(yPos+radius >yMax)
		yEnd= yPos;
	else
		yEnd= yPos+radius;

	for(int y= yStart; y<= yEnd ; y++)
	{
		for(int x=xStart; x<= xEnd;x++)
		{
			for(int k=0;k<K;k++)
			{
				c[x][y][k]= cost;
			}
		}

	}
}

void printGrid()
{
	if (grid.size() > 0)
	{
		for (unsigned int i = 0; i < grid.size(); i++)
		{
			cout << i << ": " << "(" << grid[i].x << ", " << grid[i].y << ")" << endl;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Struttura dati vuota.";
	}
}

void printBuckets(unordered_map<int, int> mymap)
{
	unsigned n = mymap.bucket_count();

	std::cout << "mymap has " << n << " buckets.\n";

	for (unsigned i = 0; i < n; ++i)
	{
		std::cout << "bucket #" << i << " contains: ";
		for (auto it = mymap.begin(i); it != mymap.end(i); ++it)
			std::cout << "[" << it->first << ":" << it->second << "] ";
		std::cout << "\n";
	}

}


void printNodesIOFlow()
{
	if (t.size() > 0)
	{
		int accIN = 0, accOUT = 0;
		for (int i = 0; i < getUsrsNum(); i++)
		{
			accIN = 0, accOUT = 0;
			cout << i << ")";
			for (int j = 0; j < getUsrsNum(); j++)
			{
				if (i != j)
				{
					accOUT += t[i][j];
					accIN += t[j][i];
				}
			}
			cout << "Out= " << accOUT << " In= " << accIN << " Tot= " << accOUT + accIN << endl;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}