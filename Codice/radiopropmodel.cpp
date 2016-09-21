#include <math.h>
#include <random>
#include <chrono>
#include "utility.h"
#include <iostream>

// use Friis at less than crossover distance
// use two-ray at more than crossover distance
//static double
double twoRayGroundPr(double Pt, double Gt, double Gr, double ht, double hr, double L, double d)
{
        /*
         *  Two-ray ground reflection model.
         *
         *	     Pt * Gt * Gr * (ht^2 * hr^2)
         *  Pr = ----------------------------
         *           d^4 * L
         *
         * The original equation in Rappaport's book assumes L = 1.
         * To be consistant with the free space equation, L is added here.
         */
	return Pt * Gt * Gr * (hr * hr * ht * ht) / (d * d * d * d * L);
}

double friisPr(double Pt, double Gt, double Gr, double lambda, double L, double d)
{
        /*
         * Friis free space equation:
         *
         *       Pt * Gt * Gr * (lambda^2)
         *   P = --------------------------
         *       (4 * pi * d)^2 * L
         */
	if (d == 0.0) //XXX probably better check < MIN_DISTANCE or some such
		return Pt;
	double M = lambda / (4 * M_PI * d);
	return (Pt * Gt * Gr * (M * M)) / L;
}

/* from threshold.cc
double pr(int i, int j)
{
	double rX, rY, rZ;		// location of receiver
  	double tX, tY, tZ;		// location of transmitter
  	double d;				// distance
	double hr, ht;		// height of recv and xmit antennas
  	double Pr;			// received signal power

  	double L = ifp->getL();			// system loss
  	double lambda = ifp->getLambda();	// wavelength

  	rX = mapGrid[i].x;
  	rY = mapGrid[i].y;

  	tX = mapGrid[j].x;
  	tY = mapGrid[j].y;

  	rZ = 0;
  	tZ = 0;
  
  	d = sqrt((rX - tX) * (rX - tX)  + (rY - tY) * (rY - tY) + (rZ - tZ) * (rZ - tZ));
    

	hr = rZ + r->getAntenna()->getZ();
	ht = tZ + t->getAntenna()->getZ();

	// calc the cross-over distance
  // 
	//       4 * PI * hr * ht
	//d = ----------------------------
	//            lambda
	//    At the crossover distance, the received power predicted by the two-ray
	//    ground model equals to that predicted by the Friis equation.
  //  

	crossover_dist = (4 * PI * ht * hr) / lambda;

   //
   //    If the transmitter is within the cross-over range , use the
   //    Friis equation.  Otherwise, use the two-ray
   //    ground reflection model.
   //  

	double Gt = 1.0; // Ideal antenna
	double Gr = 1.0; // Ideal antenna

	if(d <= crossover_dist)
	{
		Pr = Friis(t->getTxPr(), Gt, Gr, lambda, L, d);
	}
	else 
	{
    	Pr = twoRayGround(t->getTxPr(), Gt, Gr, ht, hr, L, d);   
    }
    return Pr;
}*/



double shadowingPr(double xt, double yt, double xr, double yr, double Pt, double lambda)
{
	double Gt = 1.0;
	double Gr = 1.0;
	double L = 1.0;
	double dist0_ = 1.0; //metri
	double std_db_ = 4.0; //TODO: check
	double pathlossExp_ = 2.0; //TODO: check

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);
	std::normal_distribution<double> normal(0.0,std_db_);
	//std::lognormal_distribution<double> lognormal(0.0,pathlossExp_)

	double dist = getDistance(xt, yt, xr, yr);

	// calculate receiving power at reference distance
	double Pr0 = friisPr(Pt, Gt, Gr, lambda, L, dist0_);

	// calculate average power loss predicted by path loss model
	double avg_db;
	if (dist > dist0_) 
	{
		avg_db = -10.0 * pathlossExp_ * log10(dist/dist0_);
	} 
	else 
	{
		avg_db = 0.0;
	}

	// get power loss by adding a log-normal random variable (shadowing)
	// the power loss is relative to that at reference distance dist0_
	//double powerLoss_db = avg_db + ranVar->normal(0.0, std_db_);
	double powerLoss_db = avg_db + normal(generator); //test

	// calculate the receiving power at dist
	double Pr = Pr0 * pow(10.0, powerLoss_db/10.0);
  
	return Pr;
}

double getInterferencePercentage(double xt, double yt, double xr, double yr)
{
  int n = 200; //numero di iterazioni
  double rxThresh = 1.02802e-10;
  int counter = 0;
  for(int i = 0; i < n; i++)
  {
    double Pr = shadowingPr(xt,yt,xr,yr,0.2818,0.06);

    if(Pr >= rxThresh)
      counter++;
  }
  return ((double)counter / n * 100.0);
}

void shadowingTest(int n, double rxThresh)
{
  std::cout << "Shadowing test:" << std::endl;
  std::cout << "Used RXThresh: " << rxThresh << std::endl;
  std::cout << "dist\tfreq\ttotal" << std::endl;
  for (int i = 0; i < 300; i++)
  {
    int counter = 0;
    for(int j = 0; j < n; j++)
    {
      double Pr = shadowingPr((double)i,(double)i,0.0,0.0,0.2818,0.06);

      if(Pr >= rxThresh)
        counter++;
    }
    std::cout << i << "\t" << counter << "\t" << n << std::endl;
  }
  
  
}