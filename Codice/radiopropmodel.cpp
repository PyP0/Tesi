#include <math.h>
#include <random>
#include <chrono>
#include "utility.h"
#include <iostream>

double Gt = 1.0;
double Gr = 1.0;
double L = 1.0;
double dist0_ = 1.0; //metri
double std_db_ = 4.0; //TODO: check
double pathlossExp_ = 2.0; //TODO: check
double Pt = 0.2818;
double lambda = 0.06;
double rxThresh = 7.13904e-07;//2.32413e-06; //1.02802e-10; //250m //1.69063e-06; //3m //

double getRXThresh()
{
	return rxThresh;
}
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

double shadowingPr(double dist)
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator (seed);
	std::normal_distribution<double> normal(0.0,std_db_);
	//std::lognormal_distribution<double> lognormal(0.0,pathlossExp_)

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
