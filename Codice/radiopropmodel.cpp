#include <math.h>
#include <stdio>

// use Friis at less than crossover distance
// use two-ray at more than crossover distance
//static double
double twoRayGround(double Pt, double Gt, double Gr, double ht, double hr, double L, double d)
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

double Friis(double Pt, double Gt, double Gr, double lambda, double L, double d)
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
	double M = lambda / (4 * PI * d);
	return (Pt * Gt * Gr * (M * M)) / L;
}

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
    /* 
	         4 * PI * hr * ht
	 d = ----------------------------
	             lambda
	   * At the crossover distance, the received power predicted by the two-ray
	   * ground model equals to that predicted by the Friis equation.
    */

	crossover_dist = (4 * PI * ht * hr) / lambda;

    /*
     *  If the transmitter is within the cross-over range , use the
     *  Friis equation.  Otherwise, use the two-ray
     *  ground reflection model.
    */

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
}