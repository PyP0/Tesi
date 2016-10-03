#ifndef _RADIO_PROPAGATION_MODEL_H
#define _RADIO_PROPAGATION_MODEL_H

double getRXThresh();
double twoRayGroundPr(double, double, double, double, double, double, double);
double friisPr(double, double, double, double, double, double);
double shadowingPr(double);

#endif