#ifndef _RADIO_PROPAGATION_MODEL_H
#define _RADIO_PROPAGATION_MODEL_H

double shadowingPr(double, double, double, double, double, double);
double friisPr(double, double, double, double, double, double);
void shadowingTest(int, double);
double getInterferencePercentage(double xt, double yt, double xr, double yr);
#endif