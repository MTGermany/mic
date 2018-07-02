#ifndef RANDOMUTILS_H
#define RANDOMUTILS_H

#include <math.h>
#include <stdlib.h>


//arne kesting (2006)
//static methods for random operations and basic distributions etc.

void setRandomSeed();
double myRand();
int myIntRand();

double uniform(double mean, double span);
double normalGaussian();
double gaussian(double mean, double sigma, double cutoff=3.0);

#endif // RANDOMUTILS_H
