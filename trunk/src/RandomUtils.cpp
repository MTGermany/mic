//C
#include <stdio.h>
#include <math.h>


//C++
#include <iostream>
using namespace std;
#include "RandomUtils.h"
#include <sys/time.h>

//##################################################################
// RandomUtils collection, arne 2006
//##################################################################


//test: drand48() --> ist  langsamer!


//##################################################################
// own seed and rand calls
// since time(NULL) gives time in seconds 
// (may be the same if sim less than 1s apart), I use another command
// "gettimeofday" in <sys/time.h> (info from the internet) 
//##################################################################

void setRandomSeed()
{
  printf("# general.setRandomSeed: call srand to set random seed for pseudo random generator\n");
  //cout <<"time(NULL)="<<time(NULL)<<endl;

  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

  cout << "ms="<< ms<<" ms%1000000="<< ms%1000000<<endl;
  //srand ( time(NULL) );
  srand ( ms%1000000 );

}



double myRand()
{
  return( static_cast<double>(rand())/RAND_MAX );
  //return drand48(); //is slower!
}

int myIntRand()
{
  return rand();
}



//##################################################################
// uniform distribution G(min,max) with variation sqrt((b-a)/12)
// 
//##################################################################

double uniform(double min, double max)
{
  return( (max-min)*myRand() + min );
}



//################################################################
// taken from Numerical Recipies
// returns a normally distributed variable with zero mean and unit
// variance 
// (2D-gauss integral in polar coordinates)
//################################################################

double normalGaussian()
{
  double rsq,v1,v2;
  do
    {
      v1=2.0*myRand()-1.0;	// pick two uniform numbers in the square
      v2=2.0*myRand()-1.0;	// extending from -1 to +1 in each direction,
      rsq=v1*v1+v2*v2;		// see if they are in the unit circle
    } 
  while(rsq >= 1.0 || rsq == 0.0);	// and if they are not, try again.
  double fac=sqrt(-2.0*log(rsq)/rsq);
  
  // Now make the box-Muller transformation to get two normal
  // devites. Return one and save the other for next time.
  //gset=v1*fac;
  return v2*fac;
}

//################################################################
/// normal distributed variable with given mean and variation
/// cut-off is optional, standard-cutoff is 3*sigma
///
//################################################################

double gaussian(double mean, double sigma, double cutoff)
{
  if(sigma==0) return(mean);
  double z;
  do
    {
      z = normalGaussian();
    } 
  while(fabs(z)>cutoff*sigma);
  return (sigma*z+mean);
}


