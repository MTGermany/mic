
//c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;


#include "MicroModel.h"

//################################################################
void MicroModel::writeFunddia(const char funddia_name[]){
//################################################################

  FILE * funddia_fp;              
  funddia_fp=fopen(funddia_name,"w");

  // write title line ( "#" = comment for gnuplot)

  fprintf(funddia_fp, 
     "# rho(1/km)  \t  s(m)\t   veq(km/h)   \t  Qeq(1/h)\t   \n");

  for(int ir=1; ir<=NRHO; ir++)
  {
    double rho              = rhomax*ir/NRHO;
    fprintf(funddia_fp, "%.1f       \t %.1f\t %.2f    \t %.1f \n",
      1000.*rho, 
      1./rho - 1./rhomax,
      3.6*veqtab[ir], 
      3600.*rho*veqtab[ir]);
  }

  fclose(funddia_fp);
}

//################################################################
//arne: not initialized variables could cause some trouble...
void MicroModel::initializeMicroModelVariables()
//################################################################
{
  T_react=0;
  T_react_span=0;
  lveh=0; 
  rhomax=0; 
  Qmax=0;   
  rhoQmax=0; 
  speedlimit=100000; //initially no speed limit 
}


//################################################################
double MicroModel::get_veq(double rho)
//################################################################
{
  return intp(veqtab, NRHO, rho, 0, rhomax);
}




/// calculate Qmax, and abszissa rhoQmax from veqtab (necessary for BC)

//################################################################
void  MicroModel::calc_rhoQmax ()
//################################################################
{
  int    ir   = 1;
  Qmax        = -1.;
  while(veqtab[ir] * rhomax*ir/NRHO > Qmax)
  {
    Qmax = veqtab[ir] * rhomax*ir/NRHO;
    ir++;
  }
  rhoQmax = rhomax*ir/NRHO;
}



  // intp interpolates the array tab with n+1 equidistant points
  //  in [xmin,xmax] at the location x; an error message is produced on
  //   attempting extrapolation 

//################################################################
double MicroModel::intp (const double tab[], int n,double x, 
			 double xmin, double xmax)
//################################################################
{
  double intp_value;
  double ir   = n*(x-xmin)/(xmax-xmin);
  int    i    = static_cast<int>(ir);
  double rest = ir-i;
  if ((i>=0) && (i<=n-1))  intp_value =  (1-rest) * tab[i] + rest*tab[i+1];
  else if (i==n) intp_value = tab[n];
  else {
    cout << "intp: index i = "<<i<<" (ir="<<ir<<") out of range\n";
    exit(1);
  }

  return(intp_value);
}
