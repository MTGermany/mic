#ifndef BIDM_H
#define BIDM_H



#include "MicroModel.h"
#include "CyclicBuffer.h"

class BIDM: public MicroModel
{

 public:
  
  BIDM(){;} //default constructor needed for BIDM[N] in Heterogen

  BIDM(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~BIDM(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double dv, // implement also speedlimit
		   double alpha_v0, double alpha_T, bool logging); 

  //void setSpeedlimit(double v0){this->speedlimit=v0;} //In MicroModel.h


 protected:

  // BIDM parameters see ~/papers/2016_JunFang_stochastic/

  double v0;
  double T;  // savety time gap [s]=T3 in paper 
  double T1; // lower boundary indifference region
  double T2; // upper boundary indifference region
  double s0;
  double a; 
  double b;
  double A;      // acceleration noise source [m^2/s^5] (*0.1 if dt=0.1)
  double lambda; //  speed adapt in indiff zone (=0 for orig BIDM)
  double bmax;

  // BIDM state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  double dtNewTarget; // time elapsed since new target vehicle
  double accOld;      // for the Brownian acceleration

 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // BIDM_H
