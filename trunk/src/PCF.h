#ifndef PCF_H
#define PCF_H


#include "MicroModel.h"
#include "CyclicBuffer.h"

class PCF: public MicroModel
{

 public:
  
  PCF(){;} //default constructor needed for PCF[N] in Heterogen

  PCF(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~PCF(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);


  //void setSpeedlimit(double v0){this->speedlimit=v0;} //In MicroModel.h


 protected:

  // PCF parameters

  double v0;
  double T;
  double s0;
  double tau; 
  double Q;
  int choice_model;


  // PCF state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  double dtNewTarget; // time elapsed since new target vehicle

 private:

  void get_modelparams(const char* fname);
  void calc_eq();

  // implements also speedlimit if applicable; it for debugging purposes
  double accSimple(double s, double v, double dv, 
		   double alpha_v0, double alpha_T, 
		   double x, int it, int iveh, double dt);

};

#endif // PCF_H
