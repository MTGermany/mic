#ifndef GIPPS_H
#define GIPPS_H

// Created  11.Dec 2006 by Martin Treiber


#include "MicroModel.h"
class CyclicBuffer;

class Gipps: public MicroModel
{

 public:
  
  Gipps(){;} //default constructor needed for Gipps[N] in Heterogen

  Gipps(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~Gipps(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double dv, double v0, double T); 

  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: Gipps.setSpeedlimit not yet implemented!"<<endl;
  }


 protected:

  // Gipps parameters

  double v0;
  double T;
  double a;
  double b;
  double s0; // additional generalizing parameter; s0>=2 for safety

  // Gipps state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  double dtNewTarget; // time elapsed since new target vehicle

 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // GIPPS_H
