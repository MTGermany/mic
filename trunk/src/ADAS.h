#ifndef ADAS_H
#define ADAS_H


#include "MicroModel.h"
#include "CyclicBuffer.h"

class ADAS: public MicroModel
{

 public:
  
  ADAS(){;} //default constructor needed for ADAS[N] in Heterogen

  ADAS(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~ADAS(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double dv, // implement also speedlimit
		   double alpha_v0, double alpha_T); 

  //void setSpeedlimit(double v0){this->speedlimit=v0;} //In MicroModel.h


 protected:

  // ADAS parameters (see project ADAS_ramp in mic_sim/ADAS/ )

  double v0;
  double T;
  double s0;
  double ra;
  double c1;
  double c2;
  double eta;
  double ds;

  // ADAS state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  double dtNewTarget; // time elapsed since new target vehicle

 private:

  void get_modelparams(const char* fname);
  void calc_eq();

  void testModelEquations(const char fname[]);

};

#endif // ADAS_H
