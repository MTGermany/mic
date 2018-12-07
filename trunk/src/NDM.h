#ifndef NDM_H
#define NDM_H

// overview of actually implemented models: 
// Heterogen.h and corresponding <Model>.cpp and sometimes <Model>.README files


#include "MicroModel.h"
#include "CyclicBuffer.h"

class NDM: public MicroModel
{

 public:
  
  NDM(){;} //default constructor needed for NDM[N] in Heterogen

  NDM(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~NDM(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double dv, // implement also speedlimit
		   double alpha_v0, double alpha_T); 

  //void setSpeedlimit(double v0){this->speedlimit=v0;} //In MicroModel.h


 protected:

  // NDM parameters

  double v0;
  double T;
  double s0;
  double tau; 
  double bmax;

  // NDM state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  double dtNewTarget; // time elapsed since new target vehicle

 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // NDM_H
