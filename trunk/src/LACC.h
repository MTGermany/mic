#ifndef LACC_H
#define LACC_H

//Chaozhe R He and Gabor Orosz, "Safey Guaranteed Connecterd Cruise
//Control", Eqs (4) with (5)-(7) => LACC.README
// mic modelNumber 20

// overview of actually implemented models: 
// Heterogen.h and corresponding <Model>.cpp files

// Instructions for implementing new model: NewModel.h


#include "MicroModel.h"
#include "CyclicBuffer.h"

class LACC: public MicroModel
{

 public:
  
  LACC(){;} //default constructor needed for LACC[N] in Heterogen

  LACC(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~LACC(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double dv, // implement also speedlimit
		   double alpha_v0, double alpha_T); 

  //void setSpeedlimit(double v0){this->speedlimit=v0;} //In MicroModel.h


 protected:

  // LACC parameters

  double v0;     //   \overline{v} in reference above
  double T;      //   1/kappa  in reference
  double s0;     //   h_st in reference
  double alpha;  //   as in reference; speed relax time = 1/alpha
  double lambda; //   FVDM relative speed sensitivity, = beta in reference

  // LACC state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  double dtNewTarget; // time elapsed since new target vehicle

 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // LACC_H
