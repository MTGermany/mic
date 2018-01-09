#ifndef LCM_H
#define LCM_H

// overview of actually implemented models: 
// Heterogen.h and corresponding <Model>.cpp files


#include "MicroModel.h"
#include "CyclicBuffer.h"

class LCM: public MicroModel
{

 public:
  
  LCM(){;} //default constructor needed for LCM[N] in Heterogen

  LCM(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~LCM(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double vl, // implement also speedlimit
		   double alpha_v0, double alpha_T, bool debug); 

  //void setSpeedlimit(double v0){this->speedlimit=v0;} //In MicroModel.h


 protected:

  // LCM parameters

  double v0;
  double T;
  double s0;
  double a;           // max accel. (LCM) or alpha1 (DSM)
  double b;           // own braking decel [m/s^2]
  double B;           // leader (max?) braking decel [m/s^2]
  double bmax;

  int choice_variant; // {0: original LCM; 1: MLCM; 2: DSM}
  double delta;       // exponent (MLCM), sensitivity alpha2 (DSM}
  double SMup;        // upper DSM safety margin [1] (only used for DSM)
  double SMdown;      // lower DSM safety margin [1] (only used for DSM)

  // LCM state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  double dtNewTarget; // time elapsed since new target vehicle

 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // LCM_H
