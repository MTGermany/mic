#ifndef FPE_H
#define FPE_H



#include "MicroModel.h"
#include "general.h"
#include "constants.h"
#include "ProjectParams.h"

class CyclicBuffer;

class FPE: public MicroModel
{
 public:
  
  FPE(){;}

  FPE(const char projectName[], ProjectParams* proj);

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double sback, double dt); 


  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: FPE.setSpeedlimit not yet implemented!"<<endl;
  }


 protected:

  // FPE parameters

  double iso;    // isotropy coefficient (0: car-folling ... 1: mom. cons)

  double v0;     // Desired velocity (m/s)
  double tau;    // velocity relaxation time (s)
  double a_s0;   // interact. deceleration at s0 (m/s^2)
  double s0;     // reference distance (m)
  double gamma;	 // interaction exponent a_int=a_s0*(s0/s)^gamma

  double Qfluct; // fluctuation intensity (m^2/s^3)

 private:

  double dt;
  void get_modelparams(const char* fname);
  void calc_eq();
  bool isRing;
  double roadLen; 
};

#endif // FPE_H
