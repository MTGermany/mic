#ifndef W26_H
#define W26_H

#include "MicroModel.h"
class CyclicBuffer;

class W26: public MicroModel
{
 public:
  
  W26(){;}

  W26(const char* projectName, double dt);

  //virtual ~W26(){;}

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(int s, int v, int vP, int vPP, bool debug);

  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: W26.setSpeedlimit not yet implemented!"<<endl;
  }
  

 protected:

  // old/replaced: HumanSchreck parameters [PRL 92, 238702 (2004)]
  // (2026-10): new Wiedemann-26 model by us TODO

  double v0;       //desired speed [m/s]
  double s0;       // minimum gap [m]
  double T1;       // time gap lower bound following region RCF [s]
  double T2;       // time gap upper bound following region [s]
  double dv0;      // half-width rel speed [m/s] at lower RCF bound
  double tau_a;    // speed relaxation time [s]
  double a0;       // max accel. [m/s^2]
  double b;        // comf decel. [m/s^2]
  double bmax;     // max decel. [m/s^2] (overridden by system?)
  bool use_vl;      // 0=false: use s0+T1*v etc, 1=true: s0+T1*vl
  double indiff;   // indifference level in CF (0=none, 1=full)
  
 private:

  void get_modelparams(const char* fname); 
  void calc_eq();

};

#endif // W26_H
