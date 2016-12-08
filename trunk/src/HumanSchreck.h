#ifndef HUMANSCHRECK_H
#define HUMANSCHRECK_H

#include "MicroModel.h"
class CyclicBuffer;

class HumanSchreck: public MicroModel
{
 public:
  
  HumanSchreck(){;}

  HumanSchreck(const char* projectName, double dt);

  //virtual ~HumanSchreck(){;}

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(int s, int v, int vP, int vPP, bool debug);

  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: HumanSchreck.setSpeedlimit not yet implemented!"<<endl;
  }
  

 protected:

  // HumanSchreck parameters [PRL 92, 238702 (2004)]


  int v0;     //Desired velocity (cell units/time unit)
  int vfast;  //velocity where traffic is considered not cong.
  int vslow;  //velocity where traffic is considered very cong.
  int a;      //acceleration (cell units/time unit^2)
  int D;      //braking deceleration (cell units/time unit^2)
  int gadd;   //"addtl. safety gap" in the defensive state
  int tsafe;  //"maximal time step" (int erval?) for opt. state
  double p0;     //"Troedelwahrsch." for standing vehicles
  double pd;     //"Troedelwahrsch." for v>vslow

 private:

  double dt;     //Time step (model parameter in CA's!)
  void get_modelparams(const char* fname); 
  void calc_eq();

  // Eqs (1),(3)

  bool isSafe(int s, int v, int vP, int vSafe,bool driverIsOptimistic, bool debug);

};

#endif // HUMANSCHRECK_H
