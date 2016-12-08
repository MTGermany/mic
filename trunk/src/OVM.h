#ifndef OVM_H
#define OVM_H


#include "MicroModel.h"
class CyclicBuffer;

class OVM: public MicroModel
{

 public:
  
  OVM(){;}

  OVM(const char projectName[]);

  //virtual ~OVM(){;}

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double alpha_T); 
  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: OVM.setSpeedlimit not yet implemented!"<<endl;
  }


 private:

  // OVM parameters as given by L.C. Davis, Physica A 319, 557 (2003)
  // vopt=max(v0*( tanh((s-s0)/l_int-beta) - tanh(-beta)), 0)

  // other OVM function from IDM with delta=2:
  // vopt=v0/sqrt(1+SQR(v0*T/max(s-s0, 1e-10)))

  int choice_variant; //{original OVM, v0-T OVM, dv car-following model} 

  double v0;    // approximate desired velocity
  double s0;    // minimum gap (introduced by Treiber)
  double tau;   // velocity relaxation time; sometimes a=1/tau is used
  double l_int; // half-width of the "s-shape" of the tanh's
  double beta;  // interaction length scale (where vopt approx v0/2; 

  // Notice: time headway T of the order of 2*beta/v0

  double vopt;

 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // OVM_H

