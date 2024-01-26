#ifndef FVDMMODEL_H
#define FVDMMODEL_H


#include "MicroModel.h"
class CyclicBuffer;


class FVDM: public MicroModel
{

 public:
  
  FVDM(){;}

  FVDM(const char projectName[]);

  //virtual ~FVDM(){;}

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(int choice_variant,
		   double s, double v, double dv, double alpha_T); 
  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: FVDM.setSpeedlimit not yet implemented!"<<endl;
  }


 private:

  // OVM parameters as given by L.C. Davis, Physica A 319, 557 (2003)
  // vopt=max(v0*( tanh((s-s0)/l_int-beta) - tanh(-beta)), 0)

  // other OVM function from IDM with delta=2:
  // vopt=v0/sqrt(1+SQR(v0*T/max(s-s0, 1e-10)))

  // Overview of variants: 
  //   0 =original OVM function, 1=triang. OV, 
  //   2 =3phase,
  //   3 =asymmetric original, 4=asymmetric triang, 
  //   5,6 =FVDM with Delta v/s instead of Delta v and original/triang OV function

  int choice_variant;  // {original OVM function, secBased OVM function}
  double v0;    // approximate desired velocity
  double s0;    // minimum gap (introduced by Treiber)
  double tau;   // velocity relaxation time; sometimes a=1/tau is used
  double l_int; // distance at vopt=v0/2 (e.g.,=25m)
  double l_intLoc; // 1./alpha_T*l_int 
  double beta;  // T (variant=1) or measure for tangent of V(s) at s=0
                // (of the order of 1!)
  double lambda;  // sensitivity to velocity differences (new w/resp to OVM)
  double R;  // range parameter

 private:

  void get_modelparams(const char* fname);
  void calc_eq();

};

#endif // FVDMMODEL_H
