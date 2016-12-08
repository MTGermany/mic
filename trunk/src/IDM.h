// implement new model: copy NewModel.cc and NewModel.h and implement!
#ifndef IDM_H
#define IDM_H

#include "MicroModel.h"
#include "CyclicBuffer.h"


class IDM: public MicroModel
{
 public:

  IDM(){;}
  IDM(const char projectName[]);


  //virtual ~IDM(){;}

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* cyclicBuf);

  double accSimple(double s, double v, double dv); // only demo and for export!


 protected:


  // IDM parameters

  double v0;
  double T;
  double s0;
  double s1;
  double delta; 
  double a;
  double b;
  double bmax;

 private:

  // <MT 2016 action points> if bmax<0 !!! (then true bmax=-bmax)
  bool useActionPoints;
  double a_old; // = actual acceleration if |a_IDM-a_old|<da_thr
  double da_c; // maximum acceleration difference delta_a sim U(0,da_c)
  double da_thr; //actual acc difference threshold for new action point
  // </MT 2016 action points>

  double v0min;    // if resignation effect: minimal desired vel.
  double tau_res;   // if resignation effect: time constant of v0dyn
  double alpha_v0dyn;   // if resignation effect: 
  double alpha_Tdyn;   // if resignation effect: 
  double alpha_adyn;   // dynamic alpha_v0dyn=alpha_v0_resmin ... 1 etc
  double alpha_v0_resmin;
  double alpha_T_resmax;
  double alpha_a_resmin;  // multipoliczatively to static alpha_v0 etc

  void get_modelparams(const char* fname);
  void calc_eq();


};

#endif // IDM_H

