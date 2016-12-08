#ifndef CDDA_H
#define CDDA_H

#include "MicroModel.h"
class CyclicBuffer;

class CDDA: public MicroModel
{
public:
  
  CDDA(){;}
  
  CDDA(const char projectName[], double dt);

  //virtual ~CDDA(){;}

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double vPrev, 
                   double alpha_v0, double alpha_T);


 private:

  // CDDA parameters

  double v0;  //Desired velocity (m/s)
  double T;   //Acceleration reaction time = max. time headway (s)
  double s0;  //min. bumper-to-bumper distance (m)  
  double a;   //Acceleration and deceleration (m/s^2)
  double tau; //pedestrian crossing time (not yet used)

  // CDDA state variables

  double dt;     // proj.dt
  double dtFree; // time elapsed since "free" state began
  double timeOld;
  double time;

  // helper methods

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // CDDA_H
