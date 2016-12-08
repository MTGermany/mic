#ifndef RAMP_H
#define RAMP_H


#include "ProjectParams.h"

//!!! RAMP_VEL_REDUCEFACTOR = relat. speed of inserted vehs
// => RoadSection.cpp

class Ramp
{ 

 public:

  Ramp();
  Ramp(char* projectName, ProjectParams* proj,
       int number, double xCenter, double l_ramp);
  //~Ramp();


  /** updates state of ramp
      @return internal variables accessible via get methods:
      <ul>
        <li> mergeIsOK, divergeIsOK: merges or diverges are desired and can be performed
        <li> changePos, position of merge (diverge), if mergeOK or divergeOK
        <li> leadVehIndex and divergeVehIndex
      </ul>
  */
      
  void update(double time, double x[], double s[], int imin, int imax);
  double getFlow(double time_s); 

  double getQueueLength(){return queueLength_veh;} 
  double getCenter(){return xCenter;} 
  double getLength(){return length_m;} 
  double getChangePos(){return changePos;}
  int getFrontVehIndex(){return leadVehIndex;}
  int getDivergeVehIndex(){return divergeVehIndex;}
  bool mergeIsOK(){return mergeOK;}
  bool divergeIsOK(){return divergeOK;}

 private:

  //ProjectParams* proj;
  double dt;
  double xmax;
  double xCenter;
  double changePos;
  int leadVehIndex;
  int divergeVehIndex;
  double length_m;
  int    n_times;
  double times[NBC];
  double Qrmp[NBC];
  double queueLength_veh;
  bool mergeOK;
  bool divergeOK;
  void tryMerge(double x[], double s[], double lvehMerge, int imin, int imax);
  void tryDiverge(double x[], int imin, int imax);

};

#endif // RAMP_H
