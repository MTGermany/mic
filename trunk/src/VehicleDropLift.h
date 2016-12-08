#ifndef VEHICLEDROPLIFT_H
#define VEHICLEDROPLIFT_H


#include "Vehicle.h"
#include "ProjectParams.h"
#include "Heterogen.h"
#include "Fluct.h"

class VehicleDropLift{ 

 public:

  VehicleDropLift(){;}
  VehicleDropLift(const char projectName[], ProjectParams* proj, Heterogen* het);


  bool noVehDroppedOrLifted(){return ((n_drop==-1)&&(n_lift==-1));} 
  void update(Vehicle veh[], int& imin, int& imax, double time);

 private:
  double tstart;
  double dt;
  char projectName[199];
  Heterogen* het;
  ProjectParams* proj;

  int n_drop;
  int n_lift;
  double times_drop[NTYPEMAX+1];
  double times_lift[NTYPEMAX+1];
  int iveh_back[NTYPEMAX+1];
  int iveh_lift[NTYPEMAX+1];
  int type_drop[NTYPEMAX+1];
  double s_back[NTYPEMAX+1];
  double v_drop[NTYPEMAX+1];

};

#endif // VEHICLEDROPLIFT_H
