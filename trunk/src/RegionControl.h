

#ifndef REGIONCONTROL_H
#define REGIONCONTROL_H

#include "./constants.h"
#include "./ProjectParams.h"
#include "./Vehicle.h"

class RegionControl{ 

 public:

  RegionControl();
  RegionControl(char* projectName, ProjectParams proj, int regionIndex,
		double xmin, double xmax,
		double tmin, double tmax);
  
  void update(Vehicle veh[], int imin, int imax, double time);
  
  double getAcc(double time); 
  double getVel(double time); 
  double getSpeedlimit(double time);
  double getAlphaT(double time); 
  double getAlphaV0(double time); 
 

  bool isInRegion(double x, double t){
    return (x>=xmin)&&(x<=xmax)&&(t>=tmin)&&(t<=tmax);
  }

  bool inTimeInterval(double t){
    return ( (t>=tmin)&&(t<=tmax));
  }

  bool isUpstream(double x){return (x<xmin); }
  bool isDownstream(double x){return (x>xmax);}
  bool inSpaceInterval(double x){return ( (x>=xmin)&&(x<=xmax)); }



    // only by isolated triggering tmax<=tmin in .ctrlRegion
  
  bool singleVehControlled(){return useSingle;}
  bool singleVehJustFound(){return vehicleJustFound;}
  bool singleVehControlEnded(double time);
  int get_singleVehIndex(){return ivehSingle;}

  // general type of control
  
  bool ctrlBySpeedlimit(){return (useSpeedlimit);}
  bool ctrlByAcc(){return useAcc;}
  bool ctrlByVel(){return useVel;}
  bool ctrlByAlphaV0(){return useAlphaV0;}
  bool ctrlByAlphaT(){return useAlphaT;}

  void writeInfo();
  
 private:

  void setControl(Vehicle& veh);
  void resetControl(Vehicle& veh);

  ProjectParams proj;
  static const int NDATAMAX=20001;

  bool useSpeedlimit;  // prescribe speed limit (.speedlimitRegion%i)
  bool useAcc;      // prescribing acceleration (.accRegion%i)
  bool useVel;      // prescribing speed limit (.velRegion%i)
  bool useAlphaT;       // multiplicator T (.alpha_TRegion%i) 
  bool useAlphaV0;       // multiplicator v0 (.alpha_v0Region%i) 

  // the controlled region

  double tmin,tmax; 
  double xmin,xmax;

  // the actual time and region index (e.g., 3 if controlled by accRegion3)
  double time;

  int regionIndex;
  
  // only for individual control

  bool vehicleFound;
  bool vehicleJustFound; // true only once, then new instance of ExternalControl generated
  int ivehSingle;
  bool useSingle;   // prescribe v in generic format (.<one of 4 ctrl possibilities>%i)

  
  int    n_acc, n_vel, n_speedlimit, n_alpha_v0, n_alpha_T;
  double times[NDATAMAX];
  double locations[NDATAMAX];
  double acc[NDATAMAX];
  double vel[NDATAMAX];
  double speedlimit[NDATAMAX];
  double alphaT[NDATAMAX];
  double alphaV0[NDATAMAX];

  char in_fnames[1000];
};

#endif // REGIONCONTROL_H
