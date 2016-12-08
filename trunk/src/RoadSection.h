#ifndef ROADSECTION_H
#define ROADSECTION_H

#include <string>

#include "Vehicle.h"
#include "ProjectParams.h"
#include "MicroModel.h"
#include "Heterogen.h"
#include "Ramp.h"
#include "Boundary.h"
#include "ExternalControl.h"
#include "RegionControl.h"
#include "FloatingCars.h"
#include "VehicleDropLift.h"
#include "Detector.h"
#include "Fluct.h"
#include "Consumption.h"
#include "TrajectoryLOS.h"

class CyclicBuffer; //declaration (definition not needed)

class RoadSection
{ 

 public:
  
  RoadSection(const char* projectName, ProjectParams* proj);
  virtual ~RoadSection();

  void initialize(int choice_init);
  void update(int it);
  void updateEuler(int it); // MT Apr 13; see .cpp  for documentation
  void updateBallistic(int it); // MT Apr 13; see .cpp  for documentation
  void updateTrapezoid(int it); // MT Apr 13; see .cpp  for documentation
  void updateRK4(int it); // MT Apr 13; see .cpp  for documentation

  void writeAll(int it);

 private:

  // overall parameters

  ProjectParams* proj;
  //Fluct fluct;
  Consumption* fuelConsumption;

  bool calcFuelConsumption;
  bool useEngineDataSheet; // if false and calcFuel...=true,  analytic calculation
  char projectName[199];

  double xmin;
  double xmax;
  double dt;
  double tstart;
  int counter_crashs;

  //accumulated data 
  double accum_fuel_l; 
  double accum_travtime_s;
  double accum_acc; //experimental
  double accum_jerk;//experimental

  // flow-conserving bottlenecks

  int n_v0;
  double x_v0[NXMAX];
  double arr_alpha_v0[NXMAX];
  int n_T;
  double x_T[NXMAX];
  double arr_alpha_T[NXMAX];

  // onramps

  int n_ramps;
  Ramp* ramp[NRMPMAX];
  double x_ramp[NRMPMAX];
  double l_ramp[NRMPMAX];
  int counter_rmpveh;  // counts all entering ramp vehicles, incremented by set_in(...); used for write_3Dmic(...); hack; later on every ramp its counter

  // virtual detectors and positions where to save real rho,v etc

  int n_det;
  double x_det[NDETMAX];
  Detector* detector[NDETMAX];

  // boundary conditions

  Boundary* boundary_up;
  Boundary* boundary_down;

  // selected vehicles are externally controlled

  int n_vehControlled;
  ExternalControl* externalControl[NRMPMAX];
  int i_vehControlled[NRMPMAX];

  // selected regions are externally controlled <martin mai08>

  int n_regionsControlled;
  RegionControl regionControl[NRMPMAX];
  int initVehIndex[NRMPMAX]; // only for special case individual control

  
  // add/remove externally some vehicles to simulate reaction to lane changes

  VehicleDropLift* vehDropLift;

  // writing FCD, micro and macro output 
  // from "virtual" floating cars and detctors

  FloatingCars* floatCars;

  TrajectoryLOS* trajLOS;


  // vehicles: counted from imin (first) to imax (last)
  int imin;
  int imax; 

  Vehicle veh[NVEHMAX];  //!!! check if time consuming
  MicroModel* p_models[NTYPEMAX+1];  // array of pointers to micromodels

  double distrSpan_v0_T;
  double* alpha_T_distr;  //arne, feb06
  double* alpha_v0_distr;  

  Heterogen* het;  // contains statistics of vehicle population


  //26.1.
  // circular buffer for past states (for models with reaction times)
  // to avoid conflict at lanechanges, also otherw. superfluous lengthBuffer

  //int nt_past;            // how many steps in past? (must be global!!)
  //  bool someVehHaveReact;  // true if finite reaction times

  CyclicBuffer* cyclicBuf;

  // variables controlling the output
  // arne: static const double ist nicht erlaubt ;-(

  int ndtout_3D;
  int nxout_3D;
  int ndtout_2D;
  int ndtout_FCD;
 

  // bool detectorsExist;
  bool flowCtrlExists;
  bool externalCtrlExists;
  bool regionCtrlExists;

	//########## fileHandles (oct06) 
		
  FILE* FH_fuel;
  FILE* FH_tt;
  FILE* FH_3dmic;
  FILE* FH_3dmac;
  vector<FILE*> FH_2dmac_vec;
	

	
  //########## private methods 

  // calculates right-hand side k1[], k2[] etc based on the state vector y
  // MT mai13
  void calc_rhs(int it, double alpha_v0_global, double alpha_T_global,
		double rhs[]);

  void setState(const double ynew[], int it);
  void checkCorrectNegativeSpeeds(const double ynew[],const double yold[], double dtOldNew);

  void write_3Dmic(int it, int dnveh); // every dnth veh
  void write_3Dmac(int it);
  void write_2Dmac(int it);
  void write_TravTimes(int it);
	void calc_DrivingComfort(int it, double tau_0, double& accSqr, double& jerkSqr); //Oct06
  void write_FuelConsumption(int it);
  
  
  //########################################################################
  /// get flow-conservative road inhomogeneities.
  ///
  /// They are implemented by overall multiplication factors
  /// <ul>
  /// <li> alpha_v0 for the desired velocities of the vehicle population,
  /// <li> or alpha_T for the time headway of the vehicle population.
  ///</ul>
  /// Note that alpha_v does NOT implement  speed limits 
  /// (which only apply to the faster vehicles)
  /// Input: file <projectName>.alpha_v0 or <projectName>.alpha_T
  /// Output: list of positions x_v0[] and corresponding values alpha_v0[]
  /// and same for T.
  /// 
  /// Default (no such files): No flow-conservative road inhomogeneities.
  //########################################################################

  void get_flowconsInhomog(string varname, double x_tab[], double var_tab[],int& n_tab);
    
  
  //########################################################################
  /// looks for files <projName>.engineData_*, etc and
  /// sets up  fuel consumption calculations if files found
  //########################################################################
      
  void  get_fueldata(string carName);
      
  //########################################################################
  /// get_ramps():
  /// if file .rmps exists, read relevant infrastructure file
  /// and the files for the ramp flows
  /// default: no ramps
  //########################################################################
	
  void get_ramps();

  //########################################################################
  // get virtual detectors (if file <project>.detectors exist)
  //########################################################################
	
  void get_detectors();
	
	
  //########################################################################
  // insert or remove vehicles
  //########################################################################
	
  void set_in(double pos, int leadVehIndex);
  void take_out(int vehIndex);
    
  //#######################################################################
  /// get info about external influences 
  /// (sudden braking of certain vehicles etc)
  //#######################################################################

  void get_externalControl();
  void get_regionControl(); // amrtin mai08

  void generateMacroFields(int choice_init,double rho[], double Q[],  int nxMacro);
};

#endif // ROADSECTION_H
