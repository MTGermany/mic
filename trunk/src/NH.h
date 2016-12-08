#ifndef NH_H
#define NH_H

// overview of actually implemented models: 
// Heterpgen.h and corresponding <Model>.cpp files

// Instructions for implementing new model:
// (1) copy NH.[cpp,h] to Actualmodel.cpp.[cpp,h]
//     and globally replace "nh" => <actualmodel>
//     in both new files:
//      perl -i -p -e 's/nh/actualmodel/g' <files>
//      perl -i -p -e 's/NH/ActualModel/g' <files>
// (1a) add new files to the version control:
//       svn add <files> 
// (2) Create new simulation project:
//       cpinp nh_ramp testmodel_ramp
//       cpinp nh_startStopLSA testmodel_startStopLSA
//     change model param files nh.NEW1, nh.NEW2 to appropr
//     names (e.g., testmodel.MOD1, testmodel.MOD2) and 
//     adapt parameters of these files to actual params of actual model
// (3) Implement cstr, get_modelparams, accSimple and acc in Actualmodel.[cpp,h]
//     (calc_eq() and all other methods should work unchanged!)
// (3a) if special BC and IC needed,
//      (e.g., for setting CA vehicles on integer positions)
//       implement them in Boundary.cpp->updateUpstream
//      and in RoadSection.cpp (IC)
// (3b) if special positional update needed (first-order pos for CA)
//   implement them in Vehicle.cpp  => generally look for isCA in this case)
// (4) Introduce new model in the array of vehicles by editing Heterogen.[cpp,h]
//     don't forget to define vars+#include the new model class in Heterogen.h
//     and to adapt the method Heterogen.getExtension(..)
// (5)  update doku of ALL models
//     in sim files (~/trafficSim/sources/mic_sim/ ), e.g., by 
//     perl -i -p -e 's/100=NH/14=ADAS,100=NH/g' */*.heterog
//     (apr 2011,may2012: works again, now time tested!)
// (6) Incorporate new model in makefile (include Actualmodel.o)
// (7) rm *.o before recompiling (always do this if *.h changed!)

// (8) actualize script cpinp  (which cpinp => edit => search "for model in")


#include "MicroModel.h"
#include "CyclicBuffer.h"

class NH: public MicroModel
{

 public:
  
  NH(){;} //default constructor needed for NH[N] in Heterogen

  NH(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~NH(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double sLead, double v, double dv, // implement also speedlimit
		   double alpha_v0, double alpha_T, int iveh); 

  //void setSpeedlimit(double v0){this->speedlimit=v0;} //In MicroModel.h


 protected:

  int variant;  // variant=0: NHM ("non-hypothetical steady-state model)
                // variant=1: TSM ("two-state safety-speed model")

  // NH parameters

  double v0;  //  Desired velocity [lveh/s] since dt=1 s
  double T;   // desired time gap [s]
  double b_defens; // deceleration [lveh/s^2]
  double pa;  // braking prob. [] for defensive driving, activated if dneff<v*T
  double pb;  // braking prob. [] slow-to-start (v=0, tstop>tc)
  double pc;  // braking prob. [] otherwise "driving"
  double g_safety; // safe addtl gap [lveh], part of desired gap
  double tc; // time stopping [s] after which slow-to-start sets in

  // NH state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt, here only dt=1 [s] OK
  int dtStopped; // time elapsed standing since stopped
 
 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // NH_H
