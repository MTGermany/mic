#ifndef NewModel_H
#define NewModel_H

// overview of actually implemented models: 
// Heterpgen.h and corresponding <Model>.cpp files

// Instructions for implementing new model:
// (1) copy NewModel.[cpp,h] to Actualmodel.cpp.[cpp,h]
//     and globally replace "newmodel" => <actualmodel>
//     in both new files:
//      perl -i -p -e 's/newmodel/actualmodel/g' <files>
//      perl -i -p -e 's/NewModel/ActualModel/g' <files>
// (1a) add new files to the version control:
//       git add <files> 
// (2) Create new simulation project (in mic-sim directory ):
//       cp -r newmodel/newmodel_clean testmodel
//       cd testmodel
//       cpinp newmodel_ramp testmodel_ramp
//       cpinp newmodel_startStopLSA testmodel_startStopLSA
//       rm newmodel*
//     change model param files newmodel.NEW1, newmodel.NEW2 to appropr
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
//     in sim files (~/trafficSim/mic_sim/ ), e.g., by 
//     perl -i -p -e 's/100=NewModel/17=PCF,100=NewModel/g' */*.heterog
//     perl -i -p -e 's/100=NewModel/17=PCF,100=NewModel/g' */*/*.heterog
//     (apr 2011,may2012,aug2014,dec2016: works again, now time tested!)
//     (if too much:)
//     perl -i -p -e 's/17=PCF\,17=PCF/17=PCF/g' */*.heterog */*/*.heterog
// (6) Incorporate new model in makefile (include Actualmodel.o)
// (7) actualize script cpinp  (which cpinp => edit => search "for model in")
//     and diffProjects.sh 
// (8) rm *.o (make clean) before recompiling (always do this if *.h changed!)


#include "MicroModel.h"
#include "CyclicBuffer.h"

class NewModel: public MicroModel
{

 public:
  
  NewModel(){;} //default constructor needed for NewModel[N] in Heterogen

  NewModel(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~NewModel(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double dv, // implement also speedlimit
		   double alpha_v0, double alpha_T); 

  //void setSpeedlimit(double v0){this->speedlimit=v0;} //In MicroModel.h


 protected:

  // NewModel parameters

  double v0;
  double T;
  double s0;
  double s1;
  double delta; 
  double a; 
  double b;
  double bmax;

  // NewModel state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  double dtNewTarget; // time elapsed since new target vehicle

 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // NewModel_H
