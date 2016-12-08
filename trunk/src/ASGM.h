#ifndef ASGM_H
#define ASGM_H

// Instructions for implementing new model:
// (1) copy NewModel.[cpp,h] to Actualmodel.cpp.[cpp,h]
//     and globally replace "newmodel" => <actualmodel>
//     in both new files:
//      perl -i -p -e 's/newmodel/actualmodel/g' <files>
//      perl -i -p -e 's/NewModel/ActualModel/g' <files>
// (1a) add new files to the version control:
//       svn add <files> 
// (2) Create new simulation project (cpinp newmodel testmodel),
//     change model param files newmodel.NEW1, newmodel.NEW2 to appropr
//     names (e.g., testmodel.MOD1, testmodel.MOD2) and 
//     adapt parameters of these files to actual params of actual model
// (3) Implement cstr, get_modelparams, accSimple and acc in Actualmodel.[cpp,h]
//     (calc_eq() and all other methods should work unchanged!)
// (3a) if special BC needed, implement them in Boundary.cpp->updateUpstream
// (4) Introduce new model in the array of vehicles by editing Heterogen.[cpp,h]
//     don't forget to define vars+#include the new model class in Heterogen.h
//     and to adapt the method Heterogen.getExtension(..)
// (5) Also edit model numbers in simulation file newmodel.heterog
//     and in cstr. of NewModel.cpp accordingly;
//     update doku of ALL models in sim files (../../../mic_sim directory), e.g., by 
//     perl -i -p -e 's/100=NewModel/12=PT,100=NewModel/g' */*.heterog
//     (jan 2008: works again, now time tested!)
// (6) Incorporate new model in makefile (include Actualmodel.o)
// (7) rm *.o before recompiling (always do this if *.h changed!)

// (8) actualize script cpinp  (which cpinp => edit => search "for model in")


#include "MicroModel.h"
#include "CyclicBuffer.h"

class ASGM: public MicroModel
{

 public:
  
  ASGM(){;} //default constructor needed for ASGM[N] in Heterogen

  ASGM(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~ASGM(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(int it, double x, int s, int s_avg, double v, double alpha_v0); 

  void setSpeedlimit(double v0){
  // implementation: this->speedlimit=v0; //In MicroModel.h
    cout <<"Warning: ASGM.setSpeedlimit not yet implemented!"<<endl;
  }


 protected:

  // ASGM parameters (lveh in Vehicle class)
  // for compatibility all int quantities defined as double
  
  double v0;  //Desired velocity (cell units/time unit)
  int ml; // number of anticipated NNN leaders
  double pa; // "Troedelwahrsch." (slug prob) if v > v_avg
  double pb; // slug prob standing vehicles if stopping time>tc
  double pc; // slug prob for all other cases (NSM slug factor)
  double decel_a; // deceleration if sluggish behaviour of type a
  double decel_b; // deceleration if sluggish behaviour of type b or c
  double tc; // critical stopping time for applying slug type b

  // ASGM state variables (elapsed times, old values,
  // acceleration relaxations etc)

  int dt;          // =1; dt from constructur not needed for this model

  int tstopped;
  
 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // ASGM_H
