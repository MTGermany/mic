#ifndef HDM_H
#define HDM_H

#include "IDM.h"
//#include "ProjectParams.h"
class CyclicBuffer;

class HDM: public IDM
{
 public:
  
  HDM(){;}
  HDM(const char fnameBase[], const char fname[], double dt);

  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation here !!!
  //use the default copy-constructor, deconstr and =operator
  //virtual ~HDM();

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);
  
  void setSpeedlimit(double v0){this->speedlimit=v0;} //<martin mai08>

  void init_T_react();

 

 private:

  // new HDM parameters
  
  double smax;     // maximum anticipation distance (500m)
  double s_stretch; // stretch factor for NNN interactions (1)
  int n_antimax; // maximum number of anticipated vehicles in dense traff(5)

  double renormFactorAntic[N_ANTICIPATION_MAX]; //static size because of copy cstr.

  double alpha_a_resmin; // min accel. factor due to resignation (0.6)
  double alpha_v0_resmin; // min v0- factor due to resignation (0.5)
  double alpha_T_resmax; // max increase factor of T due to resignation (1.5)
  double tau_res;        //  time scale for adaptation to traff environnement
  double v1rel;          // v-threshold for traffic quality=0 (in units v0)
  double v2rel;          // v-threshold for traffic quality=1 (in units v0)
  //double T_react_span;   //arne febr 06 (T_react distribution) --> in MicroModel

  double n_temp_antic; // temporal anticipation (arne feb 06) default is 1
  bool with_temp_anticipation(){ return( n_temp_antic>0 ); }

  // State and history variables
  //!! create cyclic buffer

  double dt_since_new_leadveh; 
  double dt;
  double a_old;
  double s_old;

  // resignation-related dyn. variables

  double beta;       // 1-exp(dt/tau_res)
  double alpha_adyn;  
  double alpha_v0dyn;     
  double alpha_Tdyn; 

  // position and history-dependend dynamical "model parameters"
    
  double Tloc;   // desired min. time headway
  double v0loc;  // desired velocity
  double aloc;   // desired max. acceleration

  void get_modelparams(const char* fname);
  void get_IDMmodelparams(const char* fname);
  void calc_eq();
  double accFree(double v);
  double accTwoVehInt(double s, double v, double dv, double a, double T_react_loc);
 


};

#endif // HDM_H

