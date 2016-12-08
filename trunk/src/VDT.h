#ifndef VDT_H
#define VDT_H

// VDT traffic model: "Variance-Determined Time gap"

#include "MicroModel.h"

class CyclicBuffer;
class ProjectParams;


class VDT: public MicroModel
{

 public:

  VDT(){;}

  VDT(const char vdtName[], int setNumber, double dt);

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T, 
	     const CyclicBuffer* const cyclicBuf);
  void setSpeedlimit(double v0){//<martin mai08>
    p_basemodel->setSpeedlimit(v0);}

  double get_rhomax(){return p_basemodel->get_rhomax();}
  double get_length(){return p_basemodel->get_length();}
  double get_Qmax(){return p_basemodel->get_Qmax();}
  double get_rhoQmax(){return p_basemodel->get_rhoQmax();}
  double get_veq(double rho){ return p_basemodel->get_veq(rho);}

 private:

  //ProjectParams* p_proj; // only for test reasons (need dt etc)

  MicroModel* p_basemodel;  // pointer to underlying base model for VDT
  int choice_basemodel;

  // new VDT parameters

  int nveh; // number of vehicles for determining variance
  double alphaT_max;    // alphaT=1 for perfectly smooth traffic
  double gamma;       // d(alphaT_VDT)/d(Varcoeff)
  //  => saturation varcoefficient: varcoeff_sat=(alphaT_max-1)/gamma

  double alphaT_VDT; // calculated actual T factor for VDT
  double dt;   // only for test purposes


  void get_modelparams(const char* fname);
  void initialize(const char projName[], int setNumber, double dt);
 
  // Methods performed by p_basemodel
  void calc_eq();  // performed by p_basemodel



};

#endif // VDT_H

