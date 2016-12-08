#ifndef KERNER_H
#define KERNER_H

#include "MicroModel.h"
#include "ProjectParams.h"
class CyclicBuffer;

//arne: noch nicht drin was draufsteht !!! (November 05)
//Kerner model muss fuer mic noch angepasst werden
//Quellen von alter version siehe
// ~/trafficSim/sources/mic_jun02/src/models.cc,-h und Kernerparams.*
// Sim alte version siehe sim/ dir dieses Directories


class Kerner: public MicroModel{

public:
  
  Kerner(){;}
  Kerner(const char vlaName[], int setNumber, double dt);

  //virtual ~Kerner(){;}

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);
  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: Kerner.setSpeedlimit not yet implemented!"<<endl;
  }

  double get_rhomax(){return p_basemodel->get_rhomax();}
  double get_length(){return p_basemodel->get_length();}
  double get_Qmax(){return p_basemodel->get_Qmax();}
  double get_rhoQmax(){return p_basemodel->get_rhoQmax();}
  double get_veq(double rho){ return p_basemodel->get_veq(rho);}
 

 private:
  double dt;
  
  //ProjectParams* p_proj; // only for test reasons (need dt etc)
  
  MicroModel* p_basemodel;  // pointer to underlying base model for VDT
  int choice_basemodel;
  
  //internal model parameters
  
  void initialize(const char projName[], int setNumber, double dt);
 
  void get_modelparams(const char* fname);

  void calc_eq();

  double accSimple(double s, double v, double dv);

};

#endif // KERNER_H
