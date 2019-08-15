#ifndef VEHICLE_H
#define VEHICLE_H


#include "ProjectParams.h"
#include "Fluct.h"
#include "MicroModel.h"
class CyclicBuffer; //forward declaration

class Vehicle
{ 
 public:
  
  Vehicle();

  Vehicle(double x, double v, ProjectParams* proj, Fluct fluct, 
	  MicroModel* p_model, int modelNumber, int setNumber);


  //<martin mai08> alpha's here are  global alpha's; that of
  // regionControl defined in this method
  void calcAcc(int it, int iveh, int imin, int imax, 
	       double alpha_v0_global, double alpha_T_global,
	       CyclicBuffer* const cyclicBuf);


  MicroModel* p_model;

  void updatePosVel();

  double getPos() const {return x;}             // front position
  double getOldPos() const {return xOld;}
  double getBackPos() const {return x-lveh;}
  double getVel() const {return v;}
  double getJerk() const {return(jerk);} // 2015 for some ctrl in RoadSection   
  double getAcc() const {return acc;} 
  //double getAccDes() const {return accDes;} //not yet implemented (2015-02)
  double getLength() const {return lveh;}


  // general-purpose

  void setVel(double newVel){v=newVel; vel_isExternallyControlled=true; }
  void setVelJump(double newVel){v=newVel; vel_isExternallyControlled=false; }
  void setPos(double newPos) {x=newPos;}  // front position


 
 
 
  int getModelNumber(){return modelNumber;}
  int getSetNumber(){return setNumber;}
 
  bool finiteTr(){return p_model->with_T_react();}
  int getID(){return id;}

  double get_veq(double rho){return p_model->get_veq(rho);}
  double getQmax(){return p_model->get_Qmax();}
  double get_rhoQmax(){return p_model->get_rhoQmax();}

  // <martin mai08> following public methods for regionControl

 
  void setAccExternal(double time);
  double get_timeBeginAccControl();
  void setVelByAcc(double newVel); // sets accExternal=(newVel-v)/dt;
                                                        // activates ctrl flag
  void setAcc(double acc){// may be overridden by calcAcc ("min calcAcc(..), accExternal")
      this->acc=acc; accExternal=acc;acc_isExternallyControlled=true;
  } 
  void setAlphaV0(double alpha_v0){this->alpha_v0=alpha_v0;}
  void setAlphaT(double alpha_T){this->alpha_T=alpha_T;}
  double getAlphaV0(){return this->alpha_v0;}
   
  void resetAcc(){acc_isExternallyControlled=false;} //<martin apr08>
  void resetVel(){vel_isExternallyControlled=false;} //<martin apr08>
  void resetAlphaV0(){alpha_v0=1;}//<martin apr08>
  void resetAlphaT(){alpha_T=1;} //<martin apr08/>
  void resetSpeedlimit(){p_model->setSpeedlimit(100000);}//<martin apr08>
  void setSpeedlimit(double newV0){p_model->setSpeedlimit(newV0);}

 private:

  double dt;  // from project parameters

  double x;  // front position
  double xOld;  // front position in last time step
  double v;
  double vOld; // speed last time step (2015-02-02)
  int modelNumber;
  int setNumber;
  bool isCAvehicle;
  double lveh;
  //bool finiteTr;

  int id; //arne 9-6-2005 fuer floatingCars

  //  int choice_model;
  //double dt_since_new_leadveh;

  double acc;   // actual acceleration
  //double accDes;   // MT 2015-02 desired acceleration before jerk etc//not yet implemented (2015-02)
  double jerk;  //arne, floating cars (17-8-04)

  Fluct* fluct;
  Fluct fluct2;

  double err_s; // eror in distance estimate (Wiener process)
  double err_dv; // eror in dv estimate (Wiener process)
  double err_a; // actual acceleration fluctuations (Wiener process, possibly tau_a=0)

  
  // martin may08>
  bool acc_isExternallyControlled;
  bool vel_isExternallyControlled;
  double accExternal;
  double timeBeginAccControl; // time of start of acceleration control
  double alpha_v0; // multiplicator for acceleration
  double alpha_T; // multiplicator for time headway
  
  //</martin may08>
  

};

#endif // VEHICLE_H
