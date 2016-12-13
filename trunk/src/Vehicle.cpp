// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;


#include "Vehicle.h"
#include "RandomUtils.h"
#include "general.h"
#include "MicroModel.h"
#include "CyclicBuffer.h"


// #############################################################
// Constructors
// #############################################################

Vehicle::Vehicle()
{
  //cout << "in Vehicle default cstr"<<endl;
}

Vehicle::Vehicle(double x, double v, ProjectParams* proj, Fluct fluctIn,
                 MicroModel* p_model, int modelNumber, int setNumber)
{
  // iveh andert sich!
  this->x=x;
  this->xOld=x;
  this->v=v;
  this->vOld=v; // 2015-02-02
  this->dt=proj->get_dt();
  // this->choice_model=choice_model;
  this->p_model=p_model;
  this->lveh=p_model->get_length();

  alpha_v0=alpha_T=1;
  acc_isExternallyControlled=false; // martin apr 08
  vel_isExternallyControlled=false; // martin apr 08

  //this->fluct=fluctIn;
  
  //this->fluct=new Fluct(proj->get_projectName(),proj->get_dt());
  //  fluct2=Fluct(dt, fluctIn->Q_acc, fluctIn->varcoeff_s, fluctIn->tau_s, fluctIn->invttc_dv, fluctIn->tau_dv);
  fluct2=Fluct(fluctIn);
  
  this->modelNumber=modelNumber;
  this->setNumber=setNumber;
  //!! martin07: eigentlich hat Gipps selbe Regel wie CA
  // (sollte also isCAvehiclesein). Aber andere Regel ist effektiver/realistischer!
  // fuer specialIssue brauche ich Gipps als !isCAvehicle
  
  this->isCAvehicle=((modelNumber==5)||(modelNumber==11)||(modelNumber==13)||(modelNumber==15));
  //this->finiteTr=finiteReactionTime;

  id=myIntRand(); //arne 9-6-2005

  //dt_since_new_leadveh=0;
  acc=0;
  //accDes=0; // not yet implemented (2015-02)
  jerk=0;
  err_s=0;
  err_dv=0;
  err_a=0;

  if(false)
    {
      cout <<"Vehicle Cstr: modelNumber="<<modelNumber<<" lveh="<<lveh<<endl;
      cout <<" p_model->get_length()="<<p_model->get_length()<<endl;
    }
} 





// #####################################################
// calcAcc
// #####################################################


  //<martin mai08> alpha's here are  global alpha's;
// the local class members alpha_v0 and alpha_T
// controlled by ExternalControl

void Vehicle::calcAcc(int it, int iveh, int imin, int imax, 
		      double alpha_v0_global, double alpha_T_global, 
		      CyclicBuffer* const cyclicBuf)
{

  double xMe   = (finiteTr()) ? cyclicBuf->get_x(iveh,  it, p_model->get_T_react())   : cyclicBuf->get_x(iveh);
  double xLead = (finiteTr()) ? cyclicBuf->get_x(iveh-1,it, p_model->get_T_react())   : cyclicBuf->get_x(iveh-1); 
  double vMe   = (finiteTr()) ? cyclicBuf->get_v(iveh,  it, p_model->get_T_react())   : cyclicBuf->get_v(iveh);
  double vLead = (finiteTr()) ? cyclicBuf->get_v(iveh-1,it, p_model->get_T_react())   : cyclicBuf->get_v(iveh-1);
  double lengthLead = cyclicBuf->get_l(iveh-1);


  bool check=false;
  if(check)
    {
      cout <<" Vehicle::calcAcc: it="<<it<<" iveh="<<iveh
	   <<" xveh[iveh]="<<xMe
	   <<" xveh[iveh-1]="<<xLead
	   <<" vveh[iveh]="<<vMe
	   <<" vveh[iveh-1]="<<vLead
	   <<endl;
    }

  // Calculate errors in estimated distance, err_s, 
  // and estimated velocity difference, dv, by a stochastic Wiener process

  //arne: error applies only to direct predecessor!!!
  //not the n anticipated vehicles (in HDM , VDT)

  // fluct-> oder fluct2.
  // !!! MT aug 12: Bug: DOS for distance_error (v error and acc error works)
  // something obscure with cyclicBuffer?? error attributes to reference iveh &
  // but passed is iveh-1
  // possibly only effective if difference of errors is called (s but not v,dv?)

  if(fluct2.isActive())
    { 
      // else the default zeros are used
      double gap=xLead-lengthLead-xMe;
      double dv=vMe-vLead;

      // fluct2.update(...) ensures s>0, v>=0, v[iveh-1]>=0 after errors
      fluct2.update(gap,vMe,dv); 
      err_s  = fluct2.get_distance_error();
      err_dv = fluct2.get_dv_error(); 
      err_a  = fluct2.get_acc_error();
      
      if(gap < 2.0) err_a = min(err_a, 0.);  //!!!
   
      //arne: diesen hack mit einem eingriff in die daten 
      // und das "undo" hinterher uebertrage ich in den cylicBuffer!!!!!!!!!
      //xveh[iveh-1] += err_s;  // 4.12.03: iveh => iveh-1
      //vveh[iveh-1] -= err_dv;

      cyclicBuf->setError_x(iveh-1, err_s);
      cyclicBuf->setError_v(iveh-1, -err_dv);
      
      if(false){
	  cout<<" Vehicle.calcAcc: after fluct2.update: "
	      <<" it="<<it<<" iveh="<<iveh
	      <<" gap (without error)="<<gap
	      <<" err_s="<<err_s
	      <<" err_dv="<<err_dv
	      <<endl;
      }
    } // if fluct2.isActive()
  
  double oldAcc=acc;
  
  double alpha_v0_loc=((alpha_v0>1+SMALL_VAL)||(alpha_v0<1-SMALL_VAL))
    ? alpha_v0 : alpha_v0_global; // if alpha_v0 neq 1, then regionControl
  double alpha_T_loc=((alpha_T>1+SMALL_VAL)||(alpha_T<1-SMALL_VAL))
    ? alpha_T : alpha_T_global; // if alpha_T neq 1, them regionControl
  
  //########################################################
  // main acceleration calculation!!!
  //########################################################

  acc=p_model->acc(it, iveh, imin, imax, alpha_v0_loc, alpha_T_loc, cyclicBuf);

  //##############################

  // calc and add acceleration fluctuations
  // and undo error in x and v --> arne: jetzt in cyclic buffer

  if(fluct2.isActive())
    {
      //xveh[iveh-1] -= err_s;  
      //vveh[iveh-1] += err_dv; 
      cyclicBuf->setError_x(iveh-1, 0);
      cyclicBuf->setError_v(iveh-1, 0);
  
      acc += err_a;
    }
  
  // martin mai08
  // if acceleration (or velocity) externally set, apply the minimum
  // of calculated acc and external acc (minimum to avoid crashes)
  
  if(acc_isExternallyControlled){
    acc=min(acc,accExternal);
    // if(it*dt>380){cout <<"vehicle externally controlled! accExternal="<<accExternal<<endl;}
  }

  // </martin mai08>
  
  jerk = (acc-oldAcc)/dt;

  if(check)
    {
      cout<<" Vehicle.calcAcc: iveh="<<iveh<<"acc="<<acc<<" and err_a="<<err_a<<endl;
    }

  if(!((acc>-10000)&&(acc<1000)))
    {
      cout<<"!((acc>-10000)&&(acc<10))="<<(!((acc>-10000)&&(acc<10)))<<endl;
      cout<<"Vehicle.calcAcc: acc="<<acc
	  <<" => something went terribly wrong!!!"<<endl;
      cout<<"it="<<it<<" iveh="<<iveh<<endl;
    }
}


// ################################################
// updatePosVel
// ################################################

void Vehicle::updatePosVel()
{
  
  //cout <<"Vehicle.updatePosVel: getModelNumber()="<<getModelNumber()<<endl;
  // increment first s; then increment s with NEW v (2th order: -0.5 a dt^2)

  xOld=x;
  vOld=v;

  if((modelNumber!=4)&&(modelNumber!=10)&&(!isCAvehicle)){ // beim FPE-Model v<0 zulassen!
    if(v<0){ v=0;} 
    // bug fix 2015-02-02
    if(vel_isExternallyControlled){
      acc=(vOld-v)/dt;
    }
    double advance = (acc*dt >= -v)
      ? dt * v + 0.5 * acc*SQR(dt)
      : -0.5 * SQR(v)/acc; 
    x+=advance;
    v+=dt*acc;
    if(v<0){ v=0; acc=0;}
  }

  // MT dec16 Gipps model position updated with new speed 
  // (separate block because of runtime)

  else if(modelNumber==10){
    if(v<0){ v=0;} 
    if(vel_isExternallyControlled){
      acc=(vOld-v)/dt;
    }
    double advance = (acc*dt >= -v)
      ? dt * v + acc*SQR(dt) // =dt*(v+acc*dt)=dt*vNew
      : -0.5 * SQR(v)/acc; 
    x+=advance;
    v+=dt*acc;
    if(v<0){ v=0; acc=0;}
  }

  // MT dec16 Laval's parsmionious CF model (PCF model)
  // (separate block because of runtime)

  else if(modelNumber==17){
    if(vel_isExternallyControlled){
      acc=(vOld-v)/dt;
    }
    double advance = (acc*dt >= -v)
      ? dt * v + 0.5*acc*SQR(dt) // =dt*(v+acc*dt)=dt*vNew
      : -0.5 * SQR(v)/acc; 
    x+=advance;
    v=advance/dt;
    if(v<0){ v=0; acc=0;}
  }


// should not be necessary in model 15 since there casts to int. However, 
// change in x update necessary

  else if (isCAvehicle){
    if(modelNumber==15){
      double lVehAdhoc=7.5; // hack!!
      v  = lVehAdhoc*static_cast<int>((v+dt*acc)/lVehAdhoc+0.5); 
      x  = lVehAdhoc*static_cast<int>((x+dt*v)/lVehAdhoc+0.5);
    }
    else{
      v  = static_cast<int>(v+dt*acc+0.5); 
      x  = static_cast<int>(x+dt*v+0.5);
    }
  }
  else{
    double advance = dt*v + 0.5*acc*SQR(dt);
    x+=advance;
    v+=dt*acc;
  }
}

//#########################################



//void Vehicle::setPos(double acc){..} // in .h file
//void Vehicle::setAcc(double acc){..} // in .h file


// martin mai08 following specific for regionControl
// initialize time counter for external accelerations and flag, 
// if uncontrolled previously

void Vehicle::setAccExternal(double time){
  if(!acc_isExternallyControlled){
    cout <<"Vehicle.initAccExternal: New control introduced"
	 <<" at t= "<<time<<" x= "<<getPos()<<endl;
    timeBeginAccControl=time;
    acc_isExternallyControlled=true;
  }
  // else do nothing
}

double Vehicle::get_timeBeginAccControl(){
  return timeBeginAccControl;
}


void Vehicle::setVelByAcc(double newVel){
    accExternal=(newVel-v)/dt; // then after acceleration step new velocity
    vel_isExternallyControlled=true;
}
