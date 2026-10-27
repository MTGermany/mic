#ifndef W26_CC
#define W26_CC


// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

//own
#include "W26.h"
#include "RandomUtils.h"
#include "general.h"
#include "InOut.h"

#include "CyclicBuffer.h"

W26::W26(const char* fname, double dt)
{
  initializeMicroModelVariables();  //arne -> init variables from MicroModel
  modelNumber=5;

  cout << "\nin W26 file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  calc_eq();
  cout <<"End W26 file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.5/lveh)="<<get_veq(0.5/lveh)<<endl;

}




//################################################################
void W26::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh);   // vehicle length [m] 

  int use_vl_int=1; // default settings
  indiff=1;
  
  inout.getvar(fp,&v0);     // desired speed [m/s]
  inout.getvar(fp,&s0);     // minimum gap [m]
  inout.getvar(fp,&T1);     // time gap lower bound following region RCF [s]
  inout.getvar(fp,&T2);     // time gap upper bound following region [s]
  inout.getvar(fp,&dv0);    // half-width rel speed [m/s] at lower RCF bound
  inout.getvar(fp,&tau_a);  // speed relaxation time [s]
  inout.getvar(fp,&a0);     // max accel. [m/s^2]
  inout.getvar(fp,&b);      // comf decel. [m/s^2]
  inout.getvar(fp,&bmax);   // max decel. [m/s^2] (overridden by system?)
  inout.getvar(fp,&use_vl_int); // 0=false: use s0+T1*v etc, 1=true: s0+T1*vl
  inout.getvar(fp,&indiff); // indifference factor in [0.1] (default 1)
  fclose(fp);

  use_vl=(use_vl_int==1);

  rhomax = 1./lveh;

}



//################################################################
void W26::calc_eq()
//################################################################

  // Calculates Steady-state speed of W26 

  // output:   
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 


{

  if(false){cout << "\nin W26.calc_eq()"<<endl;}

  // Steady-state speed table veqtab[ir]
  // assuming the middle of the indifference region


  // start with rho=0

  veqtab[0]         = v0;

  for(int ir=1; ir<=NRHO; ir++){
    double rho = rhomax*ir/NRHO;
    double s   = 1./rho - 1./rhomax;

    veqtab[ir] = min(v0, max(0., 2*(s-s0)/(T1+T2)));
    //cout<<"rho="<<rho<<" s="<<s<<" veqtab[ir]="<<veqtab[ir]<<endl;
  }
  
  calc_rhoQmax();  // Qmax, rhoQmax calc_rhoQmax inherited from MicroModel
} // W26::calc_eq()






//################################################################
// before 2026-10: acceleration of the HumanSchreck [PRL 92, 238702 (2004)]
// 2026-10: acceleration of the Wiedemann-26 model
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// vl=leader's speed

// no accSimple defined and used!
//######################################

double W26::acc(int it, int iveh, int imin, int imax,
			 double alpha_v0, double alpha_T,
			 const CyclicBuffer* const cyclicBuf)
{

  // bool debug=((it<4)&&(iveh<3));
  bool debug=((iveh==1)&&(it>=560) && (it<=650));
  //bool debug=false;

  //#############################################################
  // Local dynamical variables
  //#############################################################

  double s= cyclicBuf->get_s(iveh); //xveh[iveh-1]-xveh[iveh]-lveh[iveh-1]
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double dv= v - cyclicBuf->get_v(iveh-1);//vveh[iveh];

  //###########################################
  // space dependencies modelled by speedlimits, alpha's
  //#############################################

  double T1loc  = alpha_T*T1; 
  double T2loc  = alpha_T*T2; 
  double v0loc=min(alpha_v0*v0, speedlimit);   // martin mai08

  //###########################################
  // Actual model (part of "accSimple")
  //#############################################

  
  // central car-following regime (CF) delimitations

  double vref=(use_vl) ? v-dv : v;
  double dsCF=max(s0,(T2loc-T1loc)*vref); // !! max(s0,.)
  double sCF1=s0+T1loc*vref;
  double sCF2=sCF1+dsCF;
  double dvCF=dv0*(1+max((s-sCF1)/dsCF,0.));

   // free-flow (FF) regime

  bool isFF=(s>sCF2) && (abs(dv)<dvCF);
  double accFF=a0*(1-v/v0loc);


  // car-following (CF) regime
  
  bool isCF=(abs(dv)<dvCF) && (s>sCF1) && (s<sCF2);
  double accCF=min(accFF, 0.); // if v>v0, there is no indifference region
  
  // opening (OP) regime

  bool isOP=(dv<-dvCF);
  //double accOP=min(accFF, -dv/tau_a + (s-sCF1)/pow(tau_a,2));
  double accOP=accFF;

  // closing (CL) regime

  bool isCL=(s>=sCF1) && (dv>dvCF);
  //double accCL=-max(dv/tau_a, 0.5*pow(dv,2)/max(SMALL_VAL,s-s0));
  double accCL=-pow(0.5*v*dv/max(SMALL_VAL,s-s0),2)/b;

  // critical (CR) regime

  //bool isCR=(s<sCF1) && (dv>-dv0); // =(!(isFF||isCF||isOP||isCL))
  int theta_dv=(dv>=0) ? 1 : 0;
  //double accCR=-max(b, 0.5*pow(dv,2)/max(SMALL_VAL,s-s0)) * theta_dv;
  double accCR=-max(b, pow(0.5*v*dv/max(SMALL_VAL,s-s0),2)/b* theta_dv);

  // final acceleration
  
  double a_wanted=(isFF) ? accFF :
    (isCF) ? accCF :
    (isCL) ? accCL :
    (isOP) ? accOP : accCR;

  if(debug){
    string region=(isFF) ? "FF" : (isCF) ? "CF"
      : (isCL) ? "CL" : (isOP) ? "OP" : "CR";
    
    cout<<"W26.acc: it="<<it<<" iveh="<<iveh
	<<" s="<<s<<" v="<<v<<" dv="<<dv<<" sCF1="<<sCF1
	<<"region="<<region<<" a_wanted="<<a_wanted<<endl;

    /*
    cout<<" isFF="<<isFF<<" accFF="<<accFF<<endl
	<<" isCF="<<isCF<<" accCF="<<accCF<<endl
	<<" isOP="<<isOP<<" accOP="<<accOP<<endl
	<<" isCL="<<isCL<<" accCL="<<accCL<<endl
	<<" isCR="<<isCR<<" accCR="<<accCR<<endl;
    */

  }
    
  
  return  max(a_wanted, -bmax);
  
}



#endif // W26_CC

