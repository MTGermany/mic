#ifndef LCM_CC
#define LCM_CC

// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//alternatively there is <cstdio> which declares
//everything in namespace std
//but the explicit "using namespace std;" puts
//everything in global namespace

// c++ 
#include <iostream>
using namespace std;


//own
#include "general.h"
#include "InOut.h"
#include "LCM.h"
#include "CyclicBuffer.h"


// Instructions to incorporate new model: see LCM.h


LCM::LCM(const char fname[], double dt)
{
  initializeMicroModelVariables();  //arne -> set generic variables (lveh,Tr etc) to zero
  // otherwwise, some trouble possible; initialize... inherited from  MicroModel
  modelNumber=18; // set it correctly!!

  cout << "\nin LCM file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams

  this->dt=dt;
  calc_eq();
  cout <<"End LCM file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}



//################################################################
void LCM::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh);    // vehicle length 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&s0);
  inout.getvar(fp,&a);       // max accel. (LCM) or alpha1 (DSM)
  inout.getvar(fp,&b);       // own braking decel [m/s^2]
  inout.getvar(fp,&B);       // leader (max?) braking decel [m/s^2]
  inout.getvar(fp,&bmax);
  
  inout.getvar(fp,&choice_variant); //  {0: original LCM; 1: MLCM; 2: DSM}

  inout.getvar(fp,&delta);    //  exponent (MLCM), sensitivity alpha2 (DSM}
  inout.getvar(fp,&SMup);     //  upper DSM safety margin [1] (only used for DSM)
  inout.getvar(fp,&SMdown);   //  lower DSM safety margin [1] (only used for DSM)
  fclose(fp);

  if(false){
      cerr << "Sorry; only standard LCM (choice_variant=5) supported"
	   <<endl
	   <<"To run old projects, use the sources of src_16jun02"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;

}



//################################################################
void LCM::calc_eq(){
//################################################################

    // Calculates equilibrium velocity of LCM:
    // Finds equilibrium velocities veqtab[ir] with simple numeric relaxation
     //  (no brain, but stable and simple method...)

  // output:   veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)



    if(false){cout << "\nin LCM.calc_eq()"<<endl;}

    double v_it=v0;           // variable of the relaxation equation
    int    itmax      = 20;  // number of iteration steps in each relaxation
    double dtmax      = 2;    // iteration time step (in s) changes from
    double dtmin      = 0.02; // dtmin (rho=rhomax) to dtmax (rho=0)


    // start with rho=0

    veqtab[0]         = v0;

    for(int ir=1; ir<=NRHO; ir++)
    {
      double rho = rhomax*ir/NRHO;
      double s   = 1./rho - 1./rhomax;

      // start iteration with equilibrium velocity for the previous density
      v_it             = veqtab[ir-1];
      if(false) cout << endl;

      for (int it=1; it<=itmax; it++){
        double acc = accSimple(s,v_it,v_it,1,1,false);
        double dtloc = dtmax*v_it/v0 + dtmin; // it. step in [dtmin,dtmax]

	// actual relaxation 

        v_it            += dtloc*acc;
        if((v_it<0)||(s<s0)) v_it=0;

        if(false){ 
          cout << "rho="<< rho<< " s="<<s
               <<" v0="<<v0<<" acc="<<acc<<" v_it="<<v_it<<endl;
	}
      }
      veqtab[ir] = v_it;

    }

    calc_rhoQmax();  // Qmax, rhoQmax 
} // LCM::calc_eq()



//######################################
// acceleration of the LCM.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double LCM::accSimple(double s, double v, double vl,
		      double alpha_v0, double alpha_T, bool debug){


  double v0loc=alpha_v0*v0; //generic parameter
  double Tloc=alpha_T*T;  //generic parameter
  double aloc  = a;     //generic parameter

  double sstar=max(s0,s0+v*Tloc+0.5*(SQR(v)/b-SQR(vl)/B));
  double a_DSM=0;
  if(choice_variant==2){ // DSM
    double alpha1=a;      // redefinition for DSM
    double alpha2=delta;  // redefinition for DSM
    double safetyMargin=1-sstar/s;
    a_DSM=(safetyMargin>SMup)
      ? alpha1 * (safetyMargin-SMup) : (safetyMargin<SMdown)
      ? alpha2 * (safetyMargin-SMdown)
      : 0;
  }

  double a_wanted=(choice_variant==0)
    ? aloc*(1-v/v0loc-exp(1-s/sstar)) : (choice_variant==1)
    ? aloc*(1-v/v0loc-exp(1-pow(s/sstar,delta))) 
    : a_DSM;

  if(debug){
    cout <<"LCM.accSimple: s="<<s<<" v="<<v<<" vl="<<vl
	 <<" sstar="<<sstar<<" pow(.)="<<pow(s/sstar,delta)
	 << endl;
  }
  return max(a_wanted, -bmax);
}

//#############################################################
// acc 
//#############################################################

double LCM::acc(int it, int iveh, int imin, int imax,
		     double alpha_v0, double alpha_T,
		     const CyclicBuffer* const cyclicBuf){

    //#############################################################
    // Local dynamical variables
    //#############################################################


  double s=  cyclicBuf->get_s(iveh); //xveh[iveh-1]-xveh[iveh]-lveh[iveh-1]
  double v=  cyclicBuf->get_v(iveh); //vveh[iveh];
  double vl= cyclicBuf->get_v(iveh-1);
  
  //bool debug=(iveh==2) ? true : false;
  bool debug=false;
  
  //#############################################################
  // actual LCM formula
  //#############################################################
  
  return  accSimple(s,v,vl,alpha_v0,alpha_T,debug);
  
}



#endif // LCM_CC

