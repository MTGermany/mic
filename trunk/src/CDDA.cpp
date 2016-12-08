#ifndef CDDA_CC
#define CDDA_CC

// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;


//own
#include "CDDA.h"
#include "general.h"
#include "InOut.h"
#include "CyclicBuffer.h"

// Instructions to incorporate new model: see CDDA.h

CDDA::CDDA(const char fname[], double dt)
{
  cout << "\nin CDDA file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);

  // rhomax = 1./lveh; // in get_modelparams

  this->dt=dt;
  dtFree=0;  // start with zero "free time"
  timeOld=0;
  calc_eq();
  
  cout <<"End CDDA file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.01)="<<get_veq(0.02)
       <<" get_veq(0.02)="<<get_veq(0.02)
       <<" get_veq(0.03)="<<get_veq(0.03)
       <<" get_veq(0.04)="<<get_veq(0.04)
       <<endl;

}




//################################################################
void CDDA::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&s0);
  inout.getvar(fp,&a);
  inout.getvar(fp,&tau);
  fclose(fp);

  rhomax = 1./lveh;

}



//################################################################
void CDDA::calc_eq()
//################################################################

    // Calculates equilibrium velocity of CDDA and CDDA with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin CDDA.calc_eq()"<<endl;}

   // Find equilibrium velocities veqtab[ir] with simple relaxation
    // method: Just model for homogeneous traffic solved for 
    // the velocity v_it of one arbitrary vehicle
    //  (no brain, but stable and simple method...)

    double v_it=v0;           // variable of the relaxation equation
    int    itmax      = 20;  // number of iteration steps in each relaxation
    double dtmax      = 0.2;    // iteration time step (in s) changes from
    double dtmin      = 0.001; // dtmin (rho=rhomax) to dtmax (rho=0)

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
        dtFree=2*T; // since here I don't want delay!
        double acc = accSimple(s,v_it,v_it,1,1);
        double dtloc = dtmax*v_it/v0 + dtmin; // it. step in [dtmin,dtmax]
	/*
       double sstar   = s0 + T * v_it + s1*sqrt((v_it+0.000001)/v0);

	// acceleration for various variants

        double acc = (s>s0) 
	  ? a * (1-pow(v_it/v0, 4) - SQR(sstar/s ))
          :0;
	*/

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
    cout <<"rhoQmax="<<rhoQmax<<endl;
    //exit(0);
} // CDDA::calc_eq()



//######################################
// acceleration of the CDDA.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
//######################################

double CDDA::accSimple(double s, double v, double vPrev, 
                       double alpha_v0, double alpha_T){


  // CDDA acceleration
  double dsNeutral=0.001;   // in m //!!!
  double safetyFactor=1.0; //!!!
  double s_kin = safetyFactor*(s0+0.5/a*( v*v - vPrev*vPrev));
  double v0loc=alpha_v0*v0;
  double Tloc=alpha_T*T;
  int accState = ( ((v<v0loc+0.00001)&&(s>s_kin+dsNeutral))
    ? 1 : ((s<s_kin-dsNeutral) ? -1 : 0));

  if (time>timeOld+0.5*dt){
    if(accState==1){dtFree+=dt;}
    else{dtFree=0;}
    timeOld=time;
  }

  if((accState==1)&&(dtFree<Tloc)){accState=0;}
  if(v>v0loc){accState=-1;}

  if(false){
   cout << "accSimple: s="<<s
        <<" v="<<v<<" vPrev="<<vPrev
	<<" v0loc="<<v0loc
	<<" Tloc="<<Tloc<<" s_kin="<<s_kin
	<<" dtFree="<<dtFree
        <<" acc="<<(accState*a)<<endl;
  }

  return accState*a;
}





//################################################################

double CDDA::acc(int it, int iveh, int imin, int imax,
		 double alpha_v0, double alpha_T,
		 const CyclicBuffer* const cyclicBuf)
{
  
  //#############################################################
  // Local dynamical variables
  //#############################################################
  
  time=it*dt;

  double s     = cyclicBuf->get_s(iveh);
  double v     = cyclicBuf->get_v(iveh);
  double vPrev = cyclicBuf->get_v(iveh-1);
  
  //#############################################################
  // actual CDDA formula
  // space dependencies modelled by alpha_v0, alpha_T
  //#############################################################
  
  return accSimple(s,v,vPrev,alpha_v0,alpha_T);
  
}



#endif // CDDA_CC
