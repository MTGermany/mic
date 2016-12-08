#ifndef GIPPS_CC
#define GIPPS_CC

// Created  11.Dec 2006 by Martin Treiber

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
#include "Gipps.h"
#include "CyclicBuffer.h"



Gipps::Gipps(const char fname[], double dt)
{
  cout << "\nin Gipps file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  this->dt=dt;
  this->T=dt;  // Gipps: dt=T=Tr=tau_relax!
  calc_eq();
  cout <<"End Gipps file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}




//########################################################
void Gipps::get_modelparams(const char fname[]){
//########################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&a);
  inout.getvar(fp,&b);
  inout.getvar(fp,&s0);
  fclose(fp);

  
  rhomax = 1./lveh;

}



//################################################################
void Gipps::calc_eq()
//################################################################

    // Calculates equilibrium velocity of Gipps and Gipps with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin Gipps.calc_eq()"<<endl;}

   // Find equilibrium velocities veqtab[ir] with simple relaxation
    // method: Just model for homogeneous traffic solved for 
    // the velocity v_it of one arbitrary vehicle
    //  (no brain, but stable and simple method...)

    double v_it=v0;           // variable of the relaxation equation
    int    itmax      = 100;  // number of iteration steps in each relaxation
    double dtmax      = 2;    // iteration time step (in s) changes from
    double dtmin      = 0.01; // dtmin (rho=rhomax) to dtmax (rho=0)


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
        double acc = accSimple(s,v_it,0.,v0,T);
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
        if(v_it<0) v_it=0;

        if(false){ 
          cout << "rho="<< rho<< " s="<<s
               <<" v0="<<v0<<" acc="<<acc<<" v_it="<<v_it<<endl;
	}
      }
      veqtab[ir] = v_it;

    }

    calc_rhoQmax();  // Qmax, rhoQmax 
} // Gipps::calc_eq()



//######################################
// acceleration of the Gipps.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
//######################################

double Gipps::accSimple(double s, double v, double dv, double v0, double T){


  // Gipps acceleration
  // typically safe for s0>=2
  // T*=1; //!!! Test T neq dt (Crash falls T<=0.95 dt)
  double vp=v-dv;

  double vsafe=-b*T+sqrt(b*b*T*T+vp*vp+2*b*max(s-s0,0.)); // safe velocity
  //vsafe *= (1.-0.005*exp(-pow((s-8)/3, 2))); //!!! test FD mit Wendepunkt
  double vnew=min(vsafe, min(v+a*T, v0));
  double a_wanted = (vnew-v)/T;
  return a_wanted;
}

//########################################################
// acc 
//########################################################

double Gipps::acc(int it, int iveh, int imin, int imax,
		     double alpha_v0, double alpha_T,
		     const CyclicBuffer* const cyclicBuf)
{

    //########################################################
    // Local dynamical variables
    //########################################################


  double s= cyclicBuf->get_s(iveh); //cyclicBuf->get_x(iveh-1) - cyclicBuf->get_l(iveh-1) - cyclicBuf->get_x(iveh);
  //xveh[iveh-1]-length[iveh-1]-xveh[iveh];
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double dv= v - cyclicBuf->get_v(iveh-1); //vveh[iveh];

  //#############################################################
  // space dependencies modelled by alpha_v0, alpha_T
  // (!!! watch for alpha_T: dt unchanged, possibly inconsistent!)
  //#############################################################
  
  double v0loc = alpha_v0*v0;
  double Tloc = alpha_T*T;
  
  //#############################################################
  // actual Gipps formula
  //#############################################################
  
  return  accSimple(s,v,dv,v0loc,Tloc);
  
}



#endif // GIPPS_CC

