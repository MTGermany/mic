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
       <<" get_veq(0.5/lveh)="<<get_veq(0.5/lveh)<<endl;

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

  // MT feb20: estimated leader's deceleration
  // Activates full Gipps with theta=tau/2 if bl exists and is >0
  // (<=0 safe flag for using simplified Gipps)

  bl=-1;     // default use simplified Gipps
  theta=0.5; // default for full Gipps (in units of T)

  inout.getvar(fp,&bl); 
  inout.getvar(fp,&theta); 

  // revert to default if theta not in param file
  // comment out if theta=0 is an option (=>use simplified bsafe, full bfree)

  // if(theta<=1e-6){theta=0.5;}

  cout<<"bl="<<bl<<endl;
  cout<<"theta="<<theta<<endl; 
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
// simple version typically safe for s0>=2, full version ...

// difference full vs simplified Gipps:
//  (a) T->0.5*T+theta, 
//  (b) addtl term -b*v*T (NOT -(0.5*T+theta*v), here T real react time)
//  (c) vl^2/b -> vl^2/bl

// limiting case theta->0 and bl=b: FD of full and simplified Gipps the same
// but not the dynamics (term b*v*T always stays in the sqrt)

//######################################

double Gipps::accSimple(double s, double v, double dv, double v0, double T){

  // T*=1; //!!! Test T neq dt (Crash falls T<=0.95 dt)

  double vl=v-dv;
  bool useSimple=(bl<=0); // Use simplified Gipps of book 2013
                          // Otherwise use full Gipps of original publication

  double vsafe=(useSimple) // safe speed in next step t+T
    ? -b*T+sqrt(SQR(b*T)+2*b*max(s-s0,0.)+SQR(vl))
    : -b*T*(0.5+theta)+sqrt(SQR(b*T*(0.5+theta))+2*b*max(s-s0,0.)
			    +SQR(vl)*b/bl-b*v*T);

  double vfree=(useSimple)
    ? min(v+a*T, v0)
    : v+2.5*a*(1-v/v0)*sqrt(0.025+v/v0)*T;

  double vnew=min(vsafe, vfree);
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
  //cout <<"Gipps.accSimple: Tloc="<<Tloc<<endl;

  //#############################################################
  // actual Gipps formula
  //#############################################################
  
  return  accSimple(s,v,dv,v0loc,Tloc);
  
}



#endif // GIPPS_CC

