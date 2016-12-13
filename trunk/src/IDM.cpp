#ifndef IDM_CC
#define IDM_CC
 
// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

//own
#include "IDM.h"
#include "RandomUtils.h"
#include "general.h"
#include "InOut.h"

// ###################################################
// Constructors
// ###################################################

IDM::IDM(const char fname[])
{  
  initializeMicroModelVariables();  //arne -> set generic variables (lveh,Tr etc) to zero
  // otherwwise, some trouble possible; initialize... inherited from  MicroModel
  modelNumber=0; //arne 10-5-2005

  cout << "\nin IDM file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  calc_eq();

  // <MT 2016 action points; controlled by bmax<0> 
  useActionPoints=false;
  if(bmax<0){
    useActionPoints=true;
    da_c=-bmax;  //!! temporarily hard coded
    bmax =9;  //!! temporarily hard coded
    a_old=100; // such that action point is set in the first step
    da_thr=0;
  }
  // </MT 2016 action points>


  cout <<"End IDM file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}



//####################################################
void IDM::get_modelparams(const char fname[]){
//####################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&s0);
  inout.getvar(fp,&s1);
  inout.getvar(fp,&delta);    // IDM acceleration exponent

  inout.getvar(fp,&a);
  inout.getvar(fp,&b);

  inout.getvar(fp,&bmax);
  fclose(fp);

  if(false){
      cerr << "Sorry; only standard IDM (choice_variant=5) supported"
	   <<endl
	   <<"To run old projects, use the sources of src_16jun02"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;

  //###################################################
  // resignation effect !!!
  //###################################################
  alpha_a_resmin=0.5;  // 0.6 parameter! 0.2 ... 1
  alpha_v0_resmin=0.6;  // 0.8   0.2 ... 1
  alpha_T_resmax=1.4;  // 1.4

  alpha_adyn=1;      // initialization (leave to 1!)
  alpha_v0dyn=1;     
  alpha_Tdyn=1;     

  tau_res=240;

}



//################################################################
void IDM::calc_eq()
//################################################################

    // Calculates equilibrium velocity of NewModel:
    // Finds equilibrium velocities veqtab[ir] with simple numeric relaxation
    //  (no brain, but stable and simple method...)


  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin IDM.calc_eq()"<<endl;}

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
        double acc = accSimple(s,v_it,0.);
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

    calc_rhoQmax();  //assign values to Qmax, rhoQmax 
} // IDM::calc_eq()



//######################################
// acceleration of the IDM.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
//######################################

double IDM::accSimple(double s, double v, double dv){


  // central IDM acceleration
  // !! also/only change IDM::acc directly; 
  // !! as of Aug 2016, accSimple is not used for reasons of speed!!
  // !! IDM::acc is reference, not IDM::accSimple!

  double sstar  = s0+max(0.,T*v + 0.5*v*dv /sqrt(a*b)+s1*sqrt((v+0.000001)/v0)); 
  double a_wanted = a * ( 1.- pow((v/v0),delta) - SQR(sstar/s));

  // "IDM+" Model of Ros, TU Delft (Referee Oct 2012)
  //double a_wanted = a*min(1.- pow((v/v0),delta), 1 - SQR(sstar/s)); //!!

  return  (a_wanted>-bmax) ? a_wanted : -bmax;
}





//###################################################

double IDM::acc(int it, int iveh, int imin, int imax,
                double alpha_v0, double alpha_T,
		const CyclicBuffer* const cyclicBuf)
{
  //cout <<" in IDM.acc"<<endl;

  //##################################################
  // Local dynamical variables
  //##################################################

  double s= cyclicBuf->get_s(iveh); //xveh[iveh-1]-xveh[iveh]-lveh[iveh-1]
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double dv= v - cyclicBuf->get_v(iveh-1);//vveh[iveh];
  

    //###########################################
  // space dependencies modelled by speedlimits, alpha's
  //#############################################

  // martin mai08

  double Tloc  = alpha_T*T; 
  double v0loc=min(alpha_v0*v0, speedlimit);   // martin mai08
  double aloc  = a; //parameter

  //###############################################
  // resignation effect: multiply Tloc,v0loc, aloc 
  // with dynamical factors dependend on past history
  // (multipl => combination space inhomog+resign. effect OK)
  //###############################################
  
  if(false)
    { 
      // resignation effect:
      //if(RESIGNATION_ON==1){ // resignation effect
      double dt=0.2; //!!!
      double beta = 1.-exp(-dt/tau_res);
      double alpha_a_res=alpha_a_resmin+v/v0loc*(1-alpha_a_resmin);
      double alpha_v0_res=alpha_v0_resmin+v/v0loc*(1-alpha_v0_resmin);
      double alpha_T_res=alpha_T_resmax+v/v0loc*(1-alpha_T_resmax);
      
      alpha_v0dyn += beta*(alpha_v0_res-alpha_v0dyn);
      v0loc       *= alpha_v0dyn;
      
      alpha_Tdyn += beta*(alpha_T_res-alpha_Tdyn);
      Tloc       *= alpha_Tdyn;
      
      alpha_adyn  += beta*(alpha_a_res-alpha_adyn);
      aloc        *= alpha_adyn;
      
    }
  
  // no resignation effect; nothing to do; Tloc,vloc,aloc defined above
  
  //##################################################
  // actual IDM formula (directly recoded instead of using accSimple 
  // for speed)
  //##################################################
  
  double sstar = s0 + max(0., Tloc*v + s1*sqrt((v+0.0001)/v0loc) + 0.5*v*dv/sqrt(aloc*b));

  // Original IDM

  double a_wanted = (v<v0loc) ? aloc*( 1.- pow((v/v0loc),delta) - SQR(sstar/s))
    : aloc*( 1.- v/v0loc - SQR(sstar/s));


  // "IDM+" Model of Ros, TU Delft (Referee Oct 2012)

  //double a_wanted = aloc*min(1.- pow((v/v0loc),delta), 1 - SQR(sstar/s));//!!


  //###############################################
  // <MT 2016 action points>
  //###############################################

  if(useActionPoints){
    if(fabs(a_wanted-a_old)>da_thr){
      if(false){
	cout <<"new action point!"<<endl
	     <<" bmax="<<bmax
	     <<" a_wanted="<<a_wanted<<" a_old="<<a_old<<endl;
      }
      a_old=a_wanted; // use new acceleration for new action point
      da_thr=da_c*myRand(); // new threshold ~ U(0,da_c)
      //cout <<"da_thr="<<da_thr<<endl;
    }
    else{
      a_wanted=a_old; // no new action point -> do not change acceleration 
    }
  } // </MT 2016 action points>



  return  max(a_wanted, -bmax);
}

#endif // IDM_CC
