#ifndef OVM_CC
#define OVM_CC
// c

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

//own
#include "InOut.h"
#include "general.h"
#include "OVM.h"
#include "CyclicBuffer.h"



OVM::OVM(const char fname[])
{ 
  initializeMicroModelVariables();  //arne -> init variables from MicroModel
  modelNumber=3; //arne 10-5-2005

  cout << "\nin OVM file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  calc_eq();
  cout <<"End OVM file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}



// parameters according to L.C. Davis, Physica A 319, 557 (2003)

//################################################################
void OVM::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 
  inout.getvar(fp,&choice_variant); // vehicle length 
  inout.getvar(fp,&s0);   // addtl. introduced distance (m) for veq=0

  inout.getvar(fp,&v0);
  inout.getvar(fp,&tau);
  inout.getvar(fp,&l_int);
  inout.getvar(fp,&beta);
	inout.getvar(fp,&T_react); //arne may07 (optional last parameter)
  fclose(fp);

  if(false){
      cerr << "Sorry; only standard OVM (choice_variant=5) supported"
	   <<endl
	   <<"To run old projects, use the sources of src_16jun02"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;
  //exit(-1);

}



//################################################################
void OVM::calc_eq()
//################################################################

    // Calculates equilibrium velocity of OVM and OVM with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin OVM.calc_eq()"<<endl;}

   // Find equilibrium velocities veqtab[ir] with simple relaxation
    // method: Just model for homogeneous traffic solved for 
    // the velocity v_it of one arbitrary vehicle
    //  (no brain, but stable and simple method...)

    double v_it=v0;           // variable of the relaxation equation
    int    itmax      = 30;  // number of iteration steps in each relaxation
    double dtmax      = 0.3*tau;    // iteration time step (in s) changes from
    double dtmin      = 0.1*tau; // dtmin (rho=rhomax) to dtmax (rho=0)


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
        double acc = accSimple(s, v_it, 1.);
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
    //exit(0);
} // OVM::calc_eq()



//######################################
// acceleration of the OVM.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
//######################################

double OVM::accSimple(double s, double v, double alpha_T)
{
  double a_wanted=0;
  double l_intLoc=l_int*alpha_T;
  double betaLoc=beta;

  // OVM acceleration

  if(choice_variant==0){
    // performance: could tanh(-beta) calc. in Cstr. if necessary
    //vopt = max( 0.5*v0*( tanh((s-s0)/l_intLoc-betaLoc) - tanh(-betaLoc)), 0.);

    //<martin mai08>: OVM/FVDM nun so skaliert, dass v0 tats Wunschgeschw
    double v0Prev   = v0/(1.+tanh(betaLoc));
    vopt =max(v0Prev*( tanh((s-s0)/l_intLoc-betaLoc) - tanh(-betaLoc)), 0.);
    a_wanted = (vopt-v)/tau;
  }


  // simplest model

  else if(choice_variant==1){
    double T=1.5;       // time headway
    vopt=max( min((s-s0)/T, v0), 0.);  // optimal velocity
    a_wanted = (vopt-v)/tau;
  }
  return(a_wanted);
}


//################################################################

double OVM::acc(int it, int iveh, int imin, int imax,
                double alpha_v0, double alpha_T,
								const CyclicBuffer* const cyclicBuf)
{
    //#############################################################
    // Local dynamical variables
    //#############################################################

	//arne: extend model to reaction times (may 07)
	//old version:
  //double s=cyclicBuf->get_s(iveh);
  //double v=cyclicBuf->get_v(iveh);
	
	double T_react_local = T_react;
	double s=cyclicBuf->get_s(iveh,it,T_react_local);
  double v=cyclicBuf->get_v(iveh,it,T_react_local);

  if(false /*it%1000==0*/)
    {
      vopt =max( v0*( tanh((s-s0)/l_int-beta) - tanh(-beta)), 0.);
      cout<<"OVM.acc: it="<<it<<" iveh="<<iveh<<" T_react_local="<<T_react_local
				<<" s="<<s<<" s_without_delay="<<cyclicBuf->get_s(iveh)<<" v="<<v
				<<" v_without_delay="<<cyclicBuf->get_v(iveh)<<" vopt="<<vopt
				<<" v0="<<v0	<<" accSimple="<<accSimple(s,v,0)<<endl;
    }

    return (accSimple(s,v,alpha_T));


}

#endif // OVM_CC

