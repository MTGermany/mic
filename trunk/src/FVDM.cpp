#ifndef FVDMMODEL_CC
#define FVDMMODEL_CC
// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

//own
#include "general.h"
#include "InOut.h"
#include "FVDM.h"
#include "CyclicBuffer.h"


FVDM::FVDM(const char fname[])
{ 
  initializeMicroModelVariables();  //arne -> init variables from MicroModel
  modelNumber=7;

  //init variables from MicroModel:
  T_react=0; //bug, was not initialized!!! arne 10-5-2005
  lveh=0; 
  rhomax=0; 
  Qmax=0;   
  rhoQmax=0; 


  cout << "\nin FVDM file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  calc_eq();
  cout <<"End FVDM file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}



// parameters according to L.C. Davis, Physica A 319, 557 (2003)

//################################################################
void FVDM::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 
  inout.getvar(fp,&choice_variant); // {original OVfun, triang. OV, 3phase, 
                                    // 3=asymm orig, 4=asymm triang, 5=Delta v/s}

  inout.getvar(fp,&s0);   // addtl. introduced distance (m) for veq=0

  inout.getvar(fp,&v0); // martin mar08: Now always des. vel
  inout.getvar(fp,&tau);
  inout.getvar(fp,&l_int);
  inout.getvar(fp,&beta);

  inout.getvar(fp,&lambda);
	
  inout.getvar(fp,&T_react); //arne may07 (optional last parameter)
  fclose(fp);

  rhomax = 1./lveh;
  //exit(0);
}



//################################################################
void FVDM::calc_eq()
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

  if(false){cout << "\nin FVDM.calc_eq()"<<endl;}

   // Find equilibrium velocities veqtab[ir] with simple relaxation
    // method: Just model for homogeneous traffic solved for 
    // the velocity v_it of one arbitrary vehicle
    //  (no brain, but stable and simple method...)

    double v_it=v0;           // variable of the relaxation equation
    int    itmax      = 100;  // number of iteration steps in each relaxation
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
        double acc = accSimple(choice_variant, s, v_it, 0., 1.);
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
      cout <<"FVDM.calc_eq(): veqtab[ir]="<<veqtab[ir]<<endl;

    }

    calc_rhoQmax();  // Qmax, rhoQmax 
} // FVDM::calc_eq()



//######################################
// acceleration of the FVDM=full velocity difference model FVDM.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// choice_variant:
//   0 =original OVM function, 1=triang. OV, 
//   2 =3phase,
//   3 =asymmetric original, 4=asymmetric triang, 
//   5,6 =FVDM with Delta v/s instead of Delta v and original/triang OV function
//######################################

double FVDM::accSimple(int choice_variant,
			double s, double v, double dv, double alpha_T){
  //  cout <<" lambda="<<lambda<<endl;

  double a_wanted=0;
  double vopt=0;
  double l_intLoc=l_int*alpha_T;
  //double betaLoc=beta*alpha_T;
  double betaLoc=beta;

  if(l_intLoc<1e-6){l_intLoc=1e-6;}

  // FVDM acceleration

  // standard OVM function
  // performance: could tanh(-beta) calc. in Cstr. if necessary
  //vopt(s)  = v0Prev *( tanh((s-s0)/l_int-beta) + tanh(beta)-0.1/s)

  if((choice_variant==0) || (choice_variant==3) || (choice_variant==5)){
    //<martin mai08>: OVM/FVDM nun so skaliert, dass v0 tats Wunschgeschw
    double v0Prev   = v0/(1.+tanh(betaLoc));
    vopt =max(v0Prev*( tanh((s-s0)/l_intLoc-betaLoc) - tanh(-betaLoc)), 0.);
    if (false){
      cout <<"l_int="<<l_int<<" vopt="<<vopt<<endl;
    }
  }


  // Triangular OVM function

  else if ((choice_variant==1) || (choice_variant==4) || (choice_variant==6)){
    double T=beta;       // time headway
    vopt=max( min((s-s0)/T, v0), 0.);  // optimal velocity
  }

  // "Three-phase"  OVM function

  else if(choice_variant==2){
    double diffT=0.*pow(max(1-v/v0, 0.0001), 0.5);
    double Tmin=l_int+diffT;       // min time headway
    double Tmax=beta+diffT;       // max time headway
    double Tdyn=(s-s0)/max(v, SMALL_VAL);
    vopt=(Tdyn>Tmax)
      ? min((s-s0)/Tmax, v0) : (Tdyn>Tmin)
      ? min(v+0.,v0) : (Tdyn>0)
      ? min((s-s0)/Tmin, v0) : 0;
  }
 
  // full FVDM  acceleration

  if (choice_variant<=1){
    a_wanted = (vopt-v)/tau - lambda * dv;
  }
  else if (choice_variant==2){
    a_wanted = (vopt-v)/tau - lambda * v*dv/max(s-1.0*s0, SMALL_VAL);
    a_wanted=min(a_wanted, 5.);
  }
  
  // original  FVDM model MT jan2010
  else if((choice_variant==3) || (choice_variant==4)){
    a_wanted = (vopt-v)/tau - lambda * ((dv>0) ? dv : 0);
  }

  else if((choice_variant==5) || (choice_variant==6)){ // MT 2020
    double T=beta;       // time headway
    //a_wanted = (vopt-v)/tau - 0.6*min(dv,0.) - lambda * max(dv,0.)/max(s, SMALL_VAL);
    //a_wanted = min( (v0-v)/tau, (vopt-v)/tau - 0.6*min(dv,0.) - lambda * max(dv,0.)/max(s, SMALL_VAL));
    //a_wanted = min( (v0-v)/tau, (vopt-v)/tau - lambda * dv/max(s, SMALL_VAL));
    //a_wanted = min( (v0-v)/tau, (vopt-v)/tau - lambda * dv* v0*T/max(s,v0*T));
    a_wanted = (vopt-v)/tau - lambda * dv/max(1.,s/(v0*T));
    //a_wanted = (vopt-v)/tau - lambda * dv/max(1.,s/l_intLoc);
    //a_wanted = min( (v0-v)/tau, (vopt-v)/tau - lambda * dv/max(s,20.));
    //a_wanted = (vopt-v)/tau - lambda * dv/max(s, SMALL_VAL);


  }

  if (a_wanted>100){
    cerr<<"error: acc>100!"<<" vopt="<<vopt
	<<" v="<<v<<" tau="<<tau<<" dv="<<dv<<" lambda="<<lambda<<endl;
     exit(-1);
  }

  return max(a_wanted,-20.);
}




//#########################################################

double FVDM::acc(int it, int iveh, int imin, int imax,
		  double alpha_v0, double alpha_T,
		  const CyclicBuffer* const cyclicBuf)
{

  //#############################################################
  // Local dynamical variables
  //#############################################################
  
	//arne mai07, reaction time for FVDM
  //double s  = cyclicBuf->get_s(iveh);
  //double v  = cyclicBuf->get_v(iveh);
  //double dv = v- cyclicBuf->get_v(iveh-1);
	
	double T_react_local = T_react;
	double s=cyclicBuf->get_s(iveh,it,T_react_local);
  double v=cyclicBuf->get_v(iveh,it,T_react_local);
	double dv = v-cyclicBuf->get_v(iveh-1,it,T_react_local);
	
  if(false)
    {
      cout<<"FVDM.acc: s="<<s<<" v="<<v<<" dv="<<dv
          <<" accSimple="<<accSimple(choice_variant,s,v,0,alpha_T)<<endl;
    }

    return (accSimple(choice_variant,s,v,dv,alpha_T));
}



#endif // FVDMMODEL_CC

