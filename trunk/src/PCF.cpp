#ifndef PCF_CC
#define PCF_CC

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
#include "RandomUtils.h"
#include "InOut.h"
#include "PCF.h"
#include "CyclicBuffer.h"
//#include "constants.h"
//#include "general.h"


// Instructions to incorporate new model: see PCF.h


PCF::PCF(const char fname[], double dt)
{
  initializeMicroModelVariables();  //arne -> set generic variables (lveh,Tr etc) to zero
  // otherwwise, some trouble possible; initialize... inherited from  MicroModel
  modelNumber=17; // set it correctly!!

  cout << "\nin PCF file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams

  this->dt=dt;
  calc_eq();
  cout <<"End PCF file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}



//################################################################
void PCF::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length (lveh+s0=delta in Laval)

  inout.getvar(fp,&v0);  // Desired velocity [m/s] (=vc in Laval)
  inout.getvar(fp,&T);   // time gap [s] in CF mode (=tau in Laval)
  inout.getvar(fp,&s0);  // min. bumper-to-bumper distance [m](=0 in Laval)

  inout.getvar(fp,&tau); // speed relax time free accel (=1/beta in Laval)
  inout.getvar(fp,&Q);   // acceleration noise [m^2/s^3](=sigma^2 in Laval)

  fclose(fp);

  rhomax = 1./lveh;
}



//################################################################
void PCF::calc_eq(){
//################################################################

    // Calculates equilibrium velocity of PCF:
    // Finds equilibrium velocities veqtab[ir] with simple numeric relaxation
     //  (no brain, but stable and simple method...)

  // output:   veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)



    if(false){cout << "\nin PCF.calc_eq()"<<endl;}


    // start with rho=0

    veqtab[0]         = v0;

    for(int ir=1; ir<=NRHO; ir++){
      double rho = rhomax*ir/NRHO;
      double s   = 1./rho - 1./rhomax;
      veqtab[ir] = max(0., min(v0, (s-s0)/T));
    }

    calc_rhoQmax();  // Qmax, rhoQmax 
} // PCF::calc_eq()



//######################################
// acceleration of the PCF.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double PCF::accSimple(double s, double v, double dv,
			   double alpha_v0, double alpha_T, int it, double dt){


  double bmax=100; // crucial!! artifacts if arbitrary deceleration allowed!
  double v0loc=alpha_v0*v0; //generic parameter
  double Tloc=alpha_T*T;  //generic parameter

  // uniformly-distributed random variables with mu=0, sigma=1
  // random numbers in [-0.5 ... 0.5] have variance=1/12
  // myRand()=((double) (rand()/((double) RAND_MAX+1.)) - 0.5);
  double sqrt12=sqrt(12.);
  double rUni = sqrt12*(myRand()-0.5); // uniform, E(X)=0, Var(X)=1
  double rGauss=0;                     // Gaussian, E(X)=0, Var(X)=1
  for (int ir=0; ir<12; ir++){
    rGauss += myRand()-0.5;
  }

  double r1=rGauss;
  //double r1=rUni;

  // instance of free-flow displacement xi \sim N(mu,sigma^2)

  double dtDivTau=dt/tau;
  double muSimple=v*dt+0.5*(v0loc-v)*SQR(dt)/tau; 
  double mu=v0loc*dt-(1-exp(-dtDivTau))*tau*(v0loc-v); // exact
  double sig2=0.5*Q*pow(tau,3)*( exp(-dtDivTau)*(4-exp(-dtDivTau)) +2*dtDivTau-3);
  double xiFree=mu+sqrt(sig2)*r1;

  // interaction displacement 
  // xiInt=xl(t+dt-T)-lveh -x(t) approx s(t)+vl*(dt-T)

  double vl=v-dv;
  double xiInt=max(s-s0+vl*(dt-Tloc), 0.);

  // final PCF displacement formula

  double xi=min(xiFree, xiInt);
  double a_wanted=2*(xi-v*dt)/SQR(dt);

  if(it<2){
    cout <<"in PCF.accSimple: s="<<s<<" v="<<v<<" dv="<<dv<<endl
	 <<" dt="<<dt<<" dtDivTau="<<dtDivTau
	 <<" mu="<<mu<<" muSimple="<<muSimple<<endl
      //<<" v0*dt="<<(v0loc*dt)
      //<<" (1-exp(-dtDivTau))*tau*="<<(1-exp(-dtDivTau))<<endl
	 <<" sig2="<<sig2<<endl
	 <<" r1="<<r1<<" xiFree="<<xiFree<<" xiInt="<<xiInt<<" xi="<<xi
	 <<" a_wanted="<<a_wanted<<endl;
  }
  //else{exit(-1);}

  return max(a_wanted, -bmax);
}

//#############################################################
// acc 
//#############################################################

double PCF::acc(int it, int iveh, int imin, int imax,
		     double alpha_v0, double alpha_T,
		     const CyclicBuffer* const cyclicBuf)
{

    //#############################################################
    // Local dynamical variables
    //#############################################################


  double s= cyclicBuf->get_s(iveh); //xveh[iveh-1]-xveh[iveh]-lveh[iveh-1]
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double dv= v - cyclicBuf->get_v(iveh-1);//vveh[iveh];
  
 
  //#############################################################
  // actual PCF formula
  //#############################################################
  
  return  accSimple(s,v,dv,alpha_v0, alpha_T, it, dt);
  
}



#endif // PCF_CC

