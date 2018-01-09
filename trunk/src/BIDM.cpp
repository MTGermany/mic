#ifndef BIDM_CC
#define BIDM_CC

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
#include "RandomUtils.h"
#include "general.h"
#include "InOut.h"
#include "BIDM.h"
#include "CyclicBuffer.h"


// Instructions to incorporate new model: see BIDM.h


BIDM::BIDM(const char fname[], double dt)
{
  initializeMicroModelVariables();  //arne -> set generic variables (lveh,Tr etc) to zero
  // otherwwise, some trouble possible; initialize... inherited from  MicroModel
  modelNumber=18; // set it correctly!!

  cout << "\nin BIDM file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams

  this->dt=dt;
  this->accOld=0;
  calc_eq();
  cout <<"End BIDM file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}



//################################################################
void BIDM::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&T1);
  inout.getvar(fp,&T2);
  inout.getvar(fp,&s0);
 
  inout.getvar(fp,&a);
  inout.getvar(fp,&b);
  inout.getvar(fp,&A);
  inout.getvar(fp,&lambda);
  inout.getvar(fp,&bmax);

  fclose(fp);

  if(false){
      cerr << "Sorry; something was not correct"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;

}



//################################################################
void BIDM::calc_eq(){
//################################################################

    // Calculates equilibrium velocity of BIDM:
    // Finds equilibrium velocities veqtab[ir] with simple numeric relaxation
     //  (no brain, but stable and simple method...)

  // output:   veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)



    if(false){cout << "\nin BIDM.calc_eq()"<<endl;}

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
        double acc = accSimple(s,v_it,0,1,1,false);
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
    accOld=0; // to avoid side effects

    calc_rhoQmax();  // Qmax, rhoQmax 
} // BIDM::calc_eq()



//######################################
// acceleration of the BIDM.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double BIDM::accSimple(double s, double v, double dv,
		       double alpha_v0, double alpha_T, bool logging){


  double bmax=9; // crucial!! artifacts if arbitrary deceleration allowed!
  double v0loc=alpha_v0*v0; //generic parameter
  double Tloc=alpha_T*T;  //generic parameter
  double T1loc=alpha_T*T1;  //generic parameter
  double T2loc=alpha_T*T2;  //generic parameter
  double accWanted=0;

  // BIDM acceleration

  double sstar = s0+max(0.,Tloc*v + 0.5*v*dv/sqrt(a*b));
  double sstar1 = s0+max(0.,T1loc*v + 0.5*v*dv/sqrt(a*b));
  double sstar2 = s0+max(0.,T2loc*v + 0.5*v*dv/sqrt(a*b));

  bool inIndifferenceZone=((s>=sstar1)&&(s<=sstar2));

  if(inIndifferenceZone){
    double sqrt12=sqrt(12.);
    double r1 = sqrt12*(myRand()-0.5); // myRand \sim G(0,1)
    if(logging){cout <<" myRand-0.5="<<r1/sqrt12
		     <<" sqrt(A*dt)*r1="<<sqrt(A*dt)*r1<<endl;}
    //accWanted=accOld+sqrt(A*dt)*r1-lambda*dv; // random walk acc
    accWanted=0+sqrt(A/dt)*r1-lambda*dv; // random walk v
  }
  else{
    accWanted=a*(1.- pow((v/v0loc),4)-SQR(sstar/s) );
  }

  accWanted=max(accWanted, -bmax);

  if(logging){
    cout <<" s="<<s<<" v="<<v<<" dv="<<dv<<endl
	 <<" sstar1="<<sstar1<<" sstar2="<<sstar2
	 <<" inIndiff="<<inIndifferenceZone
	 <<"  accWanted="<<accWanted<<" accOld="<<accOld
	 <<endl;
  }

  accOld=accWanted;
 
  return accWanted;
}


//#############################################################
// acc 
//#############################################################

double BIDM::acc(int it, int iveh, int imin, int imax,
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
  // actual BIDM formula
  //#############################################################
  
  //bool logging=(iveh==2);
  bool logging=false;
  if(logging){cout <<"in BIDM: iveh="<<iveh<<endl;}
  return  accSimple(s,v,dv,alpha_v0,alpha_T,logging);
  
}



#endif // BIDM_CC

