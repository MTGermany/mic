#ifndef KernerCA_CC
#define KernerCA_CC

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
#include "KernerCA.h"
#include "CyclicBuffer.h"
#include "RandomUtils.h"


// Instructions to incorporate new model: see KernerCA.h


KernerCA::KernerCA(const char fname[], double dt)
{
  cout << "\nin KernerCA file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  this->dt=dt;
  calc_eq();
  cout <<"End KernerCA file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.5/lveh)="<<get_veq(0.5/lveh)<<endl;

}



//################################################################
void KernerCA::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&k);
  inout.getvar(fp,&pb0);
  inout.getvar(fp,&pb1);
  inout.getvar(fp,&pa1);
  inout.getvar(fp,&pa2);
  inout.getvar(fp,&vp);
  inout.getvar(fp,&lambda);
  nagelSchreck=0; // optional selector for original NSM; default=0=false
  inout.getvar(fp,&nagelSchreck); // if no data line found, nothing is done
  fclose(fp);

  if(false){
      cerr << "Sorry; only standard KernerCA (choice_variant=5) supported"
	   <<endl
	   <<"To run old projects, use the sources of src_16jun02"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;

}



//#####################################################
void KernerCA::calc_eq()
//#####################################################

    // Calculates equilibrium velocity of KernerCA and KernerCA
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:  veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin KernerCA.calc_eq()"<<endl;}

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
        double acc = accSimple(s,v_it,0.,1,1);
        double dtloc = dtmax*v_it/v0 + dtmin; // it. step in [dtmin,dtmax]

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
} // KernerCA::calc_eq()



//######################################
// acceleration of the KernerCA.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double KernerCA::accSimple(double s, double v, double dv,
			   double alpha_v0, double alpha_T){



  // KernerCA acceleration
  int v0loc=(int)(alpha_v0*v0+0.5);
  int vloc=(int)(v+0.5);
  int tau=1;  // Zeiteinheit, lasse es immer bei 1 s!
  int vNew=0;
  
  if(nagelSchreck==0){// KCA model
 
    int amax=1;    // Zellenlaenge/tau^2 mit tau=1 s und Laenge 0.5 m => steps 0.5 m/s^2
    //int amax=2;    // amax=2 is more realistic




  // dynamic deterministic part [Eqs (11.65)- (11.69) of 2009 Kerner book]

    double kloc=alpha_T*k;
    double D=lveh+kloc*vloc*tau; // double bei Kerner, da k reelle Zahl
    int vsafe=(int)s;     // (Delta x-d)/tau mit s=Delta x-d und tau=1 (s)

    // martin oct11: Test Gipps like vsafe instead of the above crude NSM-like one
    bool testBsafeGipps=true;
    if(testBsafeGipps){
      int b=4;
      int vp=(int)(v-dv+0.5);
      vsafe=(int)(min(s, -b*tau+sqrt(b*b*tau*tau+vp*vp+2*b*s)));
    }
    // end martin oct11

    int dvsign=(dv<-0.5) ? 1 : (dv>0.5) ? -1 : 0;
    int vc=(s>D-lveh) ? vloc+amax*tau : vloc+amax*tau*dvsign;
    int vtilde=min(min(v0loc,vsafe), vc);
    vtilde=max(0, vtilde);

  // stochastic part  [Eqs (11.63), (11.70) of 2009 Kerner book]
  // noise terms [myRand() ~ G(0,1)]

    double pa=(vloc<vp) ? pa1 : pa2;
    double pb=(vloc<1) ? pb0 : pb1;
    double r1= myRand();
    int eta=(r1<pb) ? -1 : (r1<pb+pa) ? 1 : 0;

    vNew=min(vtilde+amax*tau*eta, vloc+amax*tau);
    vNew=min(min(v0loc, vsafe), vNew);
    vNew=max(0, vNew);
  }
  else if(nagelSchreck==1){// Nagel-Schreckenberg or Barlovic-Model
    double r1= myRand();
    double pb=(vloc<1) ? pb0 : pb1;
    int troedel=(r1<pb) ? 1 : 0;
    
    int sloc=(int)(s+0.5);
    vNew=min(vloc+1, v0loc);
    vNew=min(vNew,sloc);
    vNew=max(0,vNew-troedel);
  }

  
  return ((vNew-vloc)/tau);
  
}

//#####################################################
// acc (in vehicle.updatePosVel there is bool isCAvehicle taking care of
// proper CA positional update xnew=xold+vNew*dt)
//#####################################################

double KernerCA::acc(int it, int iveh, int imin, int imax,
		     double alpha_v0, double alpha_T,
		     const CyclicBuffer* const cyclicBuf)
{

    //####################################################
    // Local dynamical variables
    //####################################################


  double s= cyclicBuf->get_s(iveh); 
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double dv= v - cyclicBuf->get_v(iveh-1);//vveh[iveh];
  
  
  //####################################################
  // actual KernerCA formula in accSimple
  //####################################################

  //if(it<5){
  if(false){
    cout << "KernerCA: it="<<it<<" x="<<cyclicBuf->get_x(iveh)
	 <<" s="<<s<<" v="<<v<<" dv="<<dv
	 <<" acc="<<accSimple(s,v,dv,alpha_v0,  alpha_T)
	 <<endl;
  }
  return  accSimple(s,v,dv,alpha_v0,  alpha_T);
  
}



#endif // KernerCA_CC

