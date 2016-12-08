#ifndef ASGM_CC
#define ASGM_CC

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
#include "ASGM.h"
#include "CyclicBuffer.h"
#include "RandomUtils.h"


// Instructions to incorporate new model: see NewModel.h


ASGM::ASGM(const char fname[], double dt)
{
  cout << "\nin ASGM file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  modelNumber=13; // set it correctly!!

  speedlimit=100000; // martin apr08: initially no speed limit 

  this->dt=1; // input parameter dt superfluous here
  calc_eq();

}



//################################################################
void ASGM::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);
  double mldouble;  inout.getvar(fp,&mldouble); ml=(int)(mldouble+0.5);
  inout.getvar(fp,&pa);
  inout.getvar(fp,&pb);
  inout.getvar(fp,&pc);
  inout.getvar(fp,&decel_a);
  inout.getvar(fp,&decel_b);
  inout.getvar(fp,&tc);

  fclose(fp);

  if(false){
      cerr << "Sorry; only standard ASGM (choice_variant=5) supported"
	   <<endl
	   <<"To run old projects, use the sources of src_16jun02"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;
  tstopped=0;

}



//################################################################
void ASGM::calc_eq()
//################################################################

    // Calculates equilibrium velocity of ASGM 
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:   veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                              (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin ASGM.calc_eq()"<<endl;}

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
      int s   = (int)(1./rho - 1./rhomax+0.5);

      // start iteration with equilibrium velocity for the previous density
      v_it             = veqtab[ir-1];
      if(false) cout << endl;

      for (int it=1; it<=itmax; it++){
        double acc = accSimple(0,0, s,s,v_it,1);
        double dtloc = dtmax*v_it/v0 + dtmin; // it. step in [dtmin,dtmax]

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
    tstopped=0; // to avoid side effects
    
} // ASGM::calc_eq()



//######################################
// acceleration of the ASGM.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// s_avg=avg net gaps of the own and ml leading vehicles (calculated in acc)
// alpha_v0= multiplicators of v0
// alpha_T not implemented
//######################################

double ASGM::accSimple(int it, double x,
		       int s, int s_avg, double v, 
		       double alpha_v0){


  // preparation for ASGM acceleration
  
  int v0loc=(int)(alpha_v0*v0+0.5);
  int vloc=(int)(v+0.5);
  int tcloc=(int)(tc+0.5);
  

  // ASGM acceleration
  
  tstopped=(vloc>0) ? 0 : tstopped+dt;
  double p=(vloc>s_avg) ? pa : ( (vloc==0)&&(tstopped>=tcloc)) ? pb : pc;
  int decel_slug=(vloc>s_avg) ? ((int)(decel_a+0.5)) : ((int)(decel_b+0.5));
  int vNew=min( min(vloc+1, v0loc), s);
  bool isSluggish=(myRand()<p);
  if(isSluggish){vNew=max(vNew-decel_slug, 0);}

  if(false){
    // if(it<3){
    cout <<"ASGM.accSimple: it="<<it<<" x="<<x
	 <<" vloc="<<vloc<<" s="<<s<<" s_avg="<<s_avg
	 <<" tstopped="<<tstopped<<" p="<<p<<endl;
  }

  return ((vNew-vloc)/dt);
}

//###################################################
// acc 
//###################################################

double ASGM::acc(int it, int iveh, int imin, int imax,
		     double alpha_v0, double alpha_T,
		     const CyclicBuffer* const cyclicBuf)
{

    //#################################################
    // Local dynamical variables
    //#################################################


  int s= (int)(cyclicBuf->get_s(iveh)+0.5); 
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double s_sum=s;
  int n_anti=min(ml, iveh-imin+1);
  for (int anti=1; anti<=n_anti; anti++){
    s_sum +=cyclicBuf->get_s(iveh-anti);
  }
  int s_avg=(int)(s_sum/(n_anti+1));
  double x=cyclicBuf->get_x(iveh);  // just for debugging
  if(false){cout <<"ASGM.acc: it="<<it//<<" iveh="<<iveh<<" n_anti="<<n_anti
		<<" s("<<iveh<<")="<<s
		<<" s("<<iveh-1<<")="<<cyclicBuf->get_s(iveh-1)
		<<" s("<<iveh-2<<")="<<cyclicBuf->get_s(iveh-2)
		<<" s("<<iveh-3<<")="<<cyclicBuf->get_s(iveh-3)
		<<" s_avg="<<s_avg
		<<endl;
  }
  //###################################################
  // actual ASGM formula
  //###################################################
  
  return  accSimple(it,x, s,s_avg,v, alpha_v0);
  
}



#endif // ASGM_CC

