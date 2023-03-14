#ifndef NewModel_CC
#define NewModel_CC

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
#include "NewModel.h"
#include "CyclicBuffer.h"


// Instructions to incorporate new model: see NewModel.h


NewModel::NewModel(const char fname[], double dt)
{
  initializeMicroModelVariables();  //arne -> set generic variables (lveh,Tr etc) to zero
  // otherwwise, some trouble possible; initialize... inherited from  MicroModel
  modelNumber=100; // set it correctly!!

  cout << "\nin NewModel file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams

  this->dt=dt;
  calc_eq();
  cout <<"End NewModel file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.5/lveh)="<<get_veq(0.5/lveh)<<endl;

}



//################################################################
void NewModel::get_modelparams(const char fname[]){
//################################################################
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
      cerr << "Sorry; only standard NewModel (choice_variant=5) supported"
	   <<endl
	   <<"To run old projects, use the sources of src_16jun02"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;

}



//################################################################
void NewModel::calc_eq(){
//################################################################

    // Calculates equilibrium velocity of NewModel:
    // Finds equilibrium velocities veqtab[ir] with simple numeric relaxation
     //  (no brain, but stable and simple method...)

  // output:   veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)



    if(false){cout << "\nin NewModel.calc_eq()"<<endl;}

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
        double acc = accSimple(s,v_it,0,1,1);
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
} // NewModel::calc_eq()



//######################################
// acceleration of the NewModel.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double NewModel::accSimple(double s, double v, double dv,
			   double alpha_v0, double alpha_T){


  double bmax=9; // crucial!! artifacts if arbitrary deceleration allowed!
  double v0loc=alpha_v0*v0; //generic parameter
  double Tloc=alpha_T*T;  //generic parameter
  double aloc  = a;     //generic parameter
  double a_wanted=0;

  // NewModel acceleration (below just an example for the IDM)

  if(false){
    double sstar = s0+max(0.,Tloc*v + s1*sqrt((v+0.0001)/v0loc) + 0.5*v*dv/sqrt(aloc*b));
    a_wanted = aloc*( 1.- pow((v/v0loc),delta) - SQR(sstar/s));
  }

  // NewModel acceleration (Test of the CCFM (You-Jun, 2012, referee))

  if(false){
    double k=delta; // sensitivity 1/tau
    double xc=s1; // threshold distance for interaction
    double lambda=(s<xc) ? a : 0;
    double vl=v-dv;

    double sstar = s0 + max(0., Tloc*v + 0.5*(v*v-vl*vl)/b);
    double vopt=max(0., min(v0loc, (s-s0)/T));
    a_wanted=k*(vopt-v)+lambda*(1-sstar/s);
  }

  // LCM model of Ni Haiheng (2012)

  if(false){
    double vl=v-dv;
    //double gamma=-0.0; // 1/2(1/b-1/B)
    //double B=1/(1/b-2*gamma);
    double B=6;
    double sstar=max(s0, 0.5*v*v/b-0.5*vl*vl/B + Tloc*v+s0);
    a_wanted=a*(1-v/v0loc-exp(1-s/sstar));
  }


   // "IDM+" Model of Ros, TU Delft (Referee Oct 2012)

  if(true){
    double sstar = s0 + max(0.,Tloc*v + s1*sqrt((v+0.0001)/v0loc) + 0.5*v*dv/sqrt(aloc*b));
    a_wanted = aloc*min(1.- pow((v/v0loc),delta), 1 - SQR(sstar/s));
  }



  return max(a_wanted, -bmax);
}

//#############################################################
// acc 
//#############################################################

double NewModel::acc(int it, int iveh, int imin, int imax,
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
  // actual NewModel formula
  //#############################################################
  
  return  accSimple(s,v,dv,alpha_v0,  alpha_T);
  
}



#endif // NewModel_CC

