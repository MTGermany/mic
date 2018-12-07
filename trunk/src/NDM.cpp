#ifndef NDM_CC
#define NDM_CC

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
#include "NDM.h"
#include "CyclicBuffer.h"


// Instructions to incorporate new model: see NDM.h


NDM::NDM(const char fname[], double dt)
{
  initializeMicroModelVariables();  //arne -> set generic variables (lveh,Tr etc) to zero
  // otherwwise, some trouble possible; initialize... inherited from  MicroModel
  modelNumber=100; // set it correctly!!

  cout << "\nin NDM file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams

  this->dt=dt;
  calc_eq();
  cout <<"End NDM file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}



//################################################################
void NDM::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&s0);
  inout.getvar(fp,&tau);
  inout.getvar(fp,&bmax);

  fclose(fp);

  rhomax = 1./lveh;

}



//################################################################
void NDM::calc_eq(){
//################################################################

    // Calculates equilibrium velocity of NDM:
    // Finds equilibrium velocities veqtab[ir] with simple numeric relaxation
     //  (no brain, but stable and simple method...)

  // output:   veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)



    if(false){cout << "\nin NDM.calc_eq()"<<endl;}

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
} // NDM::calc_eq()



//######################################
// acceleration of the NDM.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double NDM::accSimple(double s, double v, double dv,
			   double alpha_v0, double alpha_T){


  double v0loc=alpha_v0*v0; //generic parameter
  double Tloc=alpha_T*T;  //generic parameter

  // NDM acceleration (below just an example for the IDM)

  double seq=s0+v*Tloc;
  double thetadv=(dv<0) ? 0 : 1;
  double dec1=(s>s0) ? min(0.5*dv*dv/(s-s0), bmax) : bmax;
  dec1 *= thetadv;
  bool dec2_isActive=(s<seq) && (dv>0);
  double dec2=(dec2_isActive) ? pow((s-seq)/seq, 2) : 0;
  double a_wanted= (v0loc-v)/tau*((s>seq) ? 1 : 0)-min(dec1+dec2, bmax);
  return a_wanted;
}

//#############################################################
// acc 
//#############################################################

double NDM::acc(int it, int iveh, int imin, int imax,
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
  // actual NDM formula
  //#############################################################
  
  return  accSimple(s,v,dv,alpha_v0,  alpha_T);
  
}



#endif // NDM_CC

