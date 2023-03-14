#ifndef NH_CC
#define NH_CC

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
#include "NH.h"
#include "CyclicBuffer.h"
#include "RandomUtils.h"


// Instructions to incorporate new model: see NH.h


NH::NH(const char fname[], double dt)
{
  initializeMicroModelVariables();  //arne -> set generic variables (lveh,Tr etc) to zero
  // otherwwise, some trouble possible; initialize... inherited from  MicroModel

  modelNumber=15; // set it correctly!!

  cout << "\nin NH file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams

  this->dt=dt;
  calc_eq();
  cout <<"End NH file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.5/lveh)="<<get_veq(0.5/lveh)<<endl;

}



//################################################################
void NH::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 
                         // !! must assume in RoadSection.cpp lVehAdhoc=0.5 or 1.0 or 7.5?

  double dvariant;
  inout.getvar(fp,&dvariant); variant=(int)(dvariant+0.5);
  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&b_defens);
  inout.getvar(fp,&pa);
  inout.getvar(fp,&pb);
  inout.getvar(fp,&pc);
  inout.getvar(fp,&g_safety);
  inout.getvar(fp,&tc);

  fclose(fp);

  rhomax = 1./lveh;
  dtStopped=0;

}



//################################################################
void NH::calc_eq(){
//################################################################

    // Calculates equilibrium velocity of NH:
    // Finds equilibrium velocities veqtab[ir] with simple numeric relaxation
     //  (no brain, but stable and simple method...)

  // output:   veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)



    if(false){cout << "\nin NH.calc_eq()"<<endl;}

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
        double acc = accSimple(s,s,v_it,0,1,1,0);
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
} // NH::calc_eq()



//######################################
// acceleration of the NH.
// Argument parameters:

// s=gap [lveh]  (yes, SI!)
// sLead=gap of leading veh [lveh]  (yes, SI!)
// v=own velocity [m/s]   (yes, SI!)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double NH::accSimple(double s, double sLead, double v, double dv,
		     double alpha_v0, double alpha_T, int iveh){


  // preparation for NH acceleration
  
  int v0Int=(int)(alpha_v0*v0+0.5);  // v0 in units of lCell/dt=lveh/dt; dt=1.
  int gSafetyInt=(int)(g_safety+0.5); // possibly not necessary if gSafety int
  int vInt=(int)(v/lveh+0.5);
  int vLeadInt=(int)((v-dv)/lveh+0.5);
  int sInt=(int)(s/lveh+0.5);
  int sLeadInt=(int)(sLead/lveh+0.5);



  // NH acceleration

  dtStopped=(vInt!=0) ? 0 : dtStopped+1;

  double sstar=T*vInt; // desired space gap w/o speed differences

  int v_anti=min(vLeadInt+1, min(v0Int, sLeadInt)); // anticipated speed leader

  int seff=sInt+max(v_anti-gSafetyInt, 0);

  double pBrake=(seff<sstar) ? pa : ((vInt==0)&&(dtStopped>=tc)) ? pb : pc;

  int dvBrake=(seff<sstar) ? (int)(b_defens+0.5) : 1;

  int vWantedInt=min(min(vInt+1, v0Int), seff);

  if (myRand()<pBrake){vWantedInt=max(vWantedInt-dvBrake, 0);} // myRand() in G(0,1)

  double a_wanted=lveh*(vWantedInt-vInt);  // SI since all outer loops use SI!

  if(iveh<=2){
    cout <<"NH:acSimple: iveh="<<iveh<<" sInt="<<sInt
	 <<" vInt="<<vInt<<" vLeadInt="<<vLeadInt<<" sLeadInt="<<sLeadInt
         <<" sstar="<<sstar
         <<" v_anti="<<v_anti
	 <<" seff="<<seff<<" a_wanted="<<a_wanted
	 <<endl;
  }



  return a_wanted;
}

//#############################################################
// acc 
//#############################################################

double NH::acc(int it, int iveh, int imin, int imax,
		     double alpha_v0, double alpha_T,
		     const CyclicBuffer* const cyclicBuf)
{

    //#############################################################
    // Local dynamical variables (SI)
    //#############################################################


  double s= cyclicBuf->get_s(iveh); //xveh[iveh-1]-xveh[iveh]-lveh[iveh-1]
  double sLead= (iveh>imin) ? cyclicBuf->get_s(iveh-1): s;
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double dv= v - cyclicBuf->get_v(iveh-1);//vveh[iveh];
  if(false){
    // if(iveh<=4){
    cout <<"NH::acc: it="<<it<<" iveh="<<iveh<<" imin="<<imin
	 <<" x="<<cyclicBuf->get_x(iveh)
	 <<" xLead="<<cyclicBuf->get_x(iveh-1)
	 <<" lLead="<<cyclicBuf->get_l(iveh-1)
	 <<" s="<<s<<" sLead="<<sLead<<" v="<<v<<" dv="<<dv
	 <<" accSimple="<<accSimple(s,sLead,v,dv,alpha_v0,alpha_T,iveh)
	 <<endl;}
 
  //#############################################################
  // actual NH formula
  //#############################################################
  
  return  accSimple(s,sLead,v,dv,alpha_v0,alpha_T,iveh);
  
}



#endif // NH_CC

