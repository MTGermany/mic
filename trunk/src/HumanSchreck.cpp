#ifndef HUMANSCHRECK_CC
#define HUMANSCHRECK_CC


// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

//own
#include "HumanSchreck.h"
#include "RandomUtils.h"
#include "general.h"
#include "InOut.h"

#include "CyclicBuffer.h"

HumanSchreck::HumanSchreck(const char* fname, double dt)
{
  initializeMicroModelVariables();  //arne -> init variables from MicroModel
  modelNumber=5;

  cout << "\nin HumanSchreck file Cstr: fname= "<<fname<<endl;
  this->dt=dt;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  calc_eq();
  cout <<"End HumanSchreck file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}




//################################################################
void HumanSchreck::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);     //Desired velocity (cell units/time unit)
  inout.getvar(fp,&vfast);  //velocity where traffic is considered not cong.
  inout.getvar(fp,&vslow);  //velocity where traffic is considered very cong.
  inout.getvar(fp,&a);	    //acceleration (cell units/time unit^2)
  inout.getvar(fp,&D);	    //braking deceleration (cell units/time unit^2)
  inout.getvar(fp,&gadd);   //"addtl. safety gap" in the defensive state
  inout.getvar(fp,&tsafe);  //"maximal time step" (interval?) for opt. state
  inout.getvar(fp,&p0);     //"Troedelwahrsch." for standing vehicles
  inout.getvar(fp,&pd);     //"Troedelwahrsch." for v>vslow
  fclose(fp);

  rhomax = 1./lveh;

}



//################################################################
void HumanSchreck::calc_eq()
//################################################################

    // Calculates equilibrium velocity of HumanSchreck and HumanSchreck with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin HumanSchreck.calc_eq()"<<endl;}

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
        double acc = accSimple(static_cast<int>(s),static_cast<int>(v_it),
			       static_cast<int>(v_it),static_cast<int>(v_it), false);
        double dtloc = dtmax*v_it/v0 + dtmin; // it. step in [dtmin,dtmax]
	/*
       double sstar   = s0 + T * v_it + s1*sqrt((v_it+0.000001)/v0);

	// acceleration for various variants

        double acc = (s>s0) 
	  ? a * (1-pow(v_it/v0, 4) - SQR(sstar/s ))
          :0;
	*/

	// actual relaxation 

        v_it            += dtloc*acc;
        if((v_it<0)||(s<0)) v_it=0;

        if(false){ 
          cout << "rho="<< rho<< " s="<<s
               <<" v0="<<v0<<" acc="<<acc<<" v_it="<<v_it<<endl;
	}
      }
      veqtab[ir] = v_it;

    }

    calc_rhoQmax();  // Qmax, rhoQmax 
} // HumanSchreck::calc_eq()



//######################################
// acceleration of the HumanSchreck [PRL 92, 238702 (2004)]
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// vP=velocity of front vehicle
// vPP=velocity of front vehicle of front vehicle
//######################################

// Eqs (1),(3)
bool HumanSchreck::isSafe(int s, int v, int vP, int vSafe,
                          bool driverIsOptimistic, bool debug){ 

  // minimum netto safe distance (Eq. (3a) at actual velocity, 
  // deltaNet=Delta-length[iveh-1])

  int  deltaNet = (driverIsOptimistic) ? 0 : max(0, min(gadd, v-gadd));

  // braking time tau_l of  leading vehicle
  // for the prospected braking manoeuvres (Eqs. (3b,c))

  int tau_l=(driverIsOptimistic) ? min(vP/D,tsafe) : vP/D;

  // braking time tau_l of own vehicle at safe velocity

  int tau_f=(driverIsOptimistic) ? max(0, min(vSafe/D,tsafe)-1) : vSafe/D;

  // Eq. (1)

  int ownBrakingDistance=deltaNet+(tau_f+1)*vSafe-D*(tau_f*(tau_f+1))/2;
  int leadBrakingDistance=tau_l*vP-D*(tau_l*(tau_l+1))/2;
  if(debug){
    cout <<"HumanSchreck.isSafe: "<<endl;

    cout <<" s="<<s
	 <<" v="<<v
	 <<" vP="<<vP
	 <<" vSafe="<<vSafe<<endl;

    cout <<" driverIsOptimistic="<<(driverIsOptimistic ? "true" : "false");

    cout <<" deltaNet="<<deltaNet
	 <<" tau_l="<<tau_l
	 <<" tau_f="<<tau_f<<endl;

    cout <<" ownBrakingDistance="<<ownBrakingDistance
	 <<" leadBrakingDistance="<<leadBrakingDistance
	 <<" isSafe="
	 <<((s >= ownBrakingDistance-leadBrakingDistance)? "true" : "false")
	 <<endl;
  }

  return (s >= ownBrakingDistance-leadBrakingDistance);
}

double HumanSchreck::accSimple(int s, int v, int vP, int vPP, bool debug){


  // dependence on the trafficv situation (Eq. (2))

  bool driverIsOptimistic=( ((v <= vP) && (vP <= vPP)) || (vPP >= vfast));

  // simultaneous calculation of
  // (1) braking time tau_f of own vehicle 
  // (2) braking time tau_l of  leading vehicle
  //     for the prospected braking manoeuvres (Eqs. (3b,c))
  // (3) maximum safe velocity vsafe (Eq. (1); there called "c_n")

  int vSafe=v0;
  bool safetyReached=isSafe(s, v, vP, vSafe, driverIsOptimistic, debug);

  while ( (!safetyReached) && (vSafe>0)){
    vSafe--;
    safetyReached=isSafe(s, v, vP, vSafe, driverIsOptimistic, debug);
  }

  // in-text formula for actual update

  // step (i)

  double p=max(pd, p0-v/max(vslow,1)*(p0-pd));

  // step (ii): calc. of vSafe, see above 

  // step (iii): acceleration+deceleration

  int vAfterAcc=min(v+a, v0);
  int vAfterBrake=max(max(v-D, 0),vSafe);  // max braking decel. not exceeded!
  int vAfterAccBrake=min(vAfterAcc,vAfterBrake);

  // step (iv): stochastic "Troedeleffekt"

  double r1=myRand();//(double) (rand()/((double) RAND_MAX+1.));
  bool isSluggish = (r1<p);
  int vAfterStochasticity=(isSluggish) 
    ? max(v-D, max(vAfterAccBrake-1,0)) : vAfterAccBrake;

  // NS-CA
  //int v1=min(v0, v+a);
  //int v2=min(v1, s-1) - ( troedelt ? 1 : 0);
  //int v3=max(v2,0);
  //double acc = (v3-v)/dt;

  double acc=(vAfterStochasticity-v)/dt;
  if(debug){
    cout <<"HumanSchreck.accSimple: "<<endl;

    cout <<" s="<<s
	 <<" v="<<v
	 <<" vP="<<vP
	 <<" vPP="<<vPP<<endl;

    cout <<" driverIsOptimistic="<<(driverIsOptimistic ? "true" : "false")
	 <<" vSafe="<<vSafe<<endl;

    cout <<" vAfterAcc="<<vAfterAcc
         <<" vAfterBrake="<<vAfterBrake
         <<" vAfterAccBrake="<<vAfterAccBrake<<endl;

    cout <<" p="<<p
         <<" isSluggish="<<(isSluggish ? "true" : "false")
	 <<" vAfterStochasticity="<<vAfterStochasticity<<endl;

    cout <<" acc="<<acc<<endl;
  }
      
  return acc;
}





//################################################################
//double HumanSchreck::acc(int it, double v, double s, double dv, double a_lead,
double HumanSchreck::acc(int it, int iveh, int imin, int imax,
			 double alpha_v0, double alpha_T,
			 const CyclicBuffer* const cyclicBuf)
{

  //#############################################################
  // Local dynamical variables
  //#############################################################

  //    int s   = static_cast<int>(xveh[iveh-1]-length[iveh-1]-xveh[iveh]+0.5);
  //    int v   = static_cast<int>(vveh[iveh]);
  //    int vP  = static_cast<int>(vveh[iveh-1]);
  //    int vPP = static_cast<int>( (iveh-2<imin) ? vveh[imin] : vveh[iveh-2]);

  int s   = static_cast<int>(cyclicBuf->get_s(iveh)+0.5);
  int v   = static_cast<int>(cyclicBuf->get_v(iveh));
  int vP  = static_cast<int>(cyclicBuf->get_v(iveh-1));
  int vPP = static_cast<int>( (iveh-2<imin) ? cyclicBuf->get_v(imin) : cyclicBuf->get_v(iveh-2));

  //bool debug=(iveh==2);
  bool debug=false;
  return (accSimple(s,v,vP,vPP,debug));
  
}



#endif // HUMANSCHRECK_CC

