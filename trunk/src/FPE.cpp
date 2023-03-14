// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>

using namespace std;

#include "FPE.h"
#include "RandomUtils.h"
#include "general.h"
#include "InOut.h"
#include "CyclicBuffer.h"

FPE::FPE(const char fname[], ProjectParams* proj)
{  
  initializeMicroModelVariables();  //arne -> init variables from MicroModel
  modelNumber=4;
  cout << "\nin FPE file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  this->dt=proj->get_dt();
  this->isRing=(proj->get_choice_BCup()==3);
  this->roadLen=proj->get_xmax()-proj->get_xmin(); //Achtung, roadLen war als bool definiert!!!

  // rhomax = 1./lveh; // in get_modelparams
  calc_eq();
  cout <<"End FPE file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.5/lveh)="<<get_veq(0.5/lveh)<<endl;

  cout <<"accSimple(s=100,v=30,sback=100,dt=0.2)="
       <<accSimple(100,30,100,0.2)
       <<" \naccSimple(10,30,10,0.2)="
       <<accSimple(10,30,10,0.2)
       <<" \naccSimple(5,30,5,0.2)="
       <<accSimple(5,30,5,0.2)
       <<" \naccSimple(4,30,4,0.2)="
       <<accSimple(4,30,4,0.2)
       <<endl;
  cout <<"End FPE file Cstr"<<endl;
  //exit(0);
}




//################################################################
void FPE::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&iso);
  inout.getvar(fp,&v0);
  inout.getvar(fp,&tau);
  inout.getvar(fp,&a_s0);
  inout.getvar(fp,&s0);
  inout.getvar(fp,&gamma);
  inout.getvar(fp,&Qfluct);

  fclose(fp);
  if(false){
      cerr << "Sorry; only standard FPE (choice_variant=5) supported"
	   <<endl
	   <<"To run old projects, use the sources of src_16jun02"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;

}



//################################################################
void FPE::calc_eq()
//################################################################

    // Calculates equilibrium velocity of FPE and FPE with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  bool testfund=false;
  if(testfund){cout << "\nin FPE.calc_eq()"<<endl;}

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
      if(testfund){ cout <<" start inner loop: v_it=veqtab[ir-1]="
			 <<veqtab[ir-1]<< endl;
      }

      for (int it=1; it<=itmax; it++){
        double acc = accSimple(s,v_it,s,dt);
        double dtloc = dtmax*v_it/v0 + dtmin; // it. step in [dtmin,dtmax]

	// actual relaxation 

        v_it            += dtloc*acc;
        if(v_it<0) v_it=0;

        if(testfund){ 
          cout << "funddia: rho="<< rho<< " s="<<s
               <<" v0="<<v0<<" acc="<<acc
	       <<" dtloc="<<dtloc<<" v_it="<<v_it<<endl;
	}
      }
      veqtab[ir] = v_it;

    }

    calc_rhoQmax();  // Qmax, rhoQmax 
} // FPE::calc_eq()



//######################################
// acceleration of the FPE.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
//######################################

double FPE::accSimple(double s, double v, double sback, double dt){
  double amax_dt1=10;                 //!!!
  double amax=max(10.,amax_dt1/dt);  //!!
  // FPE acceleration
  double s_loc=max(s, 0.1);
  double sback_loc=max(sback, 0.5);
  double a_free      = (v0-v)/tau;
  double a_int_front = -a_s0*pow(s0/s_loc, gamma);
  double a_int_back  = +a_s0*pow(s0/sback_loc, gamma);
  double randomNumber= myRand();//(double) (rand()/((double) RAND_MAX+1.0)); //=G(0;1)
  double a_stoch     = sqrt(12*Qfluct/dt) * (randomNumber-0.5);
  double acc=a_free + a_int_front + iso*a_int_back + a_stoch;
  //if(false){
  if(!( (acc>-100) || (acc<100))){
    cerr<<" FPE.accSimple: error: "
	<<" accSimple(s="<<s<<", v="<<v<<", sback="<<sback<<", dt="<<dt<<")="
        <<acc<<" not in right range!"<<endl;
    exit(-1);
  }
  if(acc>amax){acc=amax;}
  if(acc<-amax){acc=-amax;}

  return( acc);
}

//################################################################

double FPE::acc(int it, int iveh, int imin, int imax,
                double alpha_v0, double alpha_T,
		const CyclicBuffer* const cyclicBuf)
{

  //#############################################################
  // Local dynamical variables
  //#############################################################
  
  double s=cyclicBuf->get_s(iveh);
  double sback=cyclicBuf->get_x(iveh)-cyclicBuf->get_l(iveh)-cyclicBuf->get_x(iveh+1);

  if(iveh==imax)
    {
      sback=cyclicBuf->get_x(imin)-cyclicBuf->get_l(imin)-cyclicBuf->get_x(imin+1);
    }

  double v=cyclicBuf->get_v(iveh);

  if(false){
    cout <<"FPE.acc: it="<<it<<" iveh="<<iveh
	 <<" imin="<<imin<<" imax="<<imax
	 <<" s="<<s<<" sback="<<sback
	 <<" v="<<v<<" acc="<<accSimple(s,v,sback,dt)
	 <<endl;
  }

  return (accSimple(s,v,sback,dt));

}







