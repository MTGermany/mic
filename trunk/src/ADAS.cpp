#ifndef ADAS_CC
#define ADAS_CC

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
#include "ADAS.h"
#include "CyclicBuffer.h"


// Instructions to incorporate new model: see ADAS.h


ADAS::ADAS(const char fname[], double dt)
{
  cout << "\nin ADAS file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  // rhomax = 1./lveh; // in get_modelparams
  modelNumber=14; // set it correctly!!

  speedlimit=100000; // martin apr08: initially no speed limit 

  this->dt=dt;
  calc_eq();

  // optionally test PT model calculations
  
  if(true){
    testModelEquations(fname);
  }
  
} // end file constructor
//######################################################
 


//########################################################
// Test routines for ADAS model calculations		   
//########################################################

void ADAS::testModelEquations(const char fname[]){
  
  cout <<"ADAS.testModelEquations: test everything for parameter set "
       <<fname<<endl;



 // Test accelerations by looking at files

  cout <<"\nADAS.testModelEquations: Calculate+Write some acceleration matrices"<<endl;

  InOut inout;

  double atab[101][inout.NYMAX];
  int n=101;
  double smax=110;
  double sref=20;
  double delta_s=smax/((double)(n-1));
  double vmax=v0;
  double vref=15;
  double delta_v=vmax/((double)(n-1));
  double dvmin=-vref;
  double dvmax=vref;
  double dvref=0;
  double delta_dv=(dvmax-dvmin)/((double)(n-1));
  
  char testfileName[NSTRMAX];
  char titleString[NSTRMAX];

  // acc(s,v)
  
  sprintf(testfileName,"%s.acctab_s_v",fname);
  sprintf(titleString,"gap s\t\tv\t\taccSimple(s,v,%f,1,1)", dvref);
  cout <<"\nCalculating and writing "<<testfileName<<" ..."<<endl;
  
  for (int is=0; is<n; is++){
    for (int iv=0; iv<n; iv++){
      double s=is*delta_s;
      double v=iv*delta_v;
      atab[is][iv]=accSimple(s,v,dvref,1,1);
    }
  }
  inout.write_array2d(testfileName,0,smax,n,0,vmax,n,atab,titleString);

    // acc(s,dv)
  
  sprintf(testfileName,"%s.acctab_s_dv",fname);
  sprintf(titleString,"gap s\t\tdv\t\taccSimple(s,%f,dv,1,1)", vref);
  cout <<"\nCalculating and writing "<<testfileName<<" ..."<<endl;
  
  for (int is=0; is<n; is++){
    for (int idv=0; idv<n; idv++){
      double s=0+is*delta_s;
      double dv=dvmin+idv*delta_dv;
      atab[is][idv]=accSimple(s,vref,dv,1,1);
    }
  }
  inout.write_array2d(testfileName,0,smax,n, dvmin,dvmax,n, atab,titleString);


  // acc(v,dv)
  
  sprintf(testfileName,"%s.acctab_v_dv",fname);
  sprintf(titleString,"gap s\t\tdv\t\taccSimple(%f,v,dv,1,1)", sref);
  cout <<"\nCalculating and writing "<<testfileName<<" ..."<<endl;
  
  for (int iv=0; iv<n; iv++){
    for (int idv=0; idv<n; idv++){
      double v=iv*delta_v;
      double dv=dvmin+idv*delta_dv;
      atab[iv][idv]=accSimple(sref,v,dv,1,1);
    }
  }
  inout.write_array2d(testfileName,0,vmax,n, dvmin,dvmax,n, atab,titleString);

 
 
} // end testModelEquations()




//################################################################
void ADAS::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&s0);
  inout.getvar(fp,&ra);
  inout.getvar(fp,&c1);
  inout.getvar(fp,&c2);
  inout.getvar(fp,&eta);
  inout.getvar(fp,&ds);
  fclose(fp);

  if(false){
      cerr << "Sorry; only standard ADAS (choice_variant=5) supported"
	   <<endl
	   <<"To run old projects, use the sources of src_16jun02"<<endl;
      exit(-1);
  }
  rhomax = 1./lveh;

}



//################################################################
void ADAS::calc_eq()
//################################################################

    // Calculates equilibrium velocity of ADAS and ADAS with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:   veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin ADAS.calc_eq()"<<endl;}

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
        double acc = accSimple(s,v_it,0,1,1);
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
        if((v_it<0)||(s<s0)) v_it=0;

        if(false){ 
          cout << "rho="<< rho<< " s="<<s
               <<" v0="<<v0<<" acc="<<acc<<" v_it="<<v_it<<endl;
	}
      }
      veqtab[ir] = v_it;

    }

    calc_rhoQmax();  // Qmax, rhoQmax 
} // ADAS::calc_eq()



//######################################
// acceleration of the ADAS.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double ADAS::accSimple(double s, double v, double dv,
			   double alpha_v0, double alpha_T){


  double tau=1./eta;
  double v0loc=alpha_v0*v0;
  double Tloc=alpha_T*T;
  double thetadv=(dv<0) ? 0 : 1;
  double sc=s0+v0loc*Tloc;
  double sloc=max(s,0.001);
  double a_wanted=0;
  double amaxParam=c2; // for v5 where c2 is not needed

 // {v4_0, v4_1=dv^2/s, v4_2=dv^2*log(sc/s)} // !!
  // ds>0: smoothing v4.0, ds=-1: v4/1, ds=-2: v4.2, ds=-3:v5
  int choice_variant=(ds>=0) ? 0 : (ds>-1.5) ? 1 : (ds>-2.5) ? 2 : 3;


  // ADAS acceleration v4.0 (-dv of Meng = approaching rate = my dv !!)

  if(choice_variant==0){ //v4.0
    double slim=max(0.001, min(ra,s)); // not for kinetic term c1/s*dv*dv*thetadv!!
    double vopt=0.5*(slim-s0)/Tloc - 0.5*ds/Tloc * log( cosh((slim-sc)/ds) / cosh(-sc/ds));
    double lambda_s=tau*(-c1/max(s-s0,0.001)*dv*dv*thetadv 
       - c2/Tloc*(1-tanh((slim-sc)/ds)) * (v-vopt) );
    a_wanted = tau*(lambda_s - 2*c1*log(ra/slim)*dv*thetadv - 2*c2*(v-vopt));
  }


  // ADAS acceleration v4.1 with dv^2/s in Lagrange function
  // ADAS acceleration v4.2 with dv^2*log(sc/s) in Lagrange function

  else if( (choice_variant==1)||(choice_variant==2)){ // v4.1, v4.2
    double amax=2*c2*tau*(tau/Tloc+1)*v0loc;
    double vopt=(s-s0)/Tloc;
    double a_free=amax*(1-v/v0loc);
    double a_int=0;
    if(choice_variant==1){// ADAS acceleration v4.1
      double lambda_s = tau*(-c1/(sloc*sloc) *dv*dv*thetadv -2*c2/T* (v-vopt));
      a_int=tau*(lambda_s-2*c1/sloc *dv*thetadv -2*c2*(v-vopt));
    }
    else if(choice_variant==2){// ADAS acceleration v4.2 
      double lambda_s = tau*(-c1/sloc*dv*dv*thetadv -2*c2/T* (v-vopt));
      a_int=tau*(lambda_s-2*c1*log(sc/sloc)*dv*thetadv -2*c2*(v-vopt));
    }
    a_wanted = min(a_free,a_int);
  }

  else if(choice_variant==3){ // v5
    double vd=max(0., (s-s0)/Tloc);
    double vopt=min(v0loc, vd);
    double aOVM=amaxParam/v0loc*(vopt-v);
    //a_wanted=(dv<0) 
    //  ? aOVM
    //  : min(aOVM, amaxParam/v0loc*(vd-v) -2*c1*dv*exp(s0/sloc)*(1+s0*dv/(eta*sloc*sloc)));
    a_wanted=min(aOVM, amaxParam/v0loc*(vd-v) -2*c1*dv*exp(s0/sloc)*(1+s0*dv/(eta*sloc*sloc)));
    //a_wanted=min(amaxParam/v0loc*(v0loc-v), amaxParam/v0loc*(vd-v) -2*c1*dv*exp(s0/sloc)*(1+s0*dv/(eta*sloc*sloc)));

  }
  return a_wanted;
}

//#############################################################
// acc 
//#############################################################

double ADAS::acc(int it, int iveh, int imin, int imax,
		     double alpha_v0, double alpha_T,
		     const CyclicBuffer* const cyclicBuf)
{

    //#############################################################
    // Local dynamical variables
    //#############################################################


  double s= cyclicBuf->get_s(iveh);
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double dv= v - cyclicBuf->get_v(iveh-1);//vveh[iveh];
  
  //#############################################################
  // space dependencies modelled by alpha_v0, alpha_T
  //#############################################################
  
  //double Tloc  = alpha_T*T;
  //double v0loc = alpha_v0*v0;
  
  //#############################################################
  // actual ADAS formula
  //#############################################################
 
  if(false){
    cout <<"s="<<s<<" v="<<v<<" dv="<<dv
         <<" accSimple="<<accSimple(s,v,dv, alpha_v0, alpha_T)<<endl;
  }

  return  accSimple(s,v,dv, alpha_v0, alpha_T);
  
}



#endif // ADAS_CC

