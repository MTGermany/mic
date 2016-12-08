#ifndef PTmodel_CC
#define PTmodel_CC

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
#include "PTmodel.h"
#include "CyclicBuffer.h"


// Instructions to incorporate new model: see PTmodel.h

//######################################
// File Constructor
//######################################

PTmodel::PTmodel(const char fname[], double dt)
{
  cout << "\nin PTmodel file Cstr: fname= "<<fname<<endl;

  this->dt=dt;

  get_modelparams(fname); // incl. derived quantities such as rhomax

  // initialize Wiener process variable
  wiener=0;

  // in File Constructor: Initialize Tables of d(U_PT)/da and d^2(U_PT)/da^2
  
  for (int i=0; i<NTABMAX; i++){
  
    double a=bmax*(-1+2*i/((double)( NTABMAX-1)));
    double x=a/a0;
    double lorenz=1/(1+x*x);
    double g=x*pow(lorenz,delta);
    double gx=pow(lorenz, delta) - 2*delta*x*x*pow(lorenz, delta+1);
    double gxx=-6*delta*x*pow(lorenz, delta+1)
      +4*delta*(delta+1)*pow(x,3)*pow(lorenz,delta+2);

    double prefactor=wm+0.5*dw*(1+tanh(x));
    double cosh2=pow(cosh(x), 2);

    uPTatab[i]= 1/a0 * (prefactor*gx+ 0.5*dw*g/cosh2)  ;
    uPTaatab[i]=1/(a0*a0) * (prefactor*gxx + dw/cosh2 * (gx-tanh(x)*g));
  } 

 
  // in File Constructor: initialize tables for equilibrium vel and fundamental diagram

  calc_eq();

  // optionally test PT model calculations
  
  if(false){
    testModelEquations(fname);
  }
  //counter_errZstarNeg=0; // error zstar<0 (pCrash>=0.5) 
  //counter_errAccLtMin=0; // error acc<-bmax
  //counter_errUcurvPos=0; // error U(a*)=min instead of max
  //counter_errVarNeg=0; // error estimated variance <0 (related to errUcurvPos)
  //counter_errAoutOfRange=0; // this should not happen because should be catched


  
 } // end file constructor
//######################################################







//########################################################
// Test routines for 	PT model calculations		   
//########################################################

void PTmodel::testModelEquations(const char fname[]){
  
  cout <<"PTmodel.testModelEquations: test everything for parameter set "
       <<fname<<endl;


  // Test accelerations on screen

  cout <<"\n(1) Test  some accelerations on screen"<<endl;

  if(false){
    double vtest=0.0;
    double dvtest=0;
    for (int i=0; i<3; i++){
      double stest=2*i;
      cout <<"s="<<stest<<" v="<<vtest<<" dv="<<dvtest
	 <<" accSimple="<< accSimple(stest,vtest,dvtest,1,1)
	 <<endl;
    }
  }

 // Test accelerations by looking at files

  cout <<"\n(2) Calculate+Write some acceleration matrices in files"<<endl;

  InOut inout;


  double atab[101][inout.NYMAX];
  int n=101;
  double smax=60;
  double sref=20;
  double delta_s=smax/((double)(n-1));
  double vmax=v0;
  double vref=20;
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

  //################################
  if(false){
    cout <<"\n(3) Tables uPTatab for U'_PT(a), uPTaatab for U''_PT(a):" <<endl;

    for (int i=0; i<NTABMAX; i++){
      double a=bmax*(-1+2*i/((double)( NTABMAX-1)));
      cout <<"i="<<i<<" a="<<a<<"  uPTatab[i]="<< uPTatab[i]
	   <<"  uPTaatab[i]="<< uPTaatab[i]<<endl;
    }
  }
  
  //exit(0);
} // end testModelEquations()




//################################################################
void PTmodel::get_modelparams(const char fname[]){
//################################################################
  FILE *fp;
  InOut inout;
  fp=fopen(fname,"r");
  filecheck(fp,fname);

  inout.getvar(fp,&lveh); // vehicle length 

  inout.getvar(fp,&v0);  // see .h file
  inout.getvar(fp,&tauOVM);

  inout.getvar(fp,&wm);
  inout.getvar(fp,&a0);
  inout.getvar(fp,&gamma);

  inout.getvar(fp,&wc);
  inout.getvar(fp,&taumax);
  inout.getvar(fp,&alpha);
  
  inout.getvar(fp,&beta);
  inout.getvar(fp,&tauCorr);

  inout.getvar(fp,&bmax);
  inout.getvar(fp,&s0);


  fclose(fp);

  // double lveheff=lveh+s0;
  rhomax = 1./lveh;
  delta=0.5*(1-gamma);		 
  dw=1-wm;

}



//################################################################
void PTmodel::calc_eq()
//################################################################

    // Calculates equilibrium velocity of PTmodel and PTmodel with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq

  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(true){cout << "\nin PTmodel.calc_eq()"<<endl;}

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

    for(int ir=1; ir<=NRHO; ir++) //!!
    //for(int ir=1; ir<=2; ir++)
    {
      double rho = rhomax*ir/NRHO;
      double s   = 1./rho - 1./rhomax;

      // start iteration with equilibrium velocity for the previous density
      v_it             = veqtab[ir-1];
      if(false) cout << endl;

      for (int it=1; it<=itmax; it++){
        double acc = accSimple(s,v_it,0,1,1);
        double dtloc = dtmax*v_it/(v0+0.01) + dtmin; // it. step in [dtmin,dtmax]

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
} // PTmodel::calc_eq()



double PTmodel::get_uPTa(double a){
  return (a<= -bmax) ? wm*(1-2*delta)*pow(a/a0, -2*delta)
    : (a<bmax) ? intp(uPTatab, NTABMAX, a, -bmax, bmax)
    : (wm+dw)*(1-2*delta)*pow(a/a0, -2*delta);
}
  
double PTmodel::get_uPTaa(double a){
  return (a<= -bmax) ? - wm*2*delta*(1-2*delta)*pow(a/a0, -2*delta-1)
    : (a<bmax) ? intp(uPTaatab, NTABMAX, a, -bmax, bmax)
    : -(wm+dw)*2*delta*(1-2*delta)*pow(a/a0, -2*delta-1);
}




//######################################
// acceleration of the PTmodel.
// Argument parameters:

// s=net distance (m),
// v=own velocity (m/s)
// dv=approaching rate (v-v_front) to the front vehicle (m/s)
// alpha_v0, alpha_T = multiplicators of v0 and T (flowc bottl)
//######################################

double PTmodel::accSimple(double s, double v, double dv,
			   double alpha_v0, double alpha_T){


 //######################################
  // locally varying parameters
 //######################################

  double v0loc=alpha_v0*v0;
  double alphaloc=alpha_T*alpha; // alpha = vel. var. coefficient; the higher, the higher T


  //######################################
  // free acceleration: OVM or Gipps like
  //######################################

  //double amax=v0loc/tauOVM;
  double afree=(v0loc-v)/tauOVM; // NICHT amax*(1 - v/v0loc) wg. moegl. v0=0!
  //double afree=(v<=v0) ? amax : -amax; // Gipps like
  // if(v<0.5*v0){afree=(0.5+1.0*v/v0)*amax;}


  //######################################
  // PT acceleration (i): Initial guess for linear generalized PT utility
  //######################################
  
  double sloc=max(s-s0,0.01);
  double vloc=max(v,0.01); // not the same as  v0loc!

  //double tau=(dv>sloc/taumax) ? sloc/dv : taumax; // TTC limited to taumax
  //double tau=(dv>2*sloc/taumax) ? 2*sloc/dv : taumax; // !!! other tau def
  double bcomf=2; // !!! third def; avoids high stop-decel for stable param vals
  double taumaxmod=max(taumax, 0.5*dv/bcomf);
  double tau=(dv>sloc/taumaxmod) ? sloc/dv : taumaxmod;

  double za=0.5*tau/(alphaloc*vloc);  // z'(a)=const, z=arg. of std normal distr.
  double logval=log(a0*wc*za/sqrt(2*PI));
  if(logval<=0){
    PTmodel::counter_errZstarNeg++;
    cout <<"PTmodel.accSimple: zstar<0 or no solution => prob of approaching"
	 <<" nearer than s0 or crash >1/2! count="<<PTmodel::counter_errZstarNeg
           <<endl; 
    return (-bmax);
  }
  double zstar=(logval>0) ? -sqrt(2*logval) : 0; // guess in arg of Gaussian 
  double astar=2/tau*(sloc/tau-dv+alphaloc*v*zstar);
  if(astar<-bmax){
    counter_errAccLtMin++;
    cout <<"PTmodel.accSimple: initial guess acc<-bmax; returning -bmax"<<
      ", count="<<counter_errAccLtMin<<endl;
    return (-bmax);
  }

  bool testAcc=false;
  
  if(testAcc){
    cout <<"PTmodel.accSimple: in testAcc:"<<endl;
    cout <<" s="<<s<<" v="<<v<<" dv="<<dv<<endl;
    cout <<" tau="<<tau<<" za="<<za<<" zstar="<<zstar<<endl;
    cout << " 0th iteration: astar="<<astar<<endl;
  }

  
  //######################################
  // PT acceleration (ii): Approximate solution for nonlineasr PT utility
  // two times Newton
  //######################################

  //first iteration (zstar from above)
  double ua=get_uPTa(astar)-wc*gaussdens(zstar)*za;            // U'(a)
  double uaa=get_uPTaa(astar)+wc*gaussdens(zstar)*zstar*za*za;            // U'(a)
  astar=(uaa<0) ? astar-ua/uaa : astar;
  if(uaa>=0){
    counter_errUcurvPos++;
    cout<<"PTmodel.accSimple: Warning: U''(a)>0 => Newton wants to go to "
	<<" utility minimum instead maximum, count="<<counter_errUcurvPos <<endl;
  }
  if(testAcc){ cout << " 1th iteration: astar="<<astar<<endl;}

  // further iterations (zstar in recursion)
  for (int k=1; k<2; k++){
    zstar=(dv+0.5*astar*tau-sloc/tau)/(alphaloc*v);
    ua=get_uPTa(astar)-wc*gaussdens(zstar)*za;            // U'(a)
    uaa=get_uPTaa(astar)+wc*gaussdens(zstar)*zstar*za*za;            // U'(a)
    astar=(uaa<0) ? astar-ua/uaa : astar;
    if(testAcc){ cout <<" " <<(k+1)<<"th iteration: astar="<<astar<<endl;}
  }
  
  //######################################
  // standard deviation of correlated errors in acceleration
  //######################################

  double vara = -1/(beta*uaa);
  double stddeva=(vara>0) ? sqrt(vara) : 0;
  
  if (vara<=0){
    counter_errVarNeg++;
    cout <<"PTmodel:accSimple:Warning: variance-1/(beta*U''(a))="
	 <<vara<<" negative! count="<<counter_errVarNeg<<endl;
  }
  
  //######################################
  // Implementing correlations with unit Wiener process
  // Wiener variable updated in next higher-level function PTmodel::acc
  //######################################
  
  double aPT= astar+stddeva*wiener;
  double aVeryNear=-0./sqrt(sloc); // -0.2/sqrt(sloc);  quick hack to introduce s0 effect !!!
  double aWanted=min(afree, aPT+aVeryNear);

  //#################################
  // nur Fehlertest!
  //#################################
  
  if (!((aWanted>-10000)&&(aWanted<10))){
    counter_errAoutOfRange++;
    cerr <<"PTmodel.accSimple: acc="<<aWanted<<" not in right range!!"<<endl;
    cerr <<"s="<<s<<" v="<<v<<" dv="<<dv<<endl;
    //double tau=(dv>sloc/taumax) ? sloc/dv : taumax; // TTC lim. to taumax
    double tau=(dv>2*sloc/taumax) ? 2*sloc/dv : taumax; // !! other tau def

    double za=0.5*tau/(alphaloc*max(v,0.01));  // z'(a)=const, z=arg. of standard normal distr.
    double zstar=-sqrt(2*log(a0*wc*za/sqrt(2*PI))); // gues in arg of Gaussian
    double astar=2/tau*(sloc/tau-dv+alphaloc*v*zstar);
    cerr   <<"Init. guess:  tau="<<tau<<" za="<<za<<" zstar="<<zstar
	   <<" astar="<<astar<<" wiener="<<wiener<<endl;
    cerr<<"final value: aPT="<<aPT<<", count="<<counter_errAoutOfRange<<endl;
    exit(-1);
  }
  return max(aWanted, -bmax);
}

//######################################################
// acc 
//######################################################

double PTmodel::acc(int it, int iveh, int imin, int imax,
		     double alpha_v0, double alpha_T,
		     const CyclicBuffer* const cyclicBuf)
{

    // Local dynamical variables

  double s= cyclicBuf->get_s(iveh);
  double v= cyclicBuf->get_v(iveh); //vveh[iveh];
  double dv= v - cyclicBuf->get_v(iveh-1);//vveh[iveh];
  
    // update dynamical variables in class scope

  wiener=wienerUpdate(wiener,tauCorr,dt);

  //#############################################################
  // actual PTmodel formula
  //#############################################################

  
  return  accSimple(s,v,dv,alpha_v0,  alpha_T);
  
}



#endif // PTmodel_CC

