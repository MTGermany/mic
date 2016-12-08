// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


// c++ 
#include <iostream>
using namespace std;

//own  
#include "Fluct.h"
#include "RandomUtils.h"
//#include "general.h"
#include "InOut.h"



//Initalisierung:
const double Fluct::SMALL_VAL_FLUCT = 0.000003;
const double Fluct::sqrt12 = sqrt(12.);


// #############################################################
// Constructors
// #############################################################

//!!! file <projName>.fluct1 belongs to first model in .heterog and so forth
Fluct::Fluct(){;} 

Fluct::Fluct(const char* projectName, int itype, double dt){
  cout<<"SMALL_VAL_FLUCT = "<<SMALL_VAL_FLUCT<<endl;
  cout<<"sqrt12 = "<<sqrt12<<endl;

  this->dt=dt;
  // reading fluct parameter from *.fluct input file

  char in_fname[199];
  char in_fnameOld[199];
  sprintf(in_fname,"%s.fluct%i",projectName, itype+1);
  sprintf(in_fnameOld,"%s.fluct",projectName);
  
  active=true;  //arne 5 feb 2004: always init variables!
                //on 64 bit g++ version active was set to false!!!

  FILE* fp;
  fp=fopen(in_fname,"r");
  if(fp==NULL){
    cout<<"Fluct::Cstr. No file "<<in_fname
	<<" to read in => No Fluctuations."<<endl;
    //exit(0); //!!!
    active=false;
    varcoeff_s=0;
    tau_s     =10;
    //stddev_dv =0;
    invttc_dv =0;
    tau_dv    =10;
    fp=fopen(in_fnameOld,"r");
    if(fp!=NULL){
      cerr <<" Fluct cstr: error: Only old file "<< in_fnameOld<<" existing;"
	   <<" migrate to "<< in_fnameOld<<"<itype>"
	   <<endl;
      //exit(-1); // Kommentiere aus, falls es auch mit .fluct ohne <n> laufen soll
    } 
  }
  else{
    InOut inout; 
    cout<<"Fluct::Cstr. Read Parameter Q_fluct_v from file "<<in_fname<<endl;
    inout.getvar(fp, &Q_acc);
    inout.getvar(fp, &varcoeff_s);
    inout.getvar(fp, &tau_s);
    inout.getvar(fp, &invttc_dv);
    inout.getvar(fp, &tau_dv);
    fclose(fp); //!! mt mar07; neu: sonst Bug nach etwa 427. Aufruf.
    // Kein Performance-Problem: Fluct File Cstr. wird ja nur bei Initialisierung aufgerufen
    cout <<"Fluct: varcoeff_s="<<varcoeff_s<<endl;//exit(0);
  }

   initialize();
}

Fluct::Fluct(double dt, double Q_acc, double varcoeff_s, double tau_s,
    double invttc_dv, double tau_dv){
  this->dt=dt;
  this->Q_acc =Q_acc;
  this->varcoeff_s =varcoeff_s;
  this->tau_s =tau_s;
  this-> invttc_dv = invttc_dv;
  this->tau_dv =tau_dv;
  active=true; 
  initialize();
  //cout <<"Fluct parameter cstr: Q_acc="<<Q_acc<<" tau_dv ="<<tau_dv <<endl;
}


void Fluct::initialize(){

  // Q_s, Q_dv from fluctuation-dissipation theorem

  s=10; // will be overridden
  v=10; // will be overridden
  dv=0; // will be overridden

  xi_s=0;
  beta_s = exp(-dt/tau_s);
  Q_s=2./tau_s;  // strength (1/s) of a Wiener (1,tau_s) process

  xi_dv=0;
  beta_dv = exp(-dt/tau_dv);
  Q_dv=2./tau_dv;  //strength (1/s) of a Wiener (1,tau_dv) process 

  xi_a=0;
}


// #############################################################
// update
// #############################################################

void Fluct::update(double s, double v, double dv)
{
  //active=true;
  //cout <<"Fluct.update: active="<<active<<" Q_acc="<<Q_acc<<endl;
  if(active)
    {
      this->s=s;
      this->v=v;
      this->dv=dv;
      // dv/dt=a_det + xi, <xi>=0, <<xi(t)*xi(t')=Q_acc*delta(t-t')
      
      if(Q_acc>SMALL_VAL_FLUCT)
	{
	  double randomVar = myRand(); // G(0,1)
	  double random_mu0_sig1 = sqrt12*(randomVar-0.5);
	  xi_a = sqrt(Q_acc/dt) * random_mu0_sig1;
	}

      // dv(est)=dv + s/ttc*xi_v(t),  xi_v(t)= Wiener(1, tau_dv) process 
      // Q_dv such that <(xi_dv)^2> = (stddev_dv)^2
      // => Q_dv=2*(stddev_dv)^2/tau
      
      if(invttc_dv>SMALL_VAL_FLUCT)
	{
	  double randomVar= myRand(); // G(0,1)
	  double random_mu0_sig1=sqrt12*(randomVar-0.5);
	  xi_dv = beta_dv*xi_dv +  sqrt(Q_dv*dt)* random_mu0_sig1;
	  
	  if(false)
	    {
	      cout<<"Fluct.update: sqrt(Q_dv*dt)="<<sqrt(Q_dv*dt)
		  <<" beta_dv="<<beta_dv
		  <<" random_mu0_sig1="<<random_mu0_sig1<<endl;
	    }
	}

      // analoguous to above with xi_s instead of xi_dv
      
      if(varcoeff_s>SMALL_VAL_FLUCT)
	{
	  double randomVar= myRand(); // G(0,1)
	  double random_mu0_sig1=sqrt12*(randomVar-0.5);
	  xi_s = beta_s*xi_s +  sqrt(Q_s*dt)* random_mu0_sig1;
	}
    }
}

// #############################################################
// get_distance_error
// #############################################################

double Fluct::get_distance_error()
{
  return s*(exp(varcoeff_s*xi_s)-1.);
}

// #############################################################
// get_dv_error
// #############################################################

double Fluct::get_dv_error()
{
  double err_dv=s*invttc_dv*xi_dv;

  // obviously, approaching rate error cannot reduce apparent velocity 
  // of predecessor to negative values
  if(err_dv>v-dv)
    {
      err_dv=v-dv;
      if(false)
	{
	  cout <<"Fluct.get_dv_error(): s="<<s<<" v="<<v
	       <<": reduced dv error to v-dv="<<(v-dv)<<endl;
	}
    }
  return err_dv;
}



