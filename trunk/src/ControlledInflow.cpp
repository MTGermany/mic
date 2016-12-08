
// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
// c++ 
#include <iostream>
using namespace std;

//own
#include "general.h"
#include "constants.h"
#include "ControlledInflow.h"

ControlledInflow::ControlledInflow(){
  n_wait=0;
  Qcap=0;
}

double ControlledInflow::getinflow(){return inflow;}

void ControlledInflow::setQcap(double Qcap_invh){Qcap=Qcap_invh/3600.;}

void ControlledInflow::incrWaitVeh(){n_wait++;}
void ControlledInflow::update(double Qin, double dt){
  this->Qin=Qin;
  if (Qin>=Qcap){
    inflow=Qcap;
    n_wait+=(Qin-inflow)*dt;
  }
  else{
    if(n_wait>0){
      inflow = min(Qcap, Qin+n_wait/dt);
      n_wait+=(Qin-inflow)*dt;
    } else{inflow = Qin;}
  }
  //cout << "update: inflow="<<inflow<<endl;
}

void ControlledInflow::write_in_log(char projectName[], int it, double dt){
  // static int DNWRITE=10;
  static bool First=TRUE;
  char   out_fname[199];

  sprintf(out_fname,"%s.in_log",projectName);
  FILE  *outfile;                  
  outfile     = First ? fopen(out_fname,"w") : fopen(out_fname,"a");
  if (First){
    First=FALSE;
    fprintf(outfile,"Time(min)\tQin(1/h)\tcontr_inflow(1/h)\tn_veh\n");
  }
  if(true){
    // if(it%DNWRITE==0){
    fprintf(outfile,"%f\t%f\t%f\t%i\n", it*dt/60, 3600*Qin, 3600*inflow,static_cast<int>(n_wait));
    //cout << "Qcap="<<Qcap<<endl;
  }
  fclose (outfile);
}

