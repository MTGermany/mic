
// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
#include <fstream>
using namespace std;

#include "constants.h"
#include "InOut.h"
#include "general.h"
#include "ExternalControl.h"

/**
#######################################################
Vehicles that are controlled externally
%#######################################################

One or more vehicles are controlled in some way if <proj>.leadvehs exists 
giving an 1d array of the vehicle indices to be controlled

The way of control is specified by the existence of exactly one of
the following extensions 
(for each line of <proj>.leadvehs, it can be a different one):

(1) <proj>.vlead<number> exists: control of speed of the vehicle with 
    index in the number'th line of <proj>.leadvehs, leadvehs[number-1]

(2) <proj>.brake<number> exists: externally control of acceleration

(3) <proj>.vleadBosch<number> exists: 
    Test car-following data of Mannstetten/Bosch .vleadBosch<n>

(4) <proj>.jump<number> exists: The gap, speed, and desired speed is 
    changed simulating a cutin/cutout. In contrast to (1)-(3), this is a 
    one-off (nonpermanent) control for each indicated time in the .jump file
    Outside jump times, the controlled vehicle drives with its 
    normal model, before the first jump controlled by the IC
    since generic model has no parameter v0, the generic alpha_v0 is chosen
    to represent new vehicles modelled by the jumps 
    (it is easy to add alpha_T, alpha_a later on)

NOTE: Both s and v (.svlead<n>) in micVLA (oct06)

NOTE2: It lies in the responsibility of the simulator to avoid crashes
*/


ExternalControl::ExternalControl(){;}


ExternalControl::ExternalControl(char* projectName, 
				 int number, int vehIndex){

  // vehicle index from .leadvehs in RoadSection


  cout <<"ExternalControl Cstr: number="<<number
       <<" vehIndex="<<vehIndex<<endl;

  this->vehIndex=vehIndex;
  this->useVel=false;
  this->useVelBosch=false;
  this->useAcc=false;
  this->useJump=false;
  this->nCtrlFiles=0;
  this->i_jump=0; // only relevant if useJump
  this->newJump=false;

   // reading externalControl flow

  char in_fname[199];

  // test if controlled-vehicle's data in file .brake%i

  sprintf(in_fname,"%s.brake%i",projectName, number);
  ifstream  infile (in_fname, ios::in);
  if(infile){ 
    nCtrlFiles++;
    useAcc=true;
    InOut inout;
    inout.get_array(in_fname, n_times, times, acc);
    if(n_times>NBC){
      cerr<<
        "ExternalControl Cstr: Error: to many (>NBC) externalControl data!\n";
      exit(-1);
    }
  }

  // test if controlled-vehicle's data in file .vleadBosch%i
  // => file format is that of Bosch: sim/drivedata.txt

  // .carfollowBosch<n>
  //   = ~/trafficSim/BoschData/Stuttgart/boschDataStuttgart.fcd[1-3]
  // Attention: More than 1 s inconsistency in jump at Set 3!!

  sprintf(in_fname,"%s.vleadBosch%i",projectName, number);
  ifstream  infile2 (in_fname, ios::in);
  if(infile2){ 
    nCtrlFiles++;
    useVelBosch=true;
    InOut inout;
    //inout.get_col(in_fname, 6, n_times, vel); // original Bosch
    inout.get_col(in_fname, 2, n_times, vel);  // vLead from data point of view
    inout.get_col(in_fname, 1, n_times, times);  
    inout.get_col(in_fname, 5, n_times, gapBack);  //  s from data point of view
    inout.get_col(in_fname, 4, n_times, velBack);  //  v from data point of view
    dtData=times[1]-times[0];
    tLastTargetEntered=0;

    if(n_times>NDATAMAX){
      cerr<<
         "ExternalControl Cstr: Error: to many "
	  <<" (>ExternalControl.NBC) externalControl data!\n";
      exit(-1);
    }
    if(useAcc){
      cerr<<"ExternalControl Cstr: Error: Cannot simultaneously "
	  <<" prescribe acceleration (.brake%i) and v (.vlead%i)!\n";
      exit(-1);
    }
  }

  // test if controlled-vehicle's data in file .vlead%i
  // file format = two columns: t, v_lead(t)

  sprintf(in_fname,"%s.vlead%i",projectName, number);
  ifstream  infile3 (in_fname, ios::in);
  if(infile3){ 
    nCtrlFiles++;
    useVel=true;
    InOut inout;
    if(n_times>NDATAMAX){
      cerr<<"ExternalControl Cstr: Error: to many "
	  <<" (>ExternalControl.NBC) externalControl data!\n";
      exit(-1);
    } 
    inout.get_col(in_fname, 1, n_times, times);
    inout.get_col(in_fname, 2, n_times, vel);

  }

  // test if controlled-vehicle's data in file .jump%i

  sprintf(in_fname,"%s.jump%i",projectName, number);
  ifstream  infile4 (in_fname, ios::in);
  if(infile4){ 
    cout <<" ExternalCtrl Cstr: leading veh with jumps"<<endl;//exit(0);
    nCtrlFiles++;
    useJump=true;
    InOut inout;
    inout.get_col(in_fname, 1, n_times, times);
    if(n_times>NBC){
      cerr<<
        "ExternalControl Cstr: Error: to many (>NBC) externalControl data!\n";
      exit(-1);
    }
    inout.get_col(in_fname, 2, n_times, s_jump);
    inout.get_col(in_fname, 3, n_times, v_jump);
    inout.get_col(in_fname, 4, n_times, alpha_v0_jump);

    // add a last artificial line to simplify bookkeeping

    n_times++;
    times[n_times-1]=1e9; // longer than any sim time (hack but OK)
    s_jump[n_times-1]=s_jump[n_times-2];
    v_jump[n_times-1]=v_jump[n_times-2];
    alpha_v0_jump[n_times-1]=alpha_v0_jump[n_times-2];

    if(false) for (int i=0; i<n_times; i++){
      cout <<"times="<<times[i]<<" s_jump="<<s_jump[i]
	   <<" v_jump="<<v_jump[i]<<" alpha_v0_jump="<<alpha_v0_jump[i]<<endl;
    }

  }

  // more than one or not a single file 
  // .vlead<n>, .vleadBosch<n>, .brake<n>, .jump<n>  found
  
  if(nCtrlFiles>1){
    cerr<<" ExternalControl Cstr: Error: cannot prescribe more than one\n"
	<<" external control file .vlead<n>, .vleadBosch<n>, .brake<n>,"
	<<" or .jump<n>"<<endl;
    exit(-1);
  }

  if(nCtrlFiles==0){
    cerr<<" ExternalControl Cstr: Error: File .leadveh or .ctrlRegions in single-veh mode exists," <<endl
	<<" but not a single ctrl file cannot prescribe more than one\n"
	<<" external control file .vlead<n>, .vleadBosch<n>, .brake<n>,"
	<<" or .jump<n>"<<endl;
    exit(-1);
  }

} // ExternalControl constructor

   

double ExternalControl::getAcc(double time_s){ // useAcc
  return( intpextp(times, acc, n_times, time_s));
} 

double ExternalControl::getVel(double time_s){ // useVel or useVelBosch
  return (useVelBosch) 
    ? intp(vel, n_times, time_s, 0, 0.1*n_times)
    : intpextp(times, vel, n_times, time_s);
} 

double ExternalControl::getVelJump(){ // discrete, not interpol. as otherwise
  return (useJump) ? v_jump[max(i_jump-1,0)] : 0;
} 

double ExternalControl::getAlphaV0Jump(){
  return (useJump) ? alpha_v0_jump[max(i_jump-1,0)] : 0;
} 


double ExternalControl::getGapBackJump(double time_s){// Bosch or jumps
  return (useVelBosch) 
    ? getGapBack(time_s+dtData)-getGapBack(time_s-dtData)
    : (useJump) ? s_jump[max(i_jump-1,0)] : 0;
} 

double ExternalControl::getGapBack(double time_s){ // only Bosch
  return (useVelBosch) 
    ? intpextp(times, gapBack, n_times, time_s) : 0;
} 

double ExternalControl::getVelBack(double time_s){ // only Bosch
  return (useVelBosch) 
    ? intpextp(times, velBack, n_times, time_s) : 0;
} 


bool ExternalControl::newTargetDetected(double time_s){
  if (useVelBosch){
    double gapBackDiffCrit=5; // minimum 1 vehicle length
    double dtminNewTargets=3;
    double gapBackDiff=getGapBackJump(time_s)
      +2*dtData*(getVel(time_s)-getVelBack(time_s));
    if((gapBackDiff>gapBackDiffCrit)
       &&(time_s-tLastTargetEntered>dtminNewTargets)){
      tLastTargetEntered=time_s;
      return true;
    }
    else return false;
  }
  else return useJump&&newJump;
}


// check if a jump of a controlled vehicle occurred if useJump

bool ExternalControl::checkNewJump(double time_s){
  newJump=useJump&&(times[i_jump]<time_s);
  if(newJump){
    i_jump++; // index guaranteed to exist: times[n_times-1] always >sim time
    cout <<"ExternalControl.checkJump: jump! t="<<time_s
    	 <<" new laggap="<<getGapBackJump(time_s)
    	 <<" speed="<<getVelJump()
    	 <<" alpha_v0="<<getAlphaV0Jump()
    	 <<endl;
  }
  return newJump;
}


// in .h file double ExternalControl::getQueueLength(){
// return queueLength_veh;}



