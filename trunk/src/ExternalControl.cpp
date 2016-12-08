
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


ExternalControl::ExternalControl(){;}


ExternalControl::ExternalControl(char* projectName, int number,int vehIndex){ 
  // vehicle index from .leadvehs in RoadSection
  // only informative function here

  cout <<"ExternalControl Cstr: number="<<number
       <<" vehIndex="<<vehIndex<<endl;

  this->vehIndex=vehIndex;
  this->useVel=false;
  this->useVelBosch=false;
  this->useAcc=false;

   // reading externalControl flow

  char in_fname[199];

  // test if controlled-vehicle's data in file .brake%i

  sprintf(in_fname,"%s.brake%i",projectName, number);
  ifstream  infile (in_fname, ios::in);
  if(infile){ 
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
  // file format =that of Bosch: sim/drivedata.txt

  // !! (martin jun12)
  // old: Original format in Version 2
  // .carfollowBosch<n>= ~/trafficSim/BoschData/Stuttgart/boschDataStuttgart.fcd[1-3].original2
  // since jun3 12: modified format 
  // .carfollowBosch<n>= ~/trafficSim/BoschData/Stuttgart/boschDataStuttgart.fcd[1-3]
  // Attention: More than 1 s inconsistency in jump at Set 3!!

  sprintf(in_fname,"%s.vleadBosch%i",projectName, number);
  cout <<"in_fname="<<in_fname<<endl;
  ifstream  infile2 (in_fname, ios::in);
  if(infile2){ 
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
    useVel=true;
    InOut inout;
    inout.get_col(in_fname, 1, n_times, times);
    inout.get_col(in_fname, 2, n_times, vel);

  
    if(n_times>NDATAMAX){
      cerr<<"ExternalControl Cstr: Error: to many "
	  <<" (>ExternalControl.NBC) externalControl data!\n";
      exit(-1);
    } 
    if(useAcc){
      cerr<<"ExternalControl Cstr: Error: Cannot simultaneously "
	  <<" prescribe acceleration (.brake%i) and v (.vlead%i)!\n";
      exit(-1);
    }
    if(useVelBosch){
      cerr<<"ExternalControl Cstr: Error: Cannot simultaneously "
	  <<" prescribe velocity in Bosch format (.vleadBosch%i)"
	  <<" and in generic format (.vlead%i) !\n";
      exit(-1);
    }

  }

  // no file .vlead$i or .brake$i found
  
  if((!useVel)&&(!useVelBosch)&&(!useAcc)){
    cerr<<" ExternalControl Cstr: Error: File .leadveh or .ctrlRegions in single-veh mode exists,"
	<<" but No file .vlead%i, .vleadBosch%i or .brake%i!\n";
    exit(-1);
  }
}

   

double ExternalControl::getAcc(double time_s){
  return( intpextp(times, acc, n_times, time_s));
} 

double ExternalControl::getVel(double time_s){
  return (useVelBosch) 
    ? intp(vel, n_times, time_s, 0, 0.1*n_times)
    : intpextp(times, vel, n_times, time_s);
} 

double ExternalControl::getGapBack(double time_s){
  return (useVelBosch) 
    ? intpextp(times, gapBack, n_times, time_s) : 0;
} 

double ExternalControl::getVelBack(double time_s){
  return (useVelBosch) 
    ? intpextp(times, velBack, n_times, time_s) : 0;
} 

double ExternalControl::getGapBackJump(double time_s){
  return (useVelBosch) 
    ? getGapBack(time_s+dtData)-getGapBack(time_s-dtData) : 0;
} 

bool ExternalControl::newTargetDetected(double time_s){
  if (!useVelBosch){return false;}
  else{
    double gapBackDiffCrit=5; // minimum 1 vehicle length
    double dtminNewTargets=3;
    double gapBackDiff=getGapBackJump(time_s)
      +2*dtData*(getVel(time_s)-getVelBack(time_s));
    if((gapBackDiff>gapBackDiffCrit)&&(time_s-tLastTargetEntered>dtminNewTargets)){
      tLastTargetEntered=time_s;
      return true;
    }
    else return false;
  }
}

// in .h file double ExternalControl::getQueueLength(){return queueLength_veh;}



