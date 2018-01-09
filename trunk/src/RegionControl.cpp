
/* c */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

using namespace std;

// c++ 
#include <iostream>
#include <fstream>
#include <string.h>

#include "./RegionControl.h"
#include "./InOut.h"
#include "./general.h"
#include "./constants.h"


//##################################################
// Notice:  Actual control in RoadSection
//##################################################

RegionControl::RegionControl(){;}
RegionControl::RegionControl(char* projectName,
			     ProjectParams proj, int regionIndex,
			     double xmin, double xmax,
			     double tmin, double tmax){

  cout <<"RegionControl Cstr:"
       <<" regionIndex="<<regionIndex
       <<" tmin="<<tmin<<" tmax="<<tmax
       <<" xmin="<<xmin<<" xmax="<<xmax<<endl;

  this->regionIndex=regionIndex;
  this->proj=proj;
  this->xmin=xmin;
  this->xmax=xmax;
  this->tmin=tmin;
  this->tmax=tmax;
  this->time=0;
  
  useSpeedlimit=false;
  useAcc=false;
  useAlphaT=false;
  useAlphaV0=false;

  useSingle=(xmax<=xmin);
  vehicleFound=false;
  vehicleJustFound=false;
   
  bool oneFileFound=false;
  char filelist[512];
  char errmsg[512];
  sprintf(filelist,"%s.speedlimit_Region%i, %s.acc_Region%i, %s.alpha_T_Region%i, or %s.alpha_v0_Region%i",
	  projectName,regionIndex,projectName,regionIndex,
	  projectName,regionIndex,projectName,regionIndex);
  sprintf(errmsg,"Cannot prescribe speed limit and acceleration simultaneously;\n only one file of the first two files on the list %s should exist!",filelist);
  
   // reading regionControl flow

  char in_fname[200];
  sprintf(in_fnames," ");
  // test if controlled-vehicle's data in a file of name .accRegion%i

  sprintf(in_fname,"%s.acc_Region%i",projectName, regionIndex);
  ifstream  infile (in_fname, ios::in);
  if(infile){ 
     cout <<"RegionControl Cstr: accControl activated"<<endl;
     useAcc=true;
     oneFileFound=true;
     sprintf(in_fnames, "%s %s", in_fnames, in_fname);
     InOut inout;
     inout.get_array(in_fname, n_acc, times, acc);
     if(n_acc>NBC){
       cerr<< "RegionControl Cstr: Error: to many (>NBC) regionControl data!\n";
      exit(-1);
    }
  }

  // test if controlled-vehicle's data in a file of name .speedlimitRegion%i
   sprintf(in_fname,"%s.speedlimit_Region%i",projectName, regionIndex);
  ifstream  infile2 (in_fname, ios::in);
  if(infile2){
    cout <<"RegionControl Cstr: speedlimit activated"<<endl;
    useSpeedlimit=true;
    oneFileFound=true;
    sprintf(in_fnames, "%s %s", in_fnames, in_fname);
    InOut inout;
    inout.get_array(in_fname, n_speedlimit, locations, speedlimit);
    if(n_speedlimit>NBC){
      cerr<< "RegionControl Cstr: Error: to many (>NBC) regionControl data!\n";
      exit(-1);
    }
    if(ctrlBySpeedlimit()){cout <<"ctrlBySpeedlimit is true"<<endl;}
  }

    // test if controlled-vehicle's data in a file of name .alpha_T_Region%i
  sprintf(in_fname,"%s.alpha_T_Region%i",projectName, regionIndex);
  ifstream  infile3 (in_fname, ios::in);
  if(infile3){ 
    cout <<"RegionControl Cstr: control by alpha_T activated"<<endl;
    useAlphaT=true;
    oneFileFound=true;
     sprintf(in_fnames, "%s %s", in_fnames, in_fname);
    InOut inout;
    inout.get_array(in_fname, n_alpha_T, locations, alphaT);
    if(n_alpha_T>NBC){
      cerr<< "RegionControl Cstr: Error: to many (>NBC) regionControl data!\n";
      exit(-1);
    }
  }
    
   // test if controlled-vehicle's data in a file of name .alpha_v0Region%i

  sprintf(in_fname,"%s.alpha_v0_Region%i",projectName, regionIndex);
  ifstream  infile4 (in_fname, ios::in);
  if(infile4){ 
    cout <<"RegionControl Cstr: control by alpha_v0 activated"<<endl;
    useAlphaV0=true;
    oneFileFound=true;
     sprintf(in_fnames, "%s %s", in_fnames, in_fname);
    InOut inout;
    inout.get_array(in_fname, n_alpha_v0, locations, alphaV0);
    if(n_alpha_v0>NBC){
      cerr<< "RegionControl Cstr: Error: to many (>NBC) regionControl data!\n";
      exit(-1);
    }
    cout <<"useAlphaV0="<<useAlphaV0<<endl;
  }
    
  // no input file found

  if((!useSingle)&&(!oneFileFound)){
    cerr<<" RegionControl Cstr: Error: File .regionCtrl exists,"
	<<" but not a single  file of the list "<<filelist<<"!"<<endl; 
    exit(-1);
  }

    // conflicting input files found
  
  if(useAcc&&(useSpeedlimit||useAlphaV0||useAlphaT)){
    cerr<<" RegionControl Cstr: Error: Conflicting input files "
	<<" found for reginIndex "<<regionIndex<<": "<<endl
	<<" cannot prescribe acceleration and speed limit,"
	<<" or speed or time headway multiplicator!"<<endl;
    exit(-1);
  }
}




//###################### set control ##########

void RegionControl::update(Vehicle  veh[], int imin, int imax, double time){

  //bool test=(regionIndex==2); // regionIndex as in ctrlRegions file
  //bool test=true;
  bool test=false;

  this->time=time;

  if(inTimeInterval(time)){
    if(test){
      cout <<"\n  in RegionControl.update: t="<<time
	   <<" regionIndex="<<regionIndex<<endl;
    }

    //############################################
    // ctrl single vehicles
    //############################################

    if(useSingle){
      vehicleJustFound=false;
      if(!vehicleFound){
	double xdown=veh[imin].getPos();
	for (int iveh=imin+1; (iveh<=imax)&&(!vehicleFound); iveh++){
	  if( (veh[iveh].getPos()-xmin)*(xdown-xmin)<=0){
	    ivehSingle=iveh;
	    vehicleFound=true;
	    vehicleJustFound=true;
	    // initialize control
	    cout <<"RegionControl.update: "
		 <<" event-oriented single-vehicle control since xmax<xmin:"
		 <<endl<<" found single vehicle index "<<ivehSingle<<" at x="
	    <<veh[ivehSingle].getPos()<<endl;
	  }
	}
	//exit(0);
      }
    }

    //#############################################
    // real region control (useSingle=false)
    //#############################################


    bool relevantForSomeVeh 
      =(!useSingle)                        // not applicable for useSingle!
      && (!isUpstream(veh[imin].getPos())) // first vehicle not upstream
      && (!isDownstream(veh[imax].getPos())); // last vehicle not downstream

    if(relevantForSomeVeh){
      //if(ctrlByAcc()){writeInfo(); }
      
      bool downstream=false;// start with imax=most upstream	  
      int iveh=imax;

      cout <<"RegionControl.update: in real region control"<<endl;
      while( (iveh>=imin)&&(!downstream)){
	if(test){
	  cout <<"t="<<time<<" iveh="<<iveh
	       <<" xmin="<<xmin<<" xmax="<<xmax
	       <<" x="<<veh[iveh].getPos()
	       <<" isUpstream="<<isUpstream(veh[iveh].getPos())
	       <<" isDownstream="<<isDownstream(veh[iveh].getPos())
	       <<" inSpaceInterval="<<inSpaceInterval(veh[iveh].getPos())
	       <<endl;
	}
	if(inSpaceInterval(veh[iveh].getPos())){
	  setControl(veh[iveh]); // now at downstream boundary
	  if(test){
	        cout <<"set regionControl for vehicle "<<iveh
		     <<" at x= "<< veh[iveh].getPos()<<endl;
	  }
	}

	// deactivate control for vehicle just having left the ctrl region

	double maxDisplacementPerTimestep=50*proj.get_dt();
	downstream=isDownstream(veh[iveh].getPos());
	if(downstream&&(veh[iveh].getPos()<xmax+maxDisplacementPerTimestep)){
	  resetControl(veh[iveh]);
	  if(test){
	        cout <<"reset regionControl for vehicle "<<iveh
		     <<" at x= "<< veh[iveh].getPos()<<endl;
	  }
	}

	iveh--;
      }
    }//relevantForSomeVeh

    if(test){cout<<"end of actual RegionControl.update loop"<<endl<<endl;}

  }//inTimeInterval(time)


  //#############################################
  // check if control time interval just ended 
  // => if so and real region control (useSingle=false), 
  // reset every vehicle inside space interval of ctrl region
  //#############################################

  if((!useSingle) && (time>tmax) &&(time<tmax+1.1*proj.get_dt())){

    if(test){cout <<" t="<<time<<" region control time for regionIndex"
		  <<regionIndex <<" just ended!"<<endl;
    }

    for (int iveh=imin; iveh<=imax; iveh++){ 
      // addtl. test to avoid side effects with other region ctrls 
      // taking place simultaneously
      if(inSpaceInterval(veh[iveh].getPos())){ 
        resetControl(veh[iveh]);
	if(test){cout <<" reset control of veh "<<iveh
		      <<" at x="<<veh[iveh].getPos()<<endl;
	}
      }
    } 
    if(test){cout<<"end of test for temporal end of regionControl"<<endl<<endl;}

  } // check if control time interval just ended 

  
}// end update






void RegionControl::setControl(Vehicle& veh){
  if(useSpeedlimit){veh.setSpeedlimit(getSpeedlimit(veh.getPos()));}
  else if(useAcc){
    double localtime=time-veh.get_timeBeginAccControl();
    veh.setAccExternal(time); // not localtime; 
    veh.setAcc(getAcc(localtime)); // localtime does not work for event-oriented ctrl
  }
  else if(useVel){veh.setVelByAcc(getVel(time));}

  // following controls are possible simultaneously!
  
  if(useAlphaT){
    //double alphaT=getAlphaT(veh.getPos());
    //cout <<"RegionControl: setControl: alphaT="<<alphaT<<endl;
    veh.setAlphaT(getAlphaT(veh.getPos()));
  }
  if(useAlphaV0){veh.setAlphaV0(getAlphaV0(veh.getPos())); }
 
}

void RegionControl::resetControl(Vehicle& veh){
  if(useSpeedlimit){veh.resetSpeedlimit();}
  else if(useAcc||useVel){veh.resetAcc();}
  else if(useAlphaT){veh.resetAlphaT();}
  else if(useAlphaV0){veh.resetAlphaV0();}
 }


bool RegionControl::singleVehControlEnded(double time){
  return vehicleFound && (time>tmax) && (time-proj.get_dt()<=tmax);
}



// following methods applied in RoadSection.cc
// martin apr08 noch altes InOut.cpp,general.cpp mit "<=" statt "<" als obere Grenze!!!
// z.B. intpextp

double RegionControl::getAcc(double time){
  return( intpextp(times, acc, n_acc, time));
} 
double RegionControl::getVel(double time){
  return( intpextp(times, vel, n_vel, time));
} 

double RegionControl::getSpeedlimit(double x){
  return( intpextp(locations, speedlimit, n_speedlimit, x));
} 

double RegionControl::getAlphaT(double x){
  // cout <<"n_alpha_T="<<n_alpha_T<<" x="<<x<<" alphaT[4]="<<alphaT[4]<<"returnval="<< intpextp(locations, alphaT, n_alpha_T, x)<<endl;
  return( intpextp(locations, alphaT, n_alpha_T, x));
} 

double RegionControl::getAlphaV0(double x){
  return( intpextp(locations, alphaV0, n_alpha_v0, x));
} 


void RegionControl::writeInfo(){
  cout <<" in RegionControl: time="<<time
       <<" controlled by info(s) in the file(s)"<<endl
       <<in_fnames<<" ..."<<endl;
}
