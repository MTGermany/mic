
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
  this->time=time;
  if(inTimeInterval(time)){

      //################################
      // ctrl single vehicles
      //################################

    // new instance of ExternalControl created one in RoadSection (see if(regionCtrlExists){..})
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
	    cout <<"RegionControl.update: event-oriented single-vehicle control:"
	    <<" found single vehicle index "<<ivehSingle<<" at x="
	    <<veh[ivehSingle].getPos()<<endl;
	  }
	}
	//exit(0);
      }
    }

    //################################
    // real region control
    //################################

    bool relevantForSomeVeh // not applicable for useSingle!
      =(!isUpstream(veh[imin+1].getPos())) //!! min+1, not min
      && (!isDownstream(veh[imax].getPos())); //!! imax-1 vs. imax

    if(relevantForSomeVeh&&(!useSingle)){
      //if(ctrlByAcc()){writeInfo(); }
      
      bool upstream=false;// start with imin=most downstream	  
      bool downstream=true;
      int iveh=imin+1; //!! min+1, not min

        //bool test=(regionIndex==2);
      bool test=false;
      while(  (iveh<=imax)&&downstream){//!! imax-1 vs. imax
	if(test){
	  cout <<"t="<<time<<" iveh="<<iveh
	       <<" xmin="<<xmin<<" xmax="<<xmax
	       <<" x="<<veh[iveh].getPos()<<endl;
	}
	if(inSpaceInterval(veh[iveh].getPos())){

	  if(test){
	    cout <<" inSpaceInterval!"<<endl;
	    cout <<"t="<<time<<" iveh="<<iveh
	       <<" xmin="<<xmin<<" xmax="<<xmax
	       <<" x="<<veh[iveh].getPos()<<endl;
	  }
	  
	  downstream=false;
	  iveh=max(iveh-1,0);
	  resetControl(veh[iveh]); // now at downstream boundary
	  if(test){
	        cout <<"downstream boundary reached for vehicle "<<iveh+1
		<<" reset regionControl for vehicle "<<iveh
		     <<" at x= "<< veh[iveh].getPos()<<endl;
	  }
	}
	iveh++;

      }

      while ( (iveh<=imax)&&(!upstream)){ //!! imax-1 vs. imax
	setControl(veh[iveh]);
	if(!isInRegion(veh[iveh].getPos(),time)){
	  upstream=true;
	  if(test){
	    cout <<" upstream index reached for vehicle "<<iveh
		 <<" at x="<<veh[iveh].getPos()<<endl;
	    //writeInfo();
	  }

	}
	iveh++;
      }

    }//relevantForSomeVeh&&( !singleVeh)
  }//inTimeInterval(time)

  // check if control time interval just endet => if so, reset everything
  if((!useSingle) && (time>tmax) &&(time<tmax+1.5*proj.get_dt())){
    cout <<"RegionControl, reginIndex="<<regionIndex
	 <<" Control has just ended!"<<endl;
    for (int iveh=imin; iveh<=imax; iveh++){ //!! min+1, not min, imax-1 vs. imax
      resetControl(veh[iveh]);
      //cout <<"t="<<time<<": resetControl of vehicle at "<<veh[iveh].getPos()<<endl;
    }
  }

  // remove control for previous-to-last(otherwise veh with v=0 stuck forever)
  resetControl(veh[imin+1]);
  
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
