
// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
#include <fstream>
using namespace std;

#include "InOut.h"
#include "general.h"
#include "constants.h"
#include "Ramp.h"

//!!! RAMP_VEL_REDUCEFACTOR = relat. speed of inserted vehs
// => RoadSection.cpp

Ramp::Ramp(){;}

//#######################################################################
Ramp::Ramp(char* projectName, ProjectParams* proj, int number, double xCenter, double l_ramp)
//#######################################################################
{
  this->dt       = proj->get_dt();
  this->xmax     = proj->get_xmax();
  this->xCenter  = xCenter;
  this->length_m = l_ramp;

   // reading ramp flow

  char in_fname[199];
  sprintf(in_fname,"%s.rmp%i",projectName, number);
  ifstream  infile (in_fname, ios::in);
  if( !infile){ 
    cerr<<" Ramp Cstr: No file "<<in_fname<<"!\n";
    exit(-1);
  }
  else{
    InOut inout;
    inout.get_array(in_fname, n_times, times, Qrmp);
    queueLength_veh=0.;
    if(n_times>NBC){
      cerr<<"Ramp Cstr: Error: to many (>NBC) ramp data!\n";
      exit(-1);
    }
  }
}   

//#######################################################################
void Ramp::update(double time, double x[], double s[], int imin, int imax)
//#######################################################################
{
  queueLength_veh += getFlow(time)*dt;
  mergeOK=false;
  divergeOK=false;

  //tryMerge, tryDiverge calculates position and index for the lane change
  // actual veh. length not dyn. relevant for this simulation -> lvehTest

  //cout<<"Ramp.update: getFlow(time)="<<getFlow(time)<<endl;
  //cout<<"Ramp.update: queueLength_veh="<<queueLength_veh<<endl;
  if( queueLength_veh >= 1.0 ){
    double lvehTest=5;  //!!!!!!!!!!!!
    tryMerge(x, s, lvehTest, imin,imax);

    //arne: achtung, schalter in constants.h ist kritisch fuer Simulation!!!!
    if(mergeOK||DROP_NONENTERING_RAMP_VEHICLES){queueLength_veh -= 1.0;}
  }
  if( queueLength_veh <= -1.0 ){tryDiverge(x,imin,imax);
    if(divergeOK){queueLength_veh += 1.0;}
  }

}


double Ramp::getFlow(double time){
  return( intpextp(times, Qrmp, n_times, time)/3600.);
} 


//########################################################
void Ramp::tryMerge(double x[], double s[], double lvehMerge, int imin, int imax)
//#######################################################
{
  bool check=false;

  // Determine first index fi and last index li of vehicles parallel ramp

  int fi=imin;
  int li=imax;

  double xUp   = xCenter-0.5*length_m;
  double xDown = xCenter+0.5*length_m;
  // changePos=xCenter;

  while( (x[fi]>xDown)&&(fi<=imax)){fi++;}
  bool noVehDown=(fi==imin);

  while( (x[li]<xUp)&&(li>=imin)){li--;}
  bool noVehUp=(li==imax);

  bool noVehParallelRamp = ( (x[fi]<xUp) || (x[li]>xDown));

  // Determine maximum distance within the ramp section
  // !! merging behaviour determined by leadVehLength (hack!),
  // MINSPACE_MERGE_M => constants.h
  double maxdist=0;
  double leadVehLength=6; //!!! ??????????????????????????????????

  if(noVehDown){s[fi]=xmax-x[fi];}  // otherwise no  s[fi=imin] defined

  for (int i=fi; i<=min(li+1, imax); i++){
    if( min(s[i], 2*(xDown-x[i]))>maxdist){ // min() relev. for first vehicle
      maxdist = min(s[i], 2*(xDown-x[i]));
      changePos=x[i]+0.5*(maxdist-lvehMerge)+leadVehLength;
      leadVehIndex=i-1; // can be -1 //!!!
      if(check){
        cout<<"tryMerge: i="<<i
	    <<" fi="<<fi<<" li="<<li<<" maxdist="<<maxdist
            <<" x[i]="<<x[i]<<" s[i]="<<s[i]
	    <<" changePos="<<changePos
	    <<endl;
      }
    }
  }

  if(noVehUp){
    if(2*(x[li]-leadVehLength-xUp) > maxdist){
      maxdist=2*(x[li]-leadVehLength-xUp);
      changePos=xUp;
      leadVehIndex=imax;
    }
  }

  if(noVehParallelRamp){ // i.e., li<fi
    maxdist=length_m;
    changePos=xCenter;
    leadVehIndex=fi-1; // !!! dort war BUG
  }

  
  //arne 18-5-2005
  //achtung, hier wirken sich rundungsfehler aus: 
  //deshalb round (man muss ziemlich stark runden wg. fluct)
  //das ist ziemlich sensitiv!!! 
  //mergeOK = ( round(maxdist) > (lvehMerge+2.0*MINSPACE_MERGE_M) );
  
  mergeOK = ( maxdist > (lvehMerge+2.0*MINSPACE_MERGE_M) );


  if(!mergeOK){
    cout <<"Ramp.tryMerge: merge not possible!"
	 <<" queueLength_veh="<<queueLength_veh
	 <<" maxdist= "<<maxdist
	 <<endl;
  }
  if(check){
    cout <<" noVehUp="<<noVehUp
         <<" noVehDown="<<noVehDown
         <<" noVehParallel="<<noVehParallelRamp
         <<" mergeOK="<<mergeOK
         <<endl;
  }
  if(check){
    cout <<"Ramp.tryMerge: end: "<<endl
	 <<" mergeOK="<<mergeOK
         <<" maxdist="<<maxdist
         <<" noVehUp="<<noVehUp
         <<" noVehDown="<<noVehDown
         <<" noVehParallel="<<noVehParallelRamp
	 <<" changePos="<<changePos
	 <<" leadVehIndex="<<leadVehIndex
	 <<endl;
  }


} // end tryMerge




//#######################################################################
void Ramp::tryDiverge(double x[], int imin, int imax){
//#######################################################################
  // calculate 
}

/*
  bool First=FALSE;
  int fi,li=0;
  // First and last index on ramp-section
  double d_rmp=0.5*dx_rmps[irmp];

  for (int i=imin+1;i<=imax;i++){
    if (!First){
      if (x[i]<(x_rmps[irmp]+d_rmp)){
	First=TRUE;
	fi=i;
      }
    }
    if (First){
      if (x[i]>(x_rmps[irmp]-d_rmp)) li =i;
    }
  }
 
  // Removal of vehicle possible
  if (First){
    queue[irmp]=queue[irmp]+1.0; 
//!!! write_rmp_stat(projectName, queue, x[li], irmp,it);
    pull_back (li);    
  }
}
*/
