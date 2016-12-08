//c
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>


// c++ 
#include <iostream>
//#include <fstream>
using namespace std;



//own
#include "constants.h"
#include "general.h"
#include "Detector.h"

Detector::Detector(){;}

Detector::Detector(int x_m, int avgInterval_sec,ProjectParams* proj, 
		   const char* projectName)
{
  sprintf(this->projectName, "%s", projectName);
  sprintf(this->aggregate_fname, "%s.x%i_det", projectName, x_m);
  sprintf(this->single_fname, "%s.x%i_single",   projectName, x_m);

  this->avgInterval = avgInterval_sec;
  this->pos=x_m;
  this->dt=proj->get_dt();
  this->tStart=proj->get_tstart();

  sum_veh=total_sum_veh=0;
  sum_v=0;
  sum_invv=0;
  sum_invq=0;
  v_single=0;
  time_lastPassage=tStart;
  iveh_lastPassage=0;
  minTimeHeadway = 0.2; // minimum time headway between two vehicles (s)
	
	FH_single=0;
	FH_det=0;
}


Detector::~Detector()
{
	//cout<<"Deconstructor of Detector ... close file handles...\n";
	if(FH_single!=0) fclose(FH_single);
	if(FH_det!=0) fclose(FH_det);
  //cout<<"Deconstructor of Detector finished ...\n";
}


void Detector::update(int it, Vehicle veh[], int imin, int imax){

  if(it==0){writeSingleVehData(0, 0, 0, 2, 0, 0, 0, 1, 100);} 

  double time=tStart+it*dt;
  int n_it_interval = static_cast<int>(avgInterval/dt+SMALL_VAL); 
  if(false){
    cout <<"Detector::update: avgInterval="<<avgInterval
	 <<" dt="<<dt<<" n_it_interval="<<n_it_interval<<endl;
  }

  //if(time-time_lastPassage>=minTimeHeadway){
  if(true){

    // find vehicle i that is nearest downstream of the detector

    int i=iveh_lastPassage; 
    while ((i<=imax)&&(veh[i].getPos()>pos)){  // <=imax, not <imax!
      i++; 
    } 
    if(i>imin){i--;}


    if( (veh[i].getOldPos() - pos -SMALL_VAL <=0)
      &&(veh[i].getPos() - pos >0)){
      if(false){cout <<"Detector::update: new vehicle crossed detector at x="
		    <<pos<<"!" <<endl
		    <<"it="<<it<<" iveh="<<i
	  //<<" imin="<<imin<<" imax="<<imax
	  // <<" x(imax)="<<veh[imax].getPos()
                    <<" posDet="<<pos
		    <<" posVeh="<<veh[i].getPos()
		    <<" oldposVeh="<<veh[i].getOldPos()
		    <<" vel="<<veh[i].getVel()
		    <<" Hint: missed veh=rmp veh"
		    <<endl;
      }


      // v_single and time_lastPassage = data of last veh before updating

      double Tbrutto=time-time_lastPassage;

      //!!! mt apr05
      //double v_prev=(v_single>0) ? v_single : veh[i].getVel(); //old
      double v_prev=(i>imin) ? veh[i-1].getVel() : v_single;  //new
      double l_prev=(i>imin) ? veh[i-1].getLength() : 0;
      double sReal=(i>imin) ? veh[i-1].getPos()-veh[i].getPos()-l_prev
	: 100;  //new
      //!!!

      time_lastPassage = time;
      iveh_lastPassage = i;
      v_single=veh[i].getVel();
 

      sum_veh  +=1;
      total_sum_veh  +=1;
      sum_v    +=v_single;
      sum_invv +=(v_single>0) ? 1./v_single : 0;
      sum_invq +=(Tbrutto>0) ? 1./Tbrutto : 0;
      writeSingleVehData(it, v_single, veh[i].getLength(), 
			 Tbrutto, v_prev, l_prev, 
			 veh[i].getModelNumber(), 
			 veh[i].getSetNumber(), sReal); 
    }
  }

  // determine if one averaging period is up; if so, 
  // reset micro data base write aggregate output

  if( it%n_it_interval==0){
    writeAggregatedData(it);
    sum_veh =0;
    sum_v   =0;
    sum_invv=0;
    sum_invq=0;
    }
}

void Detector::writeAggregatedData(int it){
	//det output

  bool vehDetected = (sum_veh>=1);
  double time=tStart+it*dt;
  //cout <<" Detector::writeAggregatedData: it="<<it<<" dt="<<dt<<" time="<<time<<endl;
	
  if(FH_det==0){
		FH_det = fopen(aggregate_fname,"w");
		filecheck (FH_det,aggregate_fname);
    fprintf(FH_det, "%s", "# t(min) \t n_veh \t <v>(km/h) \t <1/v>(h/km)\t<1/Tbrutto>(1/s)\n");
  }


  if(vehDetected){
    fprintf(FH_det, "%.6f\t  %i\t %.6f\t  %.6f\t%.6f\n",
	    time/60, sum_veh, 3.6*sum_v/sum_veh,
	    sum_invv/(3.6*sum_veh),  sum_invq/sum_veh);
  }
  else{
    fprintf(FH_det, "%.6f\t  %i\t %.6f\t  %.6f\t%.6f\n",
	    time/60, 0, 0., 0.,0.);
  }
  fflush(FH_det);
}

 
void Detector::writeSingleVehData(int it, double v_single,double l_single,
				  double Tbrutto, double v_prev,double l_prev, int type, int setNumber,
				  double sReal){


  double time=tStart+it*dt;

	if(FH_single==0){
		FH_single = fopen(single_fname,"w");
		filecheck (FH_single, single_fname);
    fprintf(FH_single, "%s",
		  "#time(s)\tv(km/h)\t\tl_veh\t\ttype\tTbrutto\t\tvPrev(km/h)\tlPrev\t vehicle count\tset number\ts\n");
  }
  else{
    fprintf(FH_single,"%f\t%f\t%f\t%i\t%f\t%f\t%f\t%i\t%i\t%f\n",
    time, (3.6*v_single), l_single, type,Tbrutto, (3.6*v_prev), l_prev, total_sum_veh, setNumber, sReal);
  }
  fflush(FH_single);
}



