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
#include "VehicleDropLift.h"


VehicleDropLift::VehicleDropLift(const char projectName[], ProjectParams* proj, Heterogen* het)
{
  //Cstr
  sprintf(this->projectName, "%s", projectName);
  this->proj=proj;
  this->het=het;
  this->dt=proj->get_dt();
  this->tstart=proj->get_tstart();
  n_drop=0;
  n_lift=0;

  char in_fname[199+1];
  sprintf(in_fname, "%s.veh_drop", projectName);
  ifstream  infile (in_fname, ios::in);

  if(infile){
    infile.close(); // get_col has own file handling
    InOut inout;
    inout.get_col(in_fname, 1, n_drop, times_drop);

    if(n_drop>NTYPEMAX){
      cerr<<"VehicleDropLift Cstr: More than "<<(NTYPEMAX-1) 
	  <<" to be dropped!\n";
      exit(-1);
    }
    inout.get_col(in_fname, 2, n_drop, iveh_back);
    inout.get_col(in_fname, 3, n_drop, type_drop);
    inout.get_col(in_fname, 4, n_drop, s_back);
    inout.get_col(in_fname, 5, n_drop, v_drop);
  }
  sprintf(in_fname, "%s.veh_lift", projectName);
  ifstream infile2 (in_fname, ios::in);

  if(infile2){
    infile2.close(); // get_col has own file handling
    InOut inout;
    inout.get_col(in_fname, 1, n_lift, times_lift);

    if(n_lift>NTYPEMAX){
      cerr<<"VehicleDropLift Cstr: More than "<<(NTYPEMAX-1) 
	  <<" to be lifted!\n";
      exit(-1);
    }
    inout.get_col(in_fname, 2, n_lift, iveh_lift);
  }
  if(true){
    cout <<"Leaving VehicleDropLift Cstr: n_drop="<<n_drop
         <<" n_lift="<<n_lift<<endl;
  }

}



  // main.cc ##################################################################
  // !!! default: drop vehicle of type 0 => standard .IDM (idm1. parameters) 
  // main.cc ##################################################################

void VehicleDropLift::update(Vehicle veh[], 
			     int& imin, int& imax, double time){//update

  if(false){  cout <<" in VehicleDropLift.update: time="<<time
		  <<" imin="<<imin
                  <<" imax="<<imax  
		  <<endl;
  }

  // insert explicitely new vehicle in lead of vehicle with
  // index i_back at a distance s_back , e.g., for simulating the effects
  // of lane changes on the longitudinal model
  // no check of collisions performed!

  if (n_drop>0){
    for (int i=0; i<n_drop; i++){
      if(fabs(time-times_drop[i]) < 0.5*dt){
	if(iveh_back[i]<=imax){  // ONLY drop if there is back vehicle!
	  int idrop= static_cast<int>(max(iveh_back[i]-1, imin-1));
          cout << "VehicleDropLift.update: Dropping vehicle at position "
               <<idrop<<" leaving vehicle indices behind unchanged!!"<<endl
               <<endl;

	  if(imin==0){
            cerr<<"VehicleDropLift: imin="<<imin<<" must be >0"
	        <<" to enable ''push-lead'' "<<endl
		<<" so that back vehicle indices remain the same\n"
		<<" so that floating-car data do not jump cars!\n"
		<<" do some ''lifts'' (file .veh_lift) before!"<<endl;
	    exit(-1);
	  }

	  //---------------------
	  if(true){
            cout <<"VehicleDropLift: BEFORE drop:\n";
	    for(int i=imin; i<imax; i++){
	      cout<<" i="<<i<<" x="<<veh[i].getPos()
		  <<" v="<<veh[i].getVel()
		//<<" type="<<veh[i].getType()
		  <<" length="<<veh[i].getLength()
		  <<endl;
	    }
	  }
	  //---------------------

          imin--;
          if(false){
            for (int j=imin; j<idrop; j++){   // "push_lead"
	      cout <<"push-lead: j="<<j<<endl;
              veh[j]=veh[j+1];
	    }
	  }

	  double xback=veh[idrop+1].getPos(); // back vehicle idrop+1

	  // vehicle to be dropped has same properties as lead vehicle iback-1
	  // (cf. last index iback-1 in push-back loop),  apart from x, v
          double x=xback + veh[idrop].getLength () + s_back[i];
	  cout<<"iback="<<(idrop+1)
	      <<" xback="<<xback<<" s_back[i]="<<s_back[i]
              <<" x="<<x<<" type="<<type_drop[i]<<endl;
	  MicroModel* pmodel = het->new_pmodel(type_drop[i]);
	  //bool finiteReactTime=(type_drop[i]==2);
	  veh[idrop] = Vehicle(x, v_drop[i], 
			       proj, het->get_fluct(type_drop[i]),
                             pmodel, 
                             het->get_modelNumber(type_drop[i]), 
                             het->get_setNumber(type_drop[i]));
	  //---------------------
	  if(true){
            cout <<"VehicleDropLift: AFTER drop:\n";
	    for(int i=imin; i<imax; i++){
	      cout<<" i="<<i<<" x="<<veh[i].getPos()
		  <<" v="<<veh[i].getVel()
		  <<" length="<<veh[i].getLength()
		  <<endl;
	    }
	  }
	  //---------------------

	}
      }
    }
  }


  if (n_lift>0){
    for (int i=0; i<n_lift; i++){
      if(fabs(time-times_lift[i]) < 0.5*dt){
	int iveh=iveh_lift[i];
	if((iveh>=imin)&&(iveh<=imax)){
          cout << "VehicleDropLift.update: Lifting vehicle "
               <<iveh_lift[i]<<"!!"<<endl
	       <<" time="<<time
	       <<" fabs(time-times_lift[i])=" << fabs(time-times_lift[i])
	       << " 0.5*dt="<<0.5*dt
               <<endl;

          imin++;
          for (int i=iveh; i>=imin; i--){    //"pull_back"
            veh[i]=veh[i-1]; 
          }
	}
      }
    }
  }
}

