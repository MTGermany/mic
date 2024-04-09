// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

#include "ProjectParams.h"

#include "RandomUtils.h"
#include "constants.h"
//#include "general.h"
#include "InOut.h"


ProjectParams::ProjectParams()
{
  //cout << "\nin ProjectParams standard Cstr"<<endl;
  //  simType=0;
  //choice_model=0;
  choice_BCup=0;
  choice_BCdown=0;
  choice_init=0;
  tstart=0;
  tend=3600;
  xmin=0;
  xmax=10000;
  dt=0.4;
  dxout_3d=400;
  dtout_3d=60;
  dnout_3d=10;
  dt_tseries=30;
  rho_init=0.02;
  random_seed=0;
  distr_v0_T=0;
  traveltime_output=0;
  trajectory_los=0;
  //comfort_output=0;
  choice_output=0;
} 
 
ProjectParams::ProjectParams(const char* projectName)
{
  char in_fname[199];
  sprintf(in_fname,"%s.proj",projectName);
  sprintf(this->projectName,"%s",projectName);

  cout <<"\n\nProjectParams file Cstr:"
       <<" Reading simulation parameters from file "
       <<in_fname<<endl;

  FILE* fp;
  fp=fopen(in_fname,"r");
  if(fp==NULL){
    cerr << "Error: no file "<< in_fname <<" found"<<endl
	 <<" Calling sequence: mic <projectName> or"<<endl
	 <<" mic <projectName> <integrationMethod>"<<endl
	 <<" (0=standard,1=Euler,2=Ballistic,3=Trapezoid,4=RK4)"
	 <<endl;
    exit(-1);
  }

  InOut inout;
	
	
	//Oct 06:
	int number_params = inout.getNumberOfDataLines(in_fname);
	if(number_params != 16){
		cerr<<"\nError!!!\n"<<endl;
		cerr<<"mic input for \""<<in_fname<<"\" requires 16 parameters (upgrade version Oct06) but provided are only "<<number_params<<endl;
		cerr<<"note that mic older than Oct06 requires 13 parameters in proj input"<<endl;
		cerr<<"\nplease run : micUpgrade "<<projectName<<endl<<endl;
		exit(-1);
	}

  // model and its parameters 

  //inout.getvar(fp,&choice_model);

  // boundary and initial conditins, infrastructure 

  inout.getvar(fp,&choice_BCup);
  inout.getvar(fp,&choice_BCdown);
  inout.getvar(fp,&choice_init);
  rho_init=0.02;
  ampl_init=0.001;
  xrelCenter=0.5;

  // Range of integration and numerical parameters 

  //inout.getvar(fp,&choice_method);
	//choice_method=0; //only Euler (oct06)
  tstart=0;
  inout.getvar(fp,&tend);
  inout.getvar(fp,&xmax);
  inout.getvar(fp,&dt);

  // control of the output (what; in which form) 
  inout.getvar(fp,&dtout_FCD);
  inout.getvar(fp,&dt_tseries); 
  inout.getvar(fp,&dxout_3d); 
  inout.getvar(fp,&dtout_3d);
  inout.getvar(fp,&dnout_3d);

  inout.getvar(fp,&choice_output);
  inout.getvar(fp,&traveltime_output);
  //inout.getvar(fp,&comfort_output);
  inout.getvar(fp,&trajectory_los);

  //arne: new feature to set random seed if parameter is defined
  //if parameter lacks the standard (fixed) seed is used (compatible with old projects)

  random_seed=0;
  inout.getvar(fp,&random_seed);   //arne
  //inout.getvar(fp,&distr_T_react);   //arne, febr. 06
  distr_v0_T=0;
  inout.getvar(fp,&distr_v0_T);  //arne, febr. 06

  fclose(fp);

  // xmin!=0 not yet used; set default
  xmin=0;

  // increase xmax by a certain amount to provide smooth fields
  // in case of periodic BC

  // (reduce also vehicle number by one to avoid crashes for periodic BC.
  // e.g. xmax=10000, rho=0.01 => 1st veh at 0, last at 10000,
  // in RoadSection.initialize(...))

	/*
  if(choice_method!=0)
    {
      cerr<<"ProjectParams: Error: At present, only Euler-CAH update rule"
	  <<" choice_method=0 implemented!"<<endl;
      exit(-1);
    }
*/
  
  if(choice_BCup==3){
    if(choice_BCdown==3){
      //int nveh=static_cast<int>(rho_init*(xmax-xmin));
      //xmax = xmin+(nveh+1)/rho_init; //!! <martin jun08> does not help
                                      // small bug at IC at x=0/xmax
    }
    else{
      cerr<<"ProjectParams: Error: choice_BCup=3 does only make sense"
	  <<" if choice_BCdown=3"<<endl;
      exit(-1);
    }
  }
  
  if(random_seed!=0)
    {
      setRandomSeed(); //from general
    }
} 

//ProjectParams::~ProjectParams(){}
