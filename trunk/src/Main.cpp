
/*****************************************************************************
Programm mic: Simulation of microscopic models

Models: 
     intelligent-driver model (IDM), vw, human


Form of the equations:
     d_t x[i] = v[i],
     d_t v[i] = F(dx,v[i],dv,possibly a[i-1]),

     with
     dx = x[i-1]-x[i], dv = v[i-1]-v[i],

     i = imin (first vehicle), ..., imax (last vehicle);
     v[i]>=0 so that x[i-1] > x[i] 

Integration scheme:
     Euler

Initial conditions: 
     Positions x[i] and velocities v[i] of all vehicles i=imin..imax

Boundary conditions:
     Free road (choice_BC==0), or 
     blocked road in lead (choice_BC==1)

With fluctuations: 
     Edit models.cc; Set fluct_stdev >0  (typically 2-7)

> ***************************************************************
> IDM  and program (C) Martin Treiber and Arne Kesting
> Last revised October 2006
> *******************************************************************/

// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

//alternatively there is <cstdio> which declares
//everything in namespace std
//but the explicit "using namespace std;" puts
//everything in global namespace

// c++ 
#include <iostream>
using namespace std;

// own
//#include "MicroModel.h"
//#include "IDM.h"   // only for test
//#include "NewModel.h"  // only for test
//#include "Gipps.h"  // only for test
//#include "HDM.h"  // only for test
//#include "OVM.h"  // only for test
#include "ProjectParams.h"
#include "RoadSection.h"
#include "Heterogen.h"
#include "Fluct.h"


//#####################################################
//#####################################################
int main(int argc, char* argv[]) {

  //#####################################################
  // Parse cmd line and determine project name
  //#####################################################

  char   projectName[199];
  if (argc==1){
    cout <<"\nPlease enter the project name\n";
    cin>> projectName;
  }
  else {
    sprintf(projectName,"%s",argv[1]);
  }

  int choice_integr=0;
  if(argc>=3){
    choice_integr=atoi(argv[2]);
  }


 //#####################################################
 // input
 //#####################################################
  
 ProjectParams* proj = new ProjectParams(projectName);
 cout <<"Main-1"<<endl;
 
 
 //#####################################################
 // Construct road section as main simulation element
 //#####################################################
 
 //hier dynamisch --> damit kein segmentation fault wg. zu geringem stack!
 
 RoadSection* roadsec = new RoadSection(projectName, proj);
 cout <<"Main-3"<<endl;
 
 
 //#####################################################
 // Construct and/or test other elements
 //#####################################################
 
 /*

 bool withTest=false;

 if(withTest){
   //InOut inout;   //access i/o routines
   
   // Heterogeneous vehicle pop. and its models
   Heterogen* testhet= new Heterogen(projectName, proj);
   cout <<"Test Heterogen:"<<endl;
   for (int i=0; i<=10; i++){
     double frac=0.1*i;
     cout<<" testhet->get_type("<<frac<<")="<<testhet->get_type(frac)<<endl;
   }
   
   cout <<"main: Test models produced by Heterogen:"<<endl;
   
   MicroModel* test_p_models[2*NTYPEMAX];
   int ntype=testhet->get_ntypes();
   for (int itype=0; itype<ntype; itype++){
     test_p_models[itype] = testhet->new_pmodel(itype);
   }
   // test if each element of array is really pointer to own instance
   // -> lengths of dereferenced object 0 and ntype must be different
   test_p_models[ntype+1] = testhet->new_pmodel(0);
   test_p_models[ntype+1]->set_length(11.1);
   
   
   //ABFANG 
   delete testhet;  //TEST
   cout<<"end test Heterogen\n";
   //exit(0);
   
   //boundaries
   Boundary* testBCup = new Boundary(projectName, true, proj, fluct);
   cout<<"testBCup.getFlow(120)="<<testBCup->getFlow(120)<<endl;
   delete testBCup;
 
 } //of test
 */

 //#####################################################
 // Prepare the simulation incl. initial output
 //#####################################################
 
 int nt =  static_cast<int>((proj->get_tend()-proj->get_tstart())/proj->get_dt())+1;

 // it=0 ... nt
 //cout <<"main: proj->get_choice_init="<<proj->get_choice_init()<<endl;
 
 roadsec->initialize(proj->get_choice_init());
 roadsec->writeAll(0);
 
 //!!! Benoetigt .IDM*, .HDM* Files!!
 
 // Dies (allgemeiner) in Heterogen::writeFunddiagrams implementieren
 if (false){
   char   in_fname[199];
   char   out_fname[199];
   sprintf(in_fname,"%s.IDM1",projectName);
   sprintf(out_fname,"%s.IDM1_funddia",projectName);
   IDM idm1=IDM(in_fname);
   idm1.writeFunddia(out_fname);
 }
 

 //#####################################################
 // Simulate: Main simulation loop!
 //#####################################################

 cout<<"Main: main simulation loop starts ....\n";
 if(true){
   setRandomSeed();
   cout <<"Setting random seed (disable in Main.cpp for testing)"<<endl;
 }

 for (int it=1; it<=nt; it++){
   //cout <<"before writeAll: it="<<it<<" nt="<<nt<<endl;

   // MT Apr 13: Select integration method by optional second cmdline parameter
   
   if(choice_integr==0){roadsec->update(it);}
   else if(choice_integr==1){roadsec->updateEuler(it);}
   else if(choice_integr==2){roadsec->updateBallistic(it);}
   else if(choice_integr==3){roadsec->updateTrapezoid(it);}
   else{roadsec->updateRK4(it);}

   //roadsec->updateBallistic(it);
   // roadsec->updateTrapezoid(it);
   // roadsec->updateRK4(it);

   roadsec->writeAll(it); //also update of detectors !
 }

 //cout<<"exit after nt="<<nt<<" steps\n";
 //#####################################################
 // end exit 
 //#####################################################

 delete roadsec; 
 delete proj;

 cout<<"mic main call finished ... \n\n";
 return(0);
}

// #########################################################







