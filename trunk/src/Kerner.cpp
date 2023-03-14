// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

//own
#include "constants.h"
#include "general.h"
#include "InOut.h"
#include "IDM.h"  
#include "VW.h"   
#include "Kerner.h"


//arne: noch nicht drin was draufsteht !!! (November 05)
//Kerner model muss fuer mic noch angepasst werden
//Quellen von alter version siehe
// ~/trafficSim/sources/mic_jun02/src/models.cc,-h und Kernerparams.*
// Sim alte version siehe sim/ dir dieses Directories





Kerner::Kerner(const char projName[], int setNumber, double dt)
{ 
  initializeMicroModelVariables();  //arne -> init variables from MicroModel
  modelNumber=0; //arne 10-5-2005

  modelNumber=8;
  this->dt=dt;

  cout << "\nin Kerner file Cstr: initializing ..."<<endl;
  initialize(projName,setNumber,dt);
  cout <<"Kerner file Cstr: before calc_eq() ..."<<endl;
  p_basemodel->calc_eq(); 
  cout <<"Kerner file Cstr: after calc_eq() ..."<<endl;
 
  //!!! Attention: Since calculated by base model; must define 
  // MicroModel variables explicitely !!! Otherwise heineous errors !!!

  lveh=get_length();
  rhoQmax=get_rhoQmax();
  Qmax=get_Qmax();
  rhomax=(lveh>0) ? 1./lveh : 1e6;
  for (int ir=0; ir<=NRHO; ir++){
    veqtab[ir]=p_basemodel->veqtab[ir];
    cout <<"veqtab="<<veqtab[ir]<<endl;
  }
  if(false){
    cout <<"End Kerner file Cstr: Test: NRHO="<<NRHO
       <<" get_veq(0.5/lveh)=" << get_veq(0.5/lveh)
       <<" get_rhoQmax()="<<get_rhoQmax()
       <<" get_Qmax()="<<get_Qmax()
       <<" get_length()="<<get_length()
       <<endl;
  }
}



//################################################################
void Kerner::initialize(const char projName[], int setNumber, double dt){
//################################################################

  char fileName[NSTRMAX];
  char baseNameRaw[NSTRMAX];
  char baseName[NSTRMAX];

  //Kerner parameters
  sprintf(fileName,"%s.Kerner%i",projName, setNumber);
  cout << "\nKerner: initializing with filename "<<fileName<<endl;
  FILE *fp;
  fp=fopen(fileName,"r");
  filecheck(fp,fileName);
  InOut inout;

  inout.getvar(fp,&choice_basemodel);

  
  fclose(fp);

  
  // filename for base model

  if(choice_basemodel==0){sprintf(baseNameRaw,"IDM");}
  else if(choice_basemodel==1){sprintf(baseNameRaw,"VW");}
  //todo
  //  else if(choice_basemodel==2){sprintf(baseNameRaw,"HDM");}
  //  else if(choice_basemodel==3){sprintf(baseNameRaw,"OVM");}
  //  else if(choice_basemodel==7){sprintf(baseNameRaw,"FVDM");}
  else{
    cerr <<" Kerner.initialize: error: no basemodel available for "
	 <<" choice_basemodel="<<choice_basemodel<<endl;
    exit(-1);
  }

  sprintf(baseName,"%s.%s%i",projName, baseNameRaw, setNumber);


  // initialize base model including parameters

  if(choice_basemodel==0){
    p_basemodel=new IDM(baseName);
  }
  else if(choice_basemodel==1){
      p_basemodel=new VW(baseName, dt);
  }
  
  //todo
  //  else if(choice_basemodel==2){
  //    char idmName[NSTRMAX];
  //    sprintf(idmName,"%s.IDM%i",projName, setNumber);
  //    p_basemodel=new Human(idmName, baseName, dt);
  //  }
  //  else if(choice_basemodel==3){
  //    p_basemodel=new OVM(baseName);
  //  }
  //  else if(choice_basemodel==7){
  //    p_basemodel=new VelDiffModel(baseName);
  //  }
  else{
    cerr <<" Kerner.initialize: error: not all available models initialized!"
	 <<endl;
    exit(-1);
  }
  // KEIN calc_eq hier!!    
} // Kerner::initialize


//################################################################
void Kerner::get_modelparams(const char fname[]){}
//################################################################

//################################################################
void Kerner::calc_eq(){
  p_basemodel -> calc_eq();
}



//################################################################
//double Kerner::acc(int it, double v, double s, double dv, double a_lead,
double Kerner::acc(int it, int iveh, int imin, int imax,
		   double alpha_v0, double alpha_T,
		   const CyclicBuffer* const cyclicBuf)
{

  //#############################################################
  // arne 22-4-2005:
  // extended for Kerner-Switching in bottleneck traffic state
  // switch (T, a, b) parameters by factor given by Kerner-model parameters
  //#############################################################

  
  return(p_basemodel->acc(it, iveh, imin, imax,alpha_v0, alpha_T,cyclicBuf));
  
}


