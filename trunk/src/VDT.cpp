#ifndef VDT_CC
#define VDT_CC

// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

//own
#include "VDT.h"
#include "constants.h"
#include "general.h"
#include "InOut.h"
#include "IDM.h"
#include "HDM.h"
#include "OVM.h"
#include "FVDM.h"
#include "CyclicBuffer.h"


VDT::VDT(const char projName[], int setNumber, double dt)
{  
  initializeMicroModelVariables();  //arne -> init variables from MicroModel
  modelNumber=6; 
  speedlimit=100000; // martin apr08: initially no speed limit 

  this->dt=dt; // some base models need dt

  cout << "\nin VDT file Cstr: initializing ..."<<endl;
  initialize(projName,setNumber,dt);
  cout <<"VDT file Cstr: before calc_eq() ..."<<endl;
  p_basemodel->calc_eq(); 
  cout <<"VDT file Cstr: after calc_eq() ..."<<endl;

  //!!! Attention: Since calculated by base model; must define 
  // MicroModel variables explicitely !!! Otherwise heineous errors !!!

  lveh=get_length();
  rhoQmax=get_rhoQmax();
  Qmax=get_Qmax();
  rhomax=(lveh>0) ? 1./lveh : 1e6;
  modelNumber=6;
  for (int ir=0; ir<=NRHO; ir++){
    veqtab[ir]=p_basemodel->veqtab[ir];
    cout <<"veqtab="<<veqtab[ir]<<endl;
  }
 
  if(false){
    cout <<"End VDT file Cstr: Test: NRHO="<<NRHO
       <<" get_veq(0.02)=" << get_veq(0.02)
       <<" get_veq(0.08)="<<get_veq(0.08)
       <<" get_rhoQmax()="<<get_rhoQmax()
       <<" get_Qmax()="<<get_Qmax()
       <<" get_length()="<<get_length()
       <<endl;
    //exit(0);
  }
}




//################################################################
void VDT::initialize(const char projName[], int setNumber, double dt){
//################################################################


  char vdtName[NSTRMAX];
  char baseNameRaw[NSTRMAX];
  char baseName[NSTRMAX];

  //VDT parameters

  sprintf(vdtName,"%s.VDT%i",projName, setNumber);
  FILE *fp;
  fp=fopen(vdtName,"r");
  filecheck(fp,vdtName);
  InOut inout;


  inout.getvar(fp,&choice_basemodel);
  inout.getvar(fp,&nveh);
  inout.getvar(fp,&alphaT_max);
  inout.getvar(fp,&gamma);
  //Qacc in .fluct file!
  alphaT_VDT=1.; // start with perfectly smooth traffic ...


  // filename for base model

  if(choice_basemodel==0)     {sprintf(baseNameRaw,"IDM");}
  else if(choice_basemodel==2){sprintf(baseNameRaw,"HDM");}
  else if(choice_basemodel==3){sprintf(baseNameRaw,"OVM");}
  else if(choice_basemodel==7){sprintf(baseNameRaw,"FVDM");}
  else{
    cerr <<" VDT.initialize: error: no basemodel available for "
	 <<" choice_basemodel="<<choice_basemodel<<endl;
    exit(-1);
  }

  sprintf(baseName,"%s.%s%i",projName, baseNameRaw, setNumber);


  // initialize base model including parameters

  if(choice_basemodel==0){
    p_basemodel=new IDM(baseName);
  }
  else if(choice_basemodel==2){
    char idmName[NSTRMAX];
    sprintf(idmName,"%s.IDM%i",projName, setNumber);
    p_basemodel=new HDM(idmName, baseName, dt);
  }
  else if(choice_basemodel==3){
    p_basemodel=new OVM(baseName);
  }
  else if(choice_basemodel==7){
    p_basemodel=new FVDM(baseName);
  }
  else{
    cerr <<" VDT.initialize: error: not all available models initialized!"
	 <<endl;
    exit(-1);
  }

  // KEIN calc_eq hier!!    

} // VDT::initialize



void VDT::get_modelparams(const char fname[]){}

double VDT::acc(int it, int iveh, int imin, int imax,
		double alpha_v0, double alpha_T,
		const CyclicBuffer* const cyclicBuf)
{
    //#############################################################
    // VDT contribution: Factor alpha_T_VDT due to perceived smoothness
    // alpha_T_VDT = max(alpha_T_min, varcoeff_v); Tloc *= alpha_T_VDT
    //#############################################################

    int nveh_actual=(nveh<(iveh-imin+1)) ? nveh : (iveh-imin+1);

    if(nveh_actual>=1)
      {
	double sumv=0;
	double sumsqr=0;

	//int n=nveh_actual;

	for (int j=0; j<nveh_actual; j++)
	  {
	    sumv += cyclicBuf->get_v(iveh-j,it,T_react);
	  }
	
	double avgv=sumv/nveh_actual;

	for (int j=0; j<nveh_actual; j++)
	  {
	    sumsqr += SQR(cyclicBuf->get_v(iveh-j,it,T_react) - avgv);
	  }
      
      double sigma2v    = sumsqr/(nveh_actual-1); //mar05
      double varcoeff_v = sqrt(sigma2v)/max(avgv,1e-10);

      //####### main VDT formula ###################

      alphaT_VDT = min(alphaT_max, 1+gamma*varcoeff_v);

      //#################################################

      //cout <<" varcoeff_v="<<varcoeff_v;
      //cout <<" alpha_T_VDT="<<alpha_T_VDT<<endl;
    
      bool check=false;
      //if(check&&(iveh==800)&&(it%10==0)){
      if(check&&(it%100==0))
	{
	  cout <<"VDT.acc: t="<<(it*dt)
	       <<" iveh="<<iveh<<" imin="<<imin<<" imax="<<imax 
	       <<" x="<<cyclicBuf->get_x(iveh,it,T_react)
	       <<" v[iveh]="<<cyclicBuf->get_v(iveh,it,T_react)
	       <<" v[iveh-1]="<<cyclicBuf->get_v(iveh-1,it,T_react)
	       <<" avgv="<<avgv<<" sumsqr="<<sumsqr<<" sigma2v="<<sigma2v<<endl
	       <<" nveh_actual="<<nveh_actual<<"  varcoeff_v="<<varcoeff_v
	       <<" alphaT_VDT="<<alphaT_VDT<<endl;
	}
      }
    else
      {
	alphaT_VDT =1;
      }

    return p_basemodel->acc(it, iveh, imin, imax,alpha_v0, alpha_T*alphaT_VDT, cyclicBuf);
}


//################################################################
void VDT::calc_eq()
{
  p_basemodel -> calc_eq();
}

#endif // VDT_CC

