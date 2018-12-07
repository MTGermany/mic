
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
#include "Heterogen.h"
#include "CyclicBuffer.h"



Heterogen::Heterogen(){;}


Heterogen::Heterogen(const char projectName[], ProjectParams* const proj)
{
  cout << "\nin Heterogen file Cstr: projectName= "<<projectName<<endl;
  sprintf(this->projectName,"%s",projectName);
  this->proj=proj;
  getHeterogenparams();
  if(ntypes>NTYPEMAX){
    cerr<<"Heterogen.cc: Error: read more than NTYPEMAX="<<NTYPEMAX
	<<" vehicle types!"<<endl;
    exit(-1);
  }

  // generate reference model types and fluct instances for copying
  //

  // here default copy constructor is used ! Don't write a copy
  // constructor if shallow copies are ok!
  // If the object has no
  // pointers to dynamically allocated memory, a shallow copy is
  // probably sufficient. Therefore the default copy constructor,
  // default assignment operator, and default destructor are ok and
  // you don't need to write your own.

  for (int itype=0; itype<ntypes; itype++)
    { 

      fluctRef[itype]=Fluct(projectName,itype,proj->get_dt()); // mt mar07

      
      MicroModel* p_model=0; //nur fuer fundDia am ende 
      char param_file[199];
      char ext[200];
      getExtension(itype,ext);
      sprintf(param_file,"%s.%s",projectName, ext);
 

      cout <<"param_file="<<param_file<<endl;

      int modelNumber=get_modelNumber(itype);

      if(modelNumber==0)
	{
	  // !! direct constructor w/o local "idm" leads to segm. fault!!
	  idmRef[itype]= IDM(param_file); //OK
	  IDM* pidm = new IDM(param_file); // new erzeugt Pointer
	  //p_modelRef[itype]= pidm;
	  p_model = pidm;
	}
      else if(modelNumber==1)
	{
	  vwRef[itype]= VW(param_file,proj->get_dt()); //OK
	  VW* pvw = new VW(param_file,proj->get_dt()); 
	  //p_modelRef[itype]= pvw;
	  p_model=pvw;
	}
      else if(modelNumber==2)
	{ 
	  // HDM needs 2 input files IDM%i und HDM%i
	  //arne: not as generic as VDT, but more difficult to apply to other models
	  char param_fileIDM[199];
	  sprintf(param_fileIDM,"%s.IDM%i", projectName, setNumber[itype]);
	  hdmRef[itype]= HDM(param_fileIDM,param_file,proj->get_dt());
	  HDM *phdm = new HDM(param_fileIDM,param_file,proj->get_dt()); 
	  //p_modelRef[itype]= phdm;
	  p_model = phdm;
	}
      else if(modelNumber==3)
	{
	  ovmRef[itype]= OVM(param_file);
	  OVM* povm = new OVM(param_file); 
	  //p_modelRef[itype]= povm;
	  p_model = povm;
	}
      else if(modelNumber==4)
	{
	  fpeRef[itype]= FPE(param_file, proj);
	  FPE* pfpe = new FPE(param_file, proj); 
	  //p_modelRef[itype]= pfpe;
	  p_model = pfpe;
	}
      else if(modelNumber==5)
	{
	  humanSchreckRef[itype]= HumanSchreck(param_file,proj->get_dt()); //OK
	  HumanSchreck* phumanSchreck = new HumanSchreck(param_file,proj->get_dt()); 
	  //p_modelRef[itype]= phumanSchreck;
	  p_model = phumanSchreck;
	}
      else if(modelNumber==6)
	{ // VDT needs 2 input files IDM%i und HDM%i
	  vdtRef[itype]= VDT(projectName,setNumber[itype],proj->get_dt()); //OK
	  VDT* pvdt = new VDT(projectName,setNumber[itype],proj->get_dt()); 
	  //p_modelRef[itype]= pvdt;
	  p_model = pvdt;
	}
      else if(modelNumber==7)
	{ // FVDM (old name:VelDiffModel)
	  vdiffRef[itype]= FVDM(param_file); //OK
	  FVDM* pvdiff = new FVDM(param_file); 
	  //p_modelRef[itype]= pvdiff;
	  p_model=pvdiff;
	}
      else if(modelNumber==8)
	{
	  //arne 9-11-2005: Kerner model
	  kernerRef[itype]= Kerner(projectName,setNumber[itype],proj->get_dt());
	  Kerner* pKerner = new Kerner(projectName,setNumber[itype],proj->get_dt()); 
	  //p_modelRef[itype]= pKerner;
	  p_model = pKerner;
	}
      else if(modelNumber==9)
	{
	  //martin 21-7-2005: CDDA model (Helbing,Rui,Treiber, hopefully PRE)
	  cddaRef[itype]= CDDA(param_file,proj->get_dt());
	  CDDA* pCDDA = new CDDA(param_file,proj->get_dt()); 
	  p_model = pCDDA;
	}
       else if(modelNumber==10)
	{
	  //martin 21-12-2006: Gipps-Model
	  gippsRef[itype]= Gipps(param_file,proj->get_dt());
	  Gipps* pGipps = new Gipps(param_file,proj->get_dt()); 
	  p_model = pGipps;
	}
       else if(modelNumber==11)
	{
	  //martin 27-05-2007: Kerner's CA Modell (Kerner-Buch S. 409)
	  kernercaRef[itype]= KernerCA(param_file,proj->get_dt());
	  KernerCA* pKernerCA = new KernerCA(param_file,proj->get_dt()); 
	  p_model = pKernerCA;
	}
       else if(modelNumber==12)
	{
	  //martin 30-01-2008: Martin: PT Modell (eigenes neues Modell mit Samer!)
	  ptmodelRef[itype]= PTmodel(param_file,proj->get_dt());
	  PTmodel* pPTmodel = new PTmodel(param_file,proj->get_dt()); 
	  p_model = pPTmodel;
	}

       else if(modelNumber==13)
	{
	  //martin 14-04-2011: ASGM model
	  asgmRef[itype]= ASGM(param_file,proj->get_dt());
	  ASGM* pASGM = new ASGM(param_file,proj->get_dt()); 
	  p_model = pASGM;
	}
      
      else if(modelNumber==14)
	{
	  //martin 10-05-2012: ADAS model
	  adasRef[itype]= ADAS(param_file,proj->get_dt());
	  ADAS* pADAS = new ADAS(param_file,proj->get_dt()); 
	  p_model = pADAS;
	}
      
      else if(modelNumber==15){
	  //martin 10-08-2014: NH model
	  nhRef[itype]= NH(param_file,proj->get_dt());
	  NH* pNH = new NH(param_file,proj->get_dt()); 
	  p_model = pNH;
      }
      
      else if(modelNumber==16){
	  //martin 2015-02: CACC model
	  caccRef[itype]= CACC(param_file,proj->get_dt());
	  CACC* pCACC = new CACC(param_file,proj->get_dt()); 
	  p_model = pCACC;
      }
      
      else if(modelNumber==17){
	  //martin 2016-12: PCF model of Laval et al.
	  pcfRef[itype]= PCF(param_file,proj->get_dt());
	  PCF* pPCF = new PCF(param_file,proj->get_dt()); 
	  p_model = pPCF;
      }

      else if(modelNumber==18){
	  //martin 2017-09: LCM and DSM models
	  lcmRef[itype]= LCM(param_file,proj->get_dt());
	  LCM* pLCM = new LCM(param_file,proj->get_dt()); 
	  p_model = pLCM;
      }
      
     else if(modelNumber==19){
          //martin 2018-01: IDM with brownian motion
          bidmRef[itype]= BIDM(param_file,proj->get_dt());
          BIDM* pBIDM = new BIDM(param_file,proj->get_dt()); 
          p_model = pBIDM;
      }

     else if(modelNumber==20){
          //martin 2018-05: linear ACC model
          laccRef[itype]= LACC(param_file,proj->get_dt());
          LACC* pLACC = new LACC(param_file,proj->get_dt()); 
          p_model = pLACC;
      }

     else if(modelNumber==21){
          //martin 2018-12: Necessary Deceleration Model (NDM) Andresen et al
          ndmRef[itype]= NDM(param_file,proj->get_dt());
          NDM* pNDM = new NDM(param_file,proj->get_dt()); 
          p_model = pNDM;
      }
 
      else if(modelNumber==100){
	  // Template model

	  newmodelRef[itype]= NewModel(param_file,proj->get_dt());
	  NewModel* pNewModel= new NewModel(param_file,proj->get_dt()); 
	  p_model = pNewModel;
	}
      else
	{
	  cerr<<" Heterogen file Cstr: Error: modelNumber="<<modelNumber
	      <<" not yet implemented; current modelNumbers=0,1,2,...,9,100"<<endl;
	  exit(-1);
	}

      // write out fund diagrams
      if(true)
	{
	  char funddiaName[199];
	  char ext[200];
	  getExtension(itype,ext);
	  sprintf(funddiaName,"%s.fund%s", projectName, ext);
	  //p_modelRef[itype]->writeFunddia(funddiaName);
	  p_model->writeFunddia(funddiaName);
	}
      
    } // end for (int itype=0; itype<ntypes; itype++)
  
  cout <<"Exit Heterogen file Cstr: ntypes="<<ntypes<<endl;
  for (int itype=0; itype<ntypes; itype++)
    {
      char ext[200];
      getExtension(itype,ext);
      //cout<<" getExtension("<<itype<<")="<<ext<<endl; 
    }
  
  //exit(0);
}  // file Cstr

// double getFrac(int itype) etc. in Heterogen.h

//################################################################
Heterogen::~Heterogen()
//################################################################
{
  cout<<"deconstructor of Heterog "<<endl;
  for(int i=0;i<=NTYPEMAX;i++)
    {
      //if(p_modelRef[i]) delete p_modelRef[i];
    }

}




//################################################################
int Heterogen::get_type(double randomNumber) const{
//################################################################
  int type=0;
  double  sumFrac=getFrac(0);
  while((sumFrac<randomNumber)&&(type<get_ntypes())){
    type++;
    sumFrac+=getFrac(type);
  }
  if(false){
    cout<<"Heterogen.get_type: randomNumber="<<randomNumber
        <<" sumFrac="<<sumFrac
        <<" type="<<type<<endl;
  }

  return(type);
}


//################################################################
void Heterogen::getHeterogenparams(){
//################################################################
  InOut inout;
  char fname[199+1];
  sprintf(fname, "%s.heterog", projectName);

  ifstream  infile (fname, ios::in);
  if(!infile)  // default: one type "IDM1"
    {
      ntypes=0;
      fraction[0]=1;
      modelNumber[0]=0;
      setNumber[0]=1;
    }
  else
    {
      infile.close(); // get_col has own file handling
      inout.get_col(fname, 1, ntypes, fraction);
      inout.get_col(fname, 2, ntypes, modelNumber);
      inout.get_col(fname, 3, ntypes, setNumber);
    }
  for (int i=0; i<ntypes; i++)
    {
      cout<<"fraction="<<fraction[i]
	  <<" modelNumber="<<modelNumber[i]
	  <<" setNumber="<<setNumber[i]<<endl;
    }
}

//################################################################
//char* Heterogen::getExtension(int itype) const{
void Heterogen::getExtension(int itype, char* extension) const{
//################################################################
 // "IDM1", "VW2", etc
  char modelName[199];


  if(itype>ntypes-1){
    cerr<<"Heterogen::getExtension: Argument itype="<<itype
	<<" >= ntypes="<<ntypes<<endl
	<<" maybe probabilities in .heterogen file add to less than 1"
	<<" or incompatible .IC* files"<<endl;

    exit(-1);
  }
  else{
    if(modelNumber[itype]==0){sprintf(modelName,"IDM");}
    else if(modelNumber[itype]==1){sprintf(modelName,"VW");}
    else if(modelNumber[itype]==2){sprintf(modelName,"HDM");}
    else if(modelNumber[itype]==3){sprintf(modelName,"OVM");}
    else if(modelNumber[itype]==4){sprintf(modelName,"FPE");}
    else if(modelNumber[itype]==5){sprintf(modelName,"HSM");} //HumanSchreck
    else if(modelNumber[itype]==6){sprintf(modelName,"VDT");}
    else if(modelNumber[itype]==7){sprintf(modelName,"FVDM");}
    else if(modelNumber[itype]==8){sprintf(modelName,"KER");} //todo
    else if(modelNumber[itype]==9){sprintf(modelName,"CDDA");}
    else if(modelNumber[itype]==10){sprintf(modelName,"GIP");} //Gipps
    else if(modelNumber[itype]==11){sprintf(modelName,"KCA");} //Kerner's CA
    else if(modelNumber[itype]==12){sprintf(modelName,"PT");} //PT model
    else if(modelNumber[itype]==13){sprintf(modelName,"ASGM");} //ASGM
    else if(modelNumber[itype]==14){sprintf(modelName,"ADAS");} //ADAS
    else if(modelNumber[itype]==15){sprintf(modelName,"NH");} //NH model
    else if(modelNumber[itype]==16){sprintf(modelName,"CACC");}
    else if(modelNumber[itype]==17){sprintf(modelName,"PCF");}
    else if(modelNumber[itype]==18){sprintf(modelName,"LCM");}
    else if(modelNumber[itype]==19){sprintf(modelName,"BIDM");}
    else if(modelNumber[itype]==20){sprintf(modelName,"LACC");}
    else if(modelNumber[itype]==21){sprintf(modelName,"NDM");}
    else if(modelNumber[itype]==100){sprintf(modelName,"NEW");}
    else{sprintf(modelName,"ToBeImplemented");}
    sprintf(extension,"%s%i", modelName, setNumber[itype]);
  }
  if(false){cout <<"Heterogen.getExtension(itype="<<itype<<"): "
       <<" setNumber[itype]="<<setNumber[itype]
		 <<" extension= "<<extension <<endl;
  }
  return;
} //getExtension


//################################################################
//wird von Boundary.update_upstream aufgerufen

MicroModel* Heterogen::new_pmodel(int itype) const{
//################################################################
  char param_file[199];
  char ext[200];
  getExtension(itype,ext);
  sprintf(param_file,"%s.%s",projectName, ext/*getExtension(itype)*/);
  if(false){
    cout<<"\nHeterogen.get_model: itype="<<itype
	<<" param_file="<<param_file
	<<endl;
  }
  int modelNumber=get_modelNumber(itype);
  MicroModel* p_model=0;

  // new noetig, da ansonsten Speicher nach Verlassen von new_pmodel
  // geloescht wird => Speicherfehler. new erzeugt Pointer!

  if(modelNumber==0){
    IDM* pidm= new IDM(idmRef[itype]); 
    p_model=pidm;                  //!! slower, but every veh indiv.params
    //p_model=p_modelRef[itype];   // !! faster, but only view own models 
  }
  else if(modelNumber==1){
    VW* pvw = new VW(vwRef[itype]); 
    p_model=pvw;                //!! slower, but every veh indiv.params
    //p_model=p_modelRef[itype];  // !! faster, but only view own models 
  }
  else if(modelNumber==2){
    HDM* phdm = new HDM(hdmRef[itype]); //copy constr --> shallow copy
    //if(proj->get_with_distributed_T_react()) phdm->init_T_react(); //arne, feb06
    if(phdm->with_distributed_T_react()) phdm->init_T_react(); //arne, feb06
    p_model=phdm;  
   //p_model=p_modelRef[itype];  // !! faster, but only view own models
  }
  else if(modelNumber==3){
    OVM* povm = new OVM(ovmRef[itype]); 
    p_model=povm;                //!! slower, but every veh indiv.params
    //p_model=p_modelRef[itype];  // !! faster, but only view own models 
  }
  else if(modelNumber==4){
    FPE* pfpe = new FPE(fpeRef[itype]); 
    p_model=pfpe;                //!! slower, but every veh indiv.params
    //p_model=p_modelRef[itype];  // !! faster, but only view own models 
  }
  else if(modelNumber==5){
    HumanSchreck* phumanSchreck = new HumanSchreck(humanSchreckRef[itype]); 
    p_model=phumanSchreck;        //
  }
  else if(modelNumber==6){
    VDT* pvdt = new VDT(vdtRef[itype]); 
    p_model=pvdt;  
  }
  else if(modelNumber==7){
    FVDM* pvdiff = new FVDM(vdiffRef[itype]); 
    p_model=pvdiff;    
  }
  else if(modelNumber==8){
    Kerner* pKerner = new Kerner(kernerRef[itype]); 
    p_model=pKerner;
  }
  else if(modelNumber==9){
    CDDA* pCDDA = new CDDA(cddaRef[itype]); 
    p_model=pCDDA;
  }
  else if(modelNumber==10){
    Gipps* pGipps = new Gipps(gippsRef[itype]); 
    p_model=pGipps;
  }
  else if(modelNumber==11){
    KernerCA* pKernerCA = new KernerCA(kernercaRef[itype]); 
    p_model=pKernerCA;
  }
  else if(modelNumber==12){
    PTmodel* pPTmodel = new PTmodel(ptmodelRef[itype]); 
    p_model=pPTmodel;
  }
  else if(modelNumber==13){
    ASGM* pASGM = new ASGM(asgmRef[itype]); 
    p_model=pASGM;
  }
  else if(modelNumber==14){
    ADAS* pADAS = new ADAS(adasRef[itype]); 
    p_model=pADAS;
  }
  else if(modelNumber==15){
    NH* pNH = new NH(nhRef[itype]); 
    p_model=pNH;
  }
  else if(modelNumber==16){
    CACC* pCACC = new CACC(caccRef[itype]); 
    p_model=pCACC;
  }
  else if(modelNumber==17){
    PCF* pPCF = new PCF(pcfRef[itype]); 
    p_model=pPCF;
  }
  else if(modelNumber==18){
    LCM* pLCM = new LCM(lcmRef[itype]); 
    p_model=pLCM;
  }
  else if(modelNumber==19){
    BIDM* pBIDM = new BIDM(bidmRef[itype]); 
    p_model=pBIDM;
  }
  else if(modelNumber==20){
    LACC* pLACC = new LACC(laccRef[itype]); 
    p_model=pLACC;
  }
  else if(modelNumber==21){
    NDM* pNDM = new NDM(ndmRef[itype]); 
    p_model=pNDM;
  }
  else if(modelNumber==100){
    NewModel* pNewModel = new NewModel(newmodelRef[itype]); 
    p_model=pNewModel;
  }
  else{
    cerr<<"MicroModel.get_model: model number "
	<<modelNumber<<" not implemented!\n";
    exit(-1);
    VW vw = VW(param_file,proj->get_dt()); // not reached; to suppress warning
    p_model=&vw;   // that p_model might be uniinitialized (not the case)
  }
  return(p_model);
}

//################################################################
// not needed: in roadsection decision is based on initial vehicles
//on road (really sim based and not input based)
//
//bool Heterogen::HDM_with_distributed_T_react()
//{
//  for (int itype=0; itype<ntypes; itype++)
//    {
//      if(hdmRef[itype].with_distributed_T_react()) return(true);
//    }
//  return(false);
//}


















