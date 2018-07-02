#ifndef HET_H
#define HET_H

//own

#include "constants.h"
#include "ProjectParams.h"
#include "MicroModel.h"

#include "Fluct.h"

#include "IDM.h" // Model 0: Intelligent-Driver Model (IDM)
#include "VW.h"
#include "HDM.h" // Model 2: Human Driver Model

#include "OVM.h"
#include "FPE.h"
#include "HumanSchreck.h"
#include "VDT.h"
#include "FVDM.h"  // Full velocity difference model
#include "Kerner.h" // Model 8: Kerner's continuous micromodel
#include "CDDA.h"   // Model 9: Helbing/Treiber,
                    // continuous deceleration-delayed-acceleration model 
                    // arxiv.org/pdf/physics/0507178

#include "Gipps.h"    // Model 10: Simplified Gipps b=B
#include "KernerCA.h" // Model 11: KKW model
#include "PTmodel.h"  // Model 12: prospect-theoretic model
#include "ASGM.h"     // Model 13: Tian, Treiber et al, average space gap (CA) model
                      // Physica A  391, Issue 11, p. 3129-3139 (2011).
#include "ADAS.h" // Model 14: advanced driver assistance systems model (TU Delft) (2012)
#include "NH.h"   // Model 15: "Non-hypothetical" model Treiber/Jun-Fan (2014)
#include "CACC.h" // Model 16: Cooperative ACC (MT 2015-02)
#include "PCF.h"  // Model 17: Parsimonious CF model of Laval 10.1016/j.trb.2014.09.004
#include "LCM.h"  // Model 18: Longitudinal Control Model, see LCM.README
#include "BIDM.h" // Model 19: IDM with Brownian motion (BIDM) 2016_JunFang_stochastic/
#include "LACC.h" // Model 20: Linear ACC Model, see LACC.README

#include "NewModel.h" 




class CyclicBuffer;

// modelNumber ={0=IDM,1=VW,2=Human, 3=OVM, 4=FPE, 5=humanSchreck,
//       6=VDT,7=FVDM, 8=Kerner, 9=CDDA, 10=Gipps,
//      11=KernerCA, 12=PT model, 13=ASGM, 14=ADAS, 15=NH, 16=CACC, 
//      17=PCF, 18=LCM, 19=BIDM,  20=LACC, 100=NewModel}
// type=line number in .heterogen file of the project!

class Heterogen
{
 public:

  Heterogen();
  Heterogen(const char projectName[], ProjectParams* const proj);

  virtual ~Heterogen();

  int get_ntypes() const {return ntypes;}  // itype=0,1,2, ..., ntypes-1 
  int get_type(double randomNumber) const; //randomNumber in [0,1]
  int get_modelNumber(int itype) const {return modelNumber[itype];}
  Fluct get_fluct(int itype) const {return fluctRef[itype];}
  int get_setNumber (int itype) const {return setNumber[itype];}
  double getFrac (int itype) const {return fraction[itype];}
  // char* getExtension(int itype) const; // "IDM1", "VW2", etc; itype=0..ntypes-1 
  void getExtension(int itype, char* extension) const;
  MicroModel* new_pmodel(int itype) const;

  //bool HDM_with_distributed_T_react();

 private:

  ProjectParams* proj;
  char projectName[199];

  int ntypes;  // how many different vehicle types

  double fraction[NTYPEMAX+1];  // percentage of vehicles of this type
  int modelNumber[NTYPEMAX+1];  // model number of this type (0=IDM,1=VW)
  int setNumber[NTYPEMAX+1]; // model parameters in <proj>.<ext>

  
  //not very elegant: default constr are implicitly called

  /** mt mar07: Fluct mit Parametern in .fluct<n>
      where <n>=nth line of .heterog. file
  */
  
  Fluct fluctRef[NTYPEMAX+1]; 

  IDM idmRef[NTYPEMAX+1]; 
  VW  vwRef[NTYPEMAX+1];
  HDM hdmRef[NTYPEMAX+1];
  VDT vdtRef[NTYPEMAX+1];
  HumanSchreck humanSchreckRef[NTYPEMAX+1];
  OVM  ovmRef[NTYPEMAX+1];
  FVDM vdiffRef[NTYPEMAX+1];
  FPE  fpeRef[NTYPEMAX+1];
  Kerner kernerRef[NTYPEMAX+1];
  CDDA cddaRef[NTYPEMAX+1];
  NewModel newmodelRef[NTYPEMAX+1];
  Gipps gippsRef[NTYPEMAX+1];
  KernerCA kernercaRef[NTYPEMAX+1];
  PTmodel ptmodelRef[NTYPEMAX+1];
  ASGM asgmRef[NTYPEMAX+1];
  ADAS adasRef[NTYPEMAX+1];
  NH nhRef[NTYPEMAX+1];
  CACC caccRef[NTYPEMAX+1];
  PCF pcfRef[NTYPEMAX+1];
  LCM lcmRef[NTYPEMAX+1];
  BIDM bidmRef[NTYPEMAX+1];
  LACC laccRef[NTYPEMAX+1];

  //  NewModel  newmodelRef[NTYPEMAX+1];
  // MicroModel* p_modelRef[NTYPEMAX+1]; 

  void getHeterogenparams();
};


#endif // HET_H

