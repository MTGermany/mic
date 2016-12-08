#ifndef BOUNDARY_H
#define BOUNDARY_H



#include "ProjectParams.h"
#include "Heterogen.h"
//#include "Fluct.h"
class Vehicle;

class Boundary{ 

 public:

  Boundary();
  Boundary(const char* projectName, bool isUpstream, ProjectParams* proj);


  //void updateUpstream(Vehicle *vehp[], int& imin, int& imax, double time); 
  //void updateDownstream(Vehicle *vehp[], int& imin, int& imax, double time); 

  //arne: hier het als referenz uebergeben --> starke performance verbesserung!
  void updateUpstream(Vehicle veh[], int& imin, int& imax,Heterogen* het, double time); 
  void updateDownstream(Vehicle veh[], int& imin, int& imax, double time); 
  double getFlow(double time_s);
  double getVel(double time_s);

  //arne:
  void set_choiceBC(int new_choiceBC);
  int get_choiceBC(){return choice_BC;}

  double get_nWait(){return nWait;}


  void ctrl_setQcap(double Qcap_invs){ctrl_Qcap=Qcap_invs;}
  double ctrl_getQcap(){return ctrl_Qcap;}
  double ctrl_get_nWait(){return ctrl_nWait;}

 private: 

  char projectName[199];
  ProjectParams* proj;
  double dt;
  double xmax;

  //Fluct fluct;
  // if up:choice_BC   = {0=Dirichlet,1=Neumann,2=zero,3=period,4=flowControl}
  // if down:choice_BC = {0=Dirichlet,1=Neumann,2=free,3=period,4=block}

  double nWait;
  int nInsertedBC;
  bool isUpstream;
  int choice_BC; 
  int n_times;
  double times[NBC+1];       // only needed to calcul. vBC_l,_r
  double vBC[NBC+1];       // only needed to calcul. vBC_l,_r
  double QBC[NBC+1];       // only needed to calcul. vBC_l,_r

  bool isCtrl;
  double ctrl_Qcap;
  double ctrl_nWait;
  double ctrl_flow;
  void ctrl_update(double time);
  void ctrl_writeLog(double time);

};

#endif // BOUNDARY_H
