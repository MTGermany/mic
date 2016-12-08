// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
#include <fstream>
using namespace std;

#include "general.h"
#include "TrajectoryLOS.h"

//standard constructor
TrajectoryLOS::TrajectoryLOS(const char* project, double dt)
{ 
  cout<<"Cstr. of TrajectoryLOS...\n";
  this->dt=dt;
  //define start and end position for traj analysis (depends on ASM bounds)
  //FIXXED HERE  !!!!!!!!!!! DESIGNED FOR HCT/TSG PROJECT !!!!!!!!!!!!!!!!!!!!
  x0=3000;
  x1=14000;
  iVectorMin=0;
  
  sprintf(filename,"%s.tseries_los",project);
	
	FH_out=0; //oct06
	
  FH_out=fopen(filename,"w");

  if(FH_out==0){
    printf("TrajectoryLOS:: Error: file \"%s\" not accessible for writing!\n",filename);
    exit(-1);
  } 

  fprintf(FH_out,"#unreliable data at beginning from init. conditions...\n");
  fprintf(FH_out,"#start: x0=%.3f m, end: x1=%.3f m\n",x0,x1); 
  fprintf(FH_out,"#comfort defined as averaged squared acc --> <(acc)^2>)(m/s^2)^2\n");
  fprintf(FH_out,"#t(h) \t\t tt(min) \t comf(m/s^2)^2 \t dummy \t  invTTC (1/s) \t <fuelCons>(l)\n");
  fflush(FH_out);


  // fuelConsumption now in RoadSection and/or Heterogen
  //arne: test 17-6-2005
  // kann man mit schalter ausschalten !!!!
  fuelConsumptionFC=0;
  calcFuelConsumption=true;
  
  if(calcFuelConsumption){
    char carName[200];
    sprintf(carName,"%s","1");
    char engineDataName[200];
    char carDataName[200];
    sprintf(engineDataName, "%s.engineData%s", project, carName);
    sprintf(carDataName,    "%s.carData%s", project, carName);
    
    ifstream infileCar(carDataName, ios::in);
    ifstream infileEngine(engineDataName, ios::in);
    
    if(!infileCar){
      cout<<"RoadSection.get_fueldata: No car data sheet file "
	  <<engineDataName <<"! => don't calculate fuel consumption"<<endl;
      calcFuelConsumption=false;
      exit(-1);
    }
    else{
      bool useEngineDataSheet=infileEngine;
      infileEngine.close();
      infileCar.close();
      bool testSimpleFormula=true;
      bool testJanteFormula=true;
      fuelConsumptionFC = new Consumption(dt,project,engineDataName,carDataName,
					  useEngineDataSheet,
					  testSimpleFormula,testJanteFormula);
    }
  }
    
} 


void TrajectoryLOS::update(double t, Vehicle veh[], int imin, int imax)
{
  //first: update existing fcd ....
  for(vector<FCD>::iterator iter=fcdVector.begin();iter<fcdVector.end();iter++)
    {
      //seek for id in vehArray and return id
      int id=(*iter).getID();
      int i=getVehicleIndex(id,veh,imin,imax);
      if(i>=0){
	//okay, update !
	int id=veh[i].getID();
	double x=veh[i].getPos();
	double s=(i-1<imin)? 100000 : veh[i-1].getPos()-veh[i-1].getLength()-veh[i].getPos();
	double dv=(i-1<imin)? 0 : veh[i-1].getVel()-veh[i].getVel(); 
	double a=veh[i].getAcc();
	double jerk=veh[i].getJerk();
	//and actual fuel consumption (new:)
	double instFuel=0;
	if(calcFuelConsumption){
	  int gear;
	  bool withJante=true;
	  double v=veh[i].getVel(); 
	  //Consumption in SI: m^3 --> 1m^3 = 1000 liter
	  double instFuelCons_m3 = fuelConsumptionFC->getFuelFlowFixedGearscheme(v,a,jerk,gear,withJante);
	  //aber fuer v=0, a=0 gibt es 10000 l/h als "error" -> quick hack
	  //setze im stillstand 1 l/h an !!!
	  instFuel = (instFuelCons_m3>0.002) ? instFuelCons_m3/10. : 1000*instFuelCons_m3;
	}
	(*iter).update(id,t,x,dt,s,dv,a, instFuel);
      }
      else{
	//delete vehicle from vector!
	double t0 = (*iter).getT0();
	double tt = (*iter).getTT();
	double comf = (*iter).getComf();
	double invTTC = (*iter).getInvTTC();
	double fuelCons = (calcFuelConsumption) ? (*iter).getFuelCons() : 0;
	double comf_dirk = (*iter).getComfDirk();
	if((*iter).dataOK()) write(t0,tt,comf,invTTC, fuelCons, comf_dirk);
	fcdVector.erase(iter);
	iVectorMin++;
      }
    }
  //and add new vehicles to vector:
  int istart=1+iVectorMin+fcdVector.size();
  for(int i=istart;i<imax;i++){
    int id=veh[i].getID();
    double pos = veh[i].getPos();
    //check ob schon drin 
    if( !isInVector(id) && pos<x0 ) fcdVector.push_back(FCD(t,id,x0,x1)); //add to end of list
  }
}

//no performance at all !!!!
int TrajectoryLOS::getVehicleIndex(int id,Vehicle veh[],int imin, int imax)
{
  for(int i=imin;i<=imax;i++)
    {
      if(veh[i].getID()==id) return(i);
    }
  return(-1); //failed !
}

//no performance at all !!!!
bool TrajectoryLOS::isInVector(int id)
{ 
  for(vector<FCD>::iterator iter=fcdVector.begin();iter<fcdVector.end();iter++)
    {
      if( id==(*iter).getID() ) return true;
    }
  return false;
}



void TrajectoryLOS::write(double t0,double tt,double comf,double invTTC, double fuel, double comf_dirk)
{  
  //printf("write : t0=%.3f\n",t0);
  //write data:
 // FILE *FH_out=fopen(filename,"a"); //append mode !!!

  if(FH_out==0){
    printf("TrajectoryLOS:: Error: file \"%s\" not accessible for writing!\n",filename);
    exit(-1);
  } 
  double zero=0.0;
  fprintf(FH_out,"%.4f \t %.4f \t %.8f \t %.4f \t %.7f \t %.6f \t %.6f\n",t0/3600.,tt/60.,comf,zero,invTTC, fuel, comf_dirk);
  fflush(FH_out);
}
