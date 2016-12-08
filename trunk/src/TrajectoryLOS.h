#ifndef TRAJECOTRYLOS_H
#define TRAJECTORYLOS_H
#include <vector>

#include "Vehicle.h"
#include "Consumption.h"

class TrajectoryLOS{

  class FCD
  {
  public:
    FCD(double t0, int id,double x0,double x1){
      this->t0 = t0;
      this->id = id;
      this->x0 = x0;
      this->x1 = x1;
      t=0;
      x=0;
      t_hack=0;
      lcomf=0;
      linvTTC=0;
      fuelCons=0;
      aComf2=0;
      a0=0.05;
      startMeasurement=false;
    }
    void update(int id, double t, double x, double dt,
		double s, double dv, double a,
		double fuel)
    {
      //printf("update veh id=%i\n",id);
      //just check for development
      this->x = x; 
      if(this->id!=id){
	printf("error:: trying to update veh id=%i with veh id=%i. exit\n",this->id,id);
	exit(-1);
      }
      if(x>=x0 && !startMeasurement){
	startMeasurement=true;
	this->t0 = t;
      }
      if(startMeasurement && x<=x1){
	this->t  = t;
	fuelCons+= dt*fuel;
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//HACK: bereich um Rampe ausschliessen fuer acc/TTC: 
	
	if(x<10500 || x> 11500){
	  t_hack  += dt;
	  lcomf   += dt*a*a;
	  linvTTC += max(dt*dv/s,0.0);
	  double a0=0.05;
	  aComf2  += dt*max(a0,fabs(a));
	}
      }
    };


    int getID(){return id;}
    double dataOK(){return (startMeasurement && x>=x1);}
    double getT0(){return t0;}
    double getTT(){return t-t0;}
    double getComf(){return lcomf/t_hack/*(t-t0)*/;}  //keine Wurzel!!!
    double getInvTTC(){return linvTTC/t_hack/*(t-t0)*/;}
    double getFuelCons(){return fuelCons;} 
    double getComfDirk(){return aComf2/(a0*t_hack);}
    
  private:
    double x0,x1;
    double t_hack;
    bool startMeasurement;
    double x;
    int id; //for check, just debugging
    double t0;
    double t;
    double lcomf;
    double linvTTC;
    double fuelCons;
    double aComf2;
    double a0;//=0.05;
		
	
  };


 public:
  TrajectoryLOS(const char* filename, double dt);
  void update(double t,Vehicle veh[], int imin, int imax); 

 private:
  double dt;
  double x0; //start x position
  double x1; //end position for traj

	FILE* FH_out;

  std::vector<FCD> fcdVector;
  int iVectorMin;

  char filename[200];
  void write(double t0,double tt,double comf,double invTTC, double fuel, double comf_dirk);
  int getVehicleIndex(int id,Vehicle veh[],int imin, int imax);
  bool isInVector(int id);

  bool calcFuelConsumption;
  Consumption* fuelConsumptionFC; //eine FuelClass for all?	

};

#endif // TRAJECTORYLOS_H
