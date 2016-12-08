#ifndef FLOATINGCARS_H
#define FLOATINGCARS_H

//own

#include "Vehicle.h"
#include "ProjectParams.h"
#include "Consumption.h"

class FloatingCars{ 

 public:

  FloatingCars();
  FloatingCars(const char projectName[], ProjectParams* proj);
  virtual ~FloatingCars();

  bool floatCarsExist(){return (carNumber>-1);}
  int  getCarNumber(){return (carNumber+1);}
  void write(Vehicle veh[], int imin, int imax, int it, double dtout);

 private:
  int getActualVehicleIndex(Vehicle veh[], int imin, int imax, int ifloat);
  double comfortFun(double acc, double jerk); // MT mar 15
  char projectName[199];
  double tstart;
  bool isRing;
  double dt;

  // now dynamic memory allocation (Nov2010)
  int* carIndicesOriginal;
  int* carIndices;
  int* carIDs;
  double* cumFuel;
  double* comfort;
  FILE** FH_cars;
	
  int carNumber;

  bool calcFuelConsumption;
  bool useEngineDataSheet;
  Consumption* fuelConsumptionFC; //eine FuelClass for all?

};

#endif // FLOATINGCARS_H
