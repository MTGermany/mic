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
#include "constants.h"
#include "InOut.h"
#include "FloatingCars.h"

FloatingCars::FloatingCars() {
	;
}

FloatingCars::FloatingCars(const char projectName[], ProjectParams* proj) {
	this->dt = proj->get_dt();
	this->tstart = proj->get_tstart();
	this->isRing = (proj->get_choice_BCup() == 3);
	sprintf(this->projectName, "%s", projectName);

	InOut inout;
	char fname[199];
	sprintf(fname, "%s.floatcars", projectName);
	ifstream infile(fname, ios::in);
	int* carIndicesLocal = new int[NVEHMAX]; //dummy
	if (!infile) {
		carNumber = 0;
	} else {
		infile.close(); // get_col has own file handling
		inout.get_col(fname, 1, carNumber, carIndicesLocal);
	}

	cout << "carNumber=" << carNumber << endl;

	// dynamic allocation of memory (nov2010)
	int nFloatingCars = carNumber;
	carIndicesOriginal = new int[nFloatingCars+1]; //static index (for filename)
	carIndices = new int[nFloatingCars+1]; //dynamic index (ramp shifts index)
	carIDs = new int[nFloatingCars+1];
	cumFuel = new double[nFloatingCars+1];
	comfort = new double[nFloatingCars+1];
	FH_cars = new FILE*[nFloatingCars+1];

	//init vehicle IDs
	for (int i = 0; i < nFloatingCars; i++) {
		carIDs[i] = -1;
		FH_cars[i] = 0;
		cumFuel[i]=0;
		comfort[i]=0;
	}

	//dublicate car indices array since actual index is dynamical
	for (int i = 0; i < carNumber; i++) {
		carIndices[i] = carIndicesLocal[i];
		carIndicesOriginal[i] = carIndicesLocal[i];
	}
	delete[] carIndicesLocal;

	// fuelConsumption now in RoadSection and/or Heterogen
	// MT 2012 No! Needed here as well for micro considerations!!
	fuelConsumptionFC = 0;

	char carName[200];
	char engineDataName[200];
	char carDataName[200];

	sprintf(carName, "%s", "1");
	sprintf(engineDataName, "%s.engineData%s", projectName, carName);
	sprintf(carDataName, "%s.carData%s", projectName, carName);

	ifstream infileEngine(engineDataName, ios::in);
	ifstream infileCar(carDataName, ios::in);
	calcFuelConsumption=infileCar;
	if (!calcFuelConsumption) {
	  calcFuelConsumption = false;
	  cout << "RoadSection.get_fueldata: No car data sheet file "
	       << carDataName
	       << "! => don't calculate fuel consumption" << endl;
	}
	else{
	  //calcFuelConsumption = true;
	  useEngineDataSheet=infileEngine;
	  infileEngine.close();
	  infileCar.close();

	  bool testSimpleFormula = true;
	  bool testJanteFormula = true;
	  fuelConsumptionFC = new Consumption(proj->get_dt(), projectName,
					      engineDataName, carDataName,
					      useEngineDataSheet,
					      testSimpleFormula, testJanteFormula);
	}

	cout << "FloatingCars Cstr: carNumber=" << this->carNumber << endl;
}

FloatingCars::~FloatingCars() {
	cout<<"decontructor of FloatingCars"<<endl;
	for (int i = 0; i < carNumber; i++) {
		cout<<"i="<<i<<endl;
		if (FH_cars[i] != 0)
			fclose( FH_cars[i]);
	}
	 //delete dynamic arrays
	if(carIndicesOriginal) delete[] carIndicesOriginal;
	if(carIndices) delete[] carIndices;
	if(carIDs) delete[] carIDs;
	if(FH_cars) delete[] FH_cars;
	if(cumFuel) delete[] cumFuel;
	if(comfort) delete[] comfort;

}

void FloatingCars::write(Vehicle veh[], int imin, int imax, int it, double dtout) {

  for (int ifloat = 0; ifloat < carNumber; ifloat++) {
    // for periodic BC vehicle indices not ident. to vehicles
    //arne: 9-6-2005: index for ramp szenarios shifted 
    //--> bugfix by introducing "getActualVehicleIndex" method

    //cout <<" FloatingCars.write: ifloat="<<ifloat<<" iveh="<<iveh<<endl;
    int iveh_actual = getActualVehicleIndex(veh, imin, imax, ifloat);
    if ((it == 0) || ((iveh_actual >= imin) && (iveh_actual <= imax))) {
      int iveh = carIndicesOriginal[ifloat]; //always the same!
      char out_fname[199];
      sprintf(out_fname, "%s.car%i", projectName, iveh);
      if (FH_cars[ifloat] == 0) {

	// Arne: too many open Filehandles will also lead to segmentation fault!
	FH_cars[ifloat] = fopen(out_fname, "w");
	filecheck(FH_cars[ifloat], out_fname);
      }

      if (it == 0) {
	fprintf( FH_cars[ifloat],"# useEngineDataSheet=%s\n",
		 ((useEngineDataSheet) ? "true" : "false"));
	fprintf( FH_cars[ifloat],"# t(s)\t x(m)\tv(m/s)\ta(SI)\ts(m)\tdv(m/s)\t gear\t fuel(l/h)\tcum.Fuel(l)\tcomfort((m/s^2)^2)\n");
      }
      if ((iveh_actual >= imin) && (iveh_actual <= imax)) {
	double t = tstart + it * dt;
	double x = veh[iveh_actual].getPos();
	double v = veh[iveh_actual].getVel();
	double a = veh[iveh_actual].getAcc();
	double jerk = veh[iveh_actual].getJerk();

	double s = veh[iveh_actual - 1].getPos() - veh[iveh_actual- 1].getLength() - x;
	double dv = v - veh[iveh_actual - 1].getVel();
	double instFuel = 0;
	int gear = 0;
	//cout <<"FloatingCars.write: calcFuelConsumption="<<calcFuelConsumption<<endl;exit(0);

	double tFuelStart=99.5; //!!! Hack MT mar 2015 for VW COOL2
	if (calcFuelConsumption&&(it*dt>tFuelStart)) {
	  instFuel= 3600*fuelConsumptionFC->getFuelFlowFixedGearscheme(v, a, jerk, gear, useEngineDataSheet);
	  cumFuel[ifloat] += instFuel/3600.*dtout;
	  comfort[ifloat] += comfortFun(a, jerk)*dtout;
	  cout <<"dtout="<<dtout<<endl;
	}

	//hack for time-delay: (see HDM.acc error message is too much output)
	if ((!isRing && veh[iveh_actual].getModelNumber() == 2)
	    && iveh_actual >= imax - NVEH_DELAY_EXCLUDED
	    && veh[iveh_actual].getPos() < DX_DELAY_EXCLUDED_M) {
	  fprintf(FH_cars[ifloat],
		  "# iveh>imax-10=>no HDM delay! %6.3f\t%6.2f\t%6.3f\t%6.3f\t%6.4f\t%6.3f\t%3i\t%8.3f\t%6.3f\t\t%6.3f\n",
		  t, x, v, a, s, dv, 
		  gear, instFuel, cumFuel[ifloat], comfort[ifloat]);
	} else {
	  fprintf(FH_cars[ifloat],
		  "%6.3f\t%6.2f\t%6.3f\t%6.3f\t%6.4f\t%6.3f\t%3i\t%8.3f\t%6.3f\t\t%6.3f\n",
		  t, x, v, a, s, dv, 
		  gear, instFuel, cumFuel[ifloat], comfort[ifloat]);
	}
	fflush( FH_cars[ifloat]);
      }
    }
  }
}

//iveh is origin veh number
//method returns correct present vehicle index in array corresponding to initial veh ID
int FloatingCars::getActualVehicleIndex(Vehicle veh[], int imin, int imax,
		int iFloat) {
	int index = -1;
	int iveh = carIndices[iFloat];

	//not in actual vehicle array:
	if (iveh < imin || iveh > imax)
		return -1;

	//fetch vehicle and memorize its id as reference:
	//if( carIDs[iFloat] < 0 && (iveh>=imin && iveh<=imax) )
	if (carIDs[iFloat] < 0  ) {
		//map vehicle index with vehicle id
		carIDs[iFloat] = veh[iveh].getID();
		return iveh;
	}

	//ring road: martins condition (still to test)
	int nveh = imax - imin;
	int i = (isRing) ? iveh + nveh * (static_cast<int> ((nveh - iveh + imin)
			/ nveh)) : iveh;
	//search vehicle for given ID in vehicle array:

	while (veh[i].getID() != carIDs[iFloat]) {
		//set_in for positive ramp shifts towards higher indices! best for performance!
		i++;
		if (i > imax)
			i = imin;
		//printf("i=%i, id=%i, seek id=%i and start with i=%i\n",i,veh[i].getID(),carIDs[iFloat],iveh);
		//is still to test for ringroad !!!
		if (i == iveh) {
			//printf("floatingCars::error in finding floatcar for index=%i and id=%i. Exit to avoid infty loop!\n", index, iveh);
			//exit(-1);
			//no more in array when vehicle is lifted/dropped
			return (index);
		}
	}
	index = i;
	carIndices[iFloat] = index; //udate dynamic index for new position in veh[] !!!
	//printf("floatingCars::success for index=%i and id=%i, write index to carIndices!\n", index, veh[index].getID());
	return (index);
}


/// MT mar 15: instantaneous driving comfort=f(acceleration, jerk)

double FloatingCars::comfortFun(double acc, double jerk){
  double pref_a=1.;  // if =1: unit acceleration squared 
  double pref_jerk=2;  // sqrt(1./pref_jerk) is jerk value
                       //  as uncomfortable as an acceleration of 1 m/s^2
  return pref_a*acc*acc+pref_jerk*jerk*jerk;
}
