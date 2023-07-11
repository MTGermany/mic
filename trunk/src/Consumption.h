
#ifndef CONSUMPTION_H
#define CONSUMPTION_H
 

// MT 2015: .engineData1 and .carData1 necessary for fuel consumption! 
// analytic consumption not implemented??


//own
//#include "ProjectParams.h"

 
class Consumption{ 
  // http://www.ch-forschung.ch/index.php?artid=122
 public:

  static const int NFMAX  = 201;  // maximum number of f steps in data sheet
  static const int NPEMAX = 201; // maximum number of pe steps in data sheet
  // !!! must be the same as InOut.NYMAX !!!
  
  Consumption();
  Consumption(double dt,  const char projectName[],
	      const char engineDataName[], const char carDataName[], 
	      bool useEngineDataSheet, bool writeAnalyticData, bool writeTabulatedData);

  /// read consumption-relevant vehicle data from a file
  void  readCarData(const char* carDataName);

  /// read engine "Kennfeld" from a file
  void  readEngineMap(const char* engineDataName);

  /// fuel consumption flow for given gear in m^3/s (SI); v, acc in SI
  /// gear=0 ("Leerlauf", 1 (first gear),2,3,...
  double getFuelFlow(double v, double acc, double jerk, 
		     int gear, bool useEngineDataSheet);

  /// optium fuel consumption flow in m^3/s
  /// gives also reference to fuel-optimized gear
  double getMinFuelFlow(double v, double acc, double jerk, 
			int& gear, bool useEngineDataSheet);

  // same for fixed gearscheme: gear=(int)(v/dv+1)
  double getFuelFlowFixedGearscheme(double v, double acc, double jerk, 
				    int& gear, bool useEngineDataSheet);

  /// update overall consumption, distance etc.
  void update(double v, double acc, double jerk);

  /// actual distance-related fuel consumption (l/(100 km)=1e5 l/m)
  double getInstConsumption100km (double v, double acc, double jerk, 
				  int gear, bool useEngineDataSheet){
    return(1e5*getFuelFlow(v,acc,jerk,gear,useEngineDataSheet)/max(v,0.001));
  }

  /// minimum distance-related fuel consumption (l/(100 km)=1e5 l/m)
  double getMinInstConsumption100km (double v, double acc, double jerk, 
				  int& gear, bool useEngineDataSheet){
    return(1e5*getMinFuelFlow(v,acc,jerk,gear,useEngineDataSheet)/max(v,0.001));
  }



  /// Average fuel consumption for all past update() steps (l/(100 km))
  double getAvgConsumption100km ();

  void setEnginePower(double powMax){this->powMax=powMax;}

  double getForceMech(double v, double acc){
    //return max(c+d*v+e*SQR(v)+mveh*acc, 0.001);
    return cForce+dForce*v+eForce*v*v+mveh*acc;
  }

  void writeConsumptFieldsDatasheet(int gear);
  void writeConsumptFieldsAnalytic();

  // gear >=1: writes mechPower, consumption etc as f(v,acc) for a given gear
  // gear=0: writes the optimal gear (with respect to fuel eff) as f(v,acc)
  void writeEngineMaps(int gear);

  // calculate specific consumption (m^3/(Ws))
  double getSpecificConsumption(double f, double pe);

  // Test methods of this class with some specific examples
  void performSomeTests();


 private:


  //##############################
  // Constants
  //##############################

   double CALORIC_DENS; // =40e6;// "Benzin": 44 MJ/kg approx 40 MJ/l
   double RHO_AIR;      // =1.29;// (kg/m^3) @ 0 cels, 1014 hPa
   double RHO_FUEL;     // =900; // of "Benzin" (kg/m^3)
   double GRAV;         // =9.81;// grav. acceleration (m/s^2)

   // in nominator: transforms g/kWh => liter/(Ws) in denom: reverse transform
   // (needed to transform the "Motorkennfeld" consField typically given in
   // g/kWh) and to get useful order of magn. for test screen output
   double tabSpeccons2LiterPerSec;  // =1./(RHO_FUEL*3.6e6); 
   double LIMIT_SPEC_CONS;// if cons(l/(Ws)) higher, 
                                          // point (f,pe) out of Bounds
 // !! wegen Plot-Buchhaltung mind Faktor 100 notwendig!
   double POW_ERROR;     // =1e7; // 10000 KW 
   double FUELFLOW_ERROR; // =POW_ERROR *FUELFLOW_2SI*LIMIT_SPEC_CONS;
  // extremely high flow in motor regimes that cannot be reached

   double cForce; // mveh*GRAV*mu0
   double dForce; // mveh*GRAV*mu1
   double eForce; // 0.5*RHO_AIR*cw*A


  //##############################
  // state variables
  //###############################

   //  ProjectParams* proj;
  double dt;
  bool useEngineDataSheet;
  bool writeAnalyticData;

  char projectName[200];
  char engineDataName[200];
  char carDataName[200];

  double v,acc;
  double consumedFuel;
  double consumptionRate;
  double travelledDistance;
  double travelledTime;


  //##############################
  // Variables of the engine data field
  //##############################

  double specConsField[NFMAX][NPEMAX]; // specific consumption (liter/Ws)

  // motor rotation rates (1/s)

  double fminDatasheet;  // minimum frequency(1/s) in data sheet
  double fmaxDatasheet;  // maximum frequency(1/s) in data sheet
  double df;    // frequency step size in data sheet (1/s)
  int    nf;    // number of f steps in data sheet

  // effective pressure difference for mech energy ("Mitteldruck") (Pa=N/m^2) 

  double pemin; // minimum pe(1/s)
  double dpe;   // pe step size in data sheet (1/s)
  int    npe;   // number of pe steps in data sheet

  

  //##############################
  // vehicle-related  variables
  //###############################

    double powMax;  // max. effective mechanical engine power (W)
    double vol;  // effective volume of the cylinders of the engine
    double mveh;  // mass of vehicle (kg)
    double cw;  // hydrodynamical cw-value (dimensionless)
    // VW: 0,48 (Kaefer), 0.42-0.46 (Golf,Passat,Scirocco 1984), 0.29 (heute)
    double A;  // front area of vehicle (m^2)
    double mu0; // constant friction coefficient (dimensionless)
    double mu1;// friction coefficient propto v (s/m)
    double powEl;  // power (W) for electrical consumption

    double pe_gear; // effective part of pe lost by gear friction (N/m^2)
    double phi[6]; // transmission ratios for the gears
    double rdyn;  // dynamical tyre radius (<static r)(m)

  // fmin,fmax from most carData files;
  // if not or if no carData file used; from engine data field or from default
  double fmin;     // idling rotation rate per s
  double fmax;     // max rotation rate per s 

    



  //##############################
  // Analytical "ad-hoc" calculation of fuel consumption
  // constants for consumption 
  // for  getFuelFlow(...), writeMap(...) if !useDataSheet
  //##############################

  // dQ/dt=cons*(p0 + pMech)
  // cons=consMin + cons2*(powMech/powMax-powRel_consMin)^2
  // powMech=v*forceMech*theta(forceMech)
  // forceMech=c+dv+ev^2+mveh*dv/dt => above

  double specCons0;     // spec consumption fuel/(brutto mech. work) (liter/J) Leerlauf
  double specConsMin;   // spec consumption (liter/J) at optimal efficiency
  double etaMax;  // maximum efficiency
  double powRel_etaMax; // rel. motor power at optimal efficiency
  double fRel_etaMax; // rel. rotation rate f/fmax at maximum efficiency


  void initializeConstants();
  void initialize();

};

#endif // CONSUMPTION_H
