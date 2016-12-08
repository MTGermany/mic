#ifndef CONST
#define CONST

const int RESIGNATION_ON = 1;   // 0=false, 1=true --> in IDM.cpp

///DRASTISCHE auswirkung auf Simulation!!!
const int DROP_NONENTERING_RAMP_VEHICLES = 1;//0=false, true=1, bool geht nicht
                                             //--> in Ramp.cpp
const double MINSPACE_MERGE_M=1.0; //--> used in Ramp.cpp
const double RAMP_VEL_REDUCEFACTOR=0.6; //!!! VDT 0.5, HSM 0.7, allg 0.5 .. 0.7
// remove RoadSection.o+recompile after change of RAMP_VEL_REDUCEFACTOR

//const int NT_HISTORY=10;    // Number of saved past timesteps for each vehicle
const int NXMAX    = 200;    // Max. number of x values in macr. 3d data
const int NTMAX    = 200;    // Max. number of t values in macr. 3d data
const int NTABMAX    = 1001;    // Max. number of tabulated data (PT model)

const int NVEHMAX  = 50001; //arne 21-1-2005: 10001;    // Max. number of vehicles 
const int NTYPEMAX = 50;       // Max. number of different vehicle types
const int NDETMAX  = 32; //16;       // Max. number of virtual detectors
const int NTRMPMAX = 5;        // max. number of pert positions
const int NRMPMAX  = 100;        // max. number of ramp positions


const int N_ANTICIPATION_MAX=20; //max. number of anticipated vehicles
const int NVEH_DELAY_EXCLUDED=2; //hack for vehicle upstream inflow 
const double DX_DELAY_EXCLUDED_M=300; //hack for vehicle upstream (switch off reaction time)

// arne: increase from 10 to 300 to be flexible in choosing small dt! 4 feb 2004
//arne: mit cyclicBuffer nicht mehr noetig !!

const char COMMENTCHAR = '%';
const char COMMENTCHAR2 = '#';
const int  NSTRMAX = 256;

const int NBC      = 1441;      // max. number of jumps for Dirichlet-BC
const int NRHO     = 50;       // time critical 100=>20: 30% faster

const double SMALL_VAL = 1.e-7;
const double LARGE_VAL = 1./SMALL_VAL;

#endif //CONST


