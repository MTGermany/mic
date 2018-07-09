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
#include "InOut.h"
#include "Consumption.h"



double testFuel(double v){
  //return ((v<28.9)&&(v>28.1));// signal v=28.8713 // mt jun18
  return false;
}


// MT 2015: .engineData1 and .carData1 necessary for fuel consumption! 
// analytic consumption not implemented??

Consumption::Consumption(){;}

Consumption::Consumption(double dt, 
			 const char projectName[], 
			 const char engineDataName[], 
			 const char carDataName[],
			 bool useEngineDataSheet,
			 bool writeAnalyticData,
			 bool writeTabulatedData){
  initializeConstants();

  //this->proj=proj;
  //  this->dt=proj->dt;
  this->dt=dt;
  sprintf(this->projectName, "%s", projectName);
  sprintf(this->engineDataName, "%s", engineDataName);
  sprintf(this->carDataName, "%s", carDataName);
  bool useCarDataFromFile=(strlen(carDataName)!=0);

  cout <<"Consumption Cstr: strlen(engineDataName)="
       <<strlen(engineDataName)<<" useEngineDataSheet="<<useEngineDataSheet<<endl; 
  this->writeAnalyticData=writeAnalyticData;

  if(useEngineDataSheet){
    readEngineMap(engineDataName); //defines also fmin,fmax
  }


 


  if(useCarDataFromFile){
    readCarData(carDataName);
  }
  else{
    powMax =85000;  // max. mech Motorleistung Pmech (W)
    mveh  =1200;  // mass of vehicle (kg)
    cw     =0.30;  // hydrodynamical cw-value (dimensionless)
    A      =2.5;  // front area of vehicle (m^2)
    mu0    =0.015; // constant friction coefficient (dimensionless)
    mu1    =0.0003;// friction coefficient propto v (s/m)
    powEl   =2000; // Leerlaufleistung bzw. elektr. abgenommene Leistung
    cForce=mveh*GRAV*mu0;
    dForce=mveh*GRAV*mu1;
    eForce=0.5*RHO_AIR*cw*A;

  }

  //cout <<"Consumption: before initialize()"<<endl;
  initialize();
  cout <<"Consumption: before writeAnalyticData()"<<endl;

  //#####################################################################
  // Activate only for Test of Consumption:
  //performSomeTests(); cout <<"Test completed: exiting ..."<<endl; exit(0);
  //#####################################################################


  if(writeAnalyticData){
    //setEnginePower(85000);
    writeConsumptFieldsAnalytic();
  }
  //cout <<"fmin="<<fmin<<" fmax="<<fmax<<endl; exit(0); exit(0);

  cout <<"useEngineDataSheet="<<useEngineDataSheet<<endl;
  cout <<"Consumption: before writeTabulateData()"<<endl;
  if(writeTabulatedData&&useEngineDataSheet){  // EngineDataSheet must exist!
    cout <<"before writeConsumptFieldsDatasheet(5): useEngineDataSheet="<<useEngineDataSheet<<endl;
    writeConsumptFieldsDatasheet(5,useEngineDataSheet);
    cout <<"after writeConsumptFieldsDatasheet(5): useEngineDataSheet="<<useEngineDataSheet<<endl;

    //writeConsumptFieldsDatasheet(4,useEngineDataSheet);
    //writeConsumptFieldsDatasheet(3,useEngineDataSheet);
    //writeConsumptFieldsDatasheet(2,useEngineDataSheet);
    //writeConsumptFieldsDatasheet(1,useEngineDataSheet);
    writeConsumptFieldsDatasheet(0,useEngineDataSheet);
  }



  cout <<"End Consumption.Cstr: writeTabulatedData="<<writeTabulatedData<<endl;
  //exit(0);
} // end file Cstr

void Consumption::initializeConstants(){

  CALORIC_DENS =40e6;// gaoline ("Benzin"): 44 MJ/kg approx 40 MJ/l
  RHO_AIR      =1.29;// (kg/m^3) @ 0 cels, 1014 hPa
  RHO_FUEL     =900; // of gasoline (kg/m^3)
  GRAV         =9.81;// grav. acceleration (m/s^2)

  // in nominator: transforms g/kWh => l/(Ws) in denom: reverse transform
  // (needed to transform the "Motorkennfeld" specConsField typically given in
  // g/kWh) and to get useful order of magn. for test screen output
  tabSpeccons2LiterPerSec   =1./(RHO_FUEL*3.6e6); 

  // if cons(l/(Ws)) higher, point (f,pe) out of Bounds
  LIMIT_SPEC_CONS =600*tabSpeccons2LiterPerSec; // 900=3-4 times the minimum


  // ! wegen Plot-Buchhaltung mind Faktor 100 notwendig!
  POW_ERROR      =1e7; // 10000 KW 
  FUELFLOW_ERROR =POW_ERROR *LIMIT_SPEC_CONS;

}


void Consumption::readCarData(const char* carDataName){

  cout <<"\nConsumption.readCarData:"
       <<" Reading vehicle data from file "
       <<carDataName<<endl;

  FILE* fp=0;
  fp=fopen(carDataName,"r");
  if(fp==NULL){
    cout << "Error: no file "<< carDataName <<" found"<<endl;
    cout <<"exiting!\n";exit(-1);
  }
	

  InOut inout;


  inout.getvar(fp,&powMax);
  inout.getvar(fp,&vol);
  inout.getvar(fp,&mveh);
  inout.getvar(fp,&cw);
  inout.getvar(fp,&A);
  inout.getvar(fp,&mu0);
  inout.getvar(fp,&mu1);
  inout.getvar(fp,&powEl);
  inout.getvar(fp,&phi[0]); // ! evtl. ohne "&"
  inout.getvar(fp,&phi[1]);
  inout.getvar(fp,&phi[2]);
  inout.getvar(fp,&phi[3]);
  inout.getvar(fp,&phi[4]);
  inout.getvar(fp,&rdyn);
  inout.getvar(fp,&pe_gear);

  inout.getvar(fp,&fmin ); // if getvar does not find a data line, it does nothing
  inout.getvar(fp,&fmax);

  
  cForce      = mveh*GRAV*mu0;
  dForce      = mveh*GRAV*mu1;
  eForce      = 0.5*RHO_AIR*cw*A;

  if(false){
    cout <<"Consumption.readCarData: mveh="<<mveh
	 <<" cForce="<<cForce<<" dForce="<<dForce<<" eForce="<<eForce<<" rdyn="<<rdyn<<endl;
    cout <<"fmin="<<fmin<<" fmax="<<fmax<<endl; exit(0);
  }
	
	fclose(fp);
}

  
void Consumption::readEngineMap(const char* engineDataName){

    ifstream  infile (engineDataName, ios::in);

    if(!infile){
      cerr<<"Consumption file Cstr: Error: No data sheet file "<<engineDataName<<"!"<<endl;
      cout <<"exiting!\n"; 
      exit(-1);
    }
		infile.close();
		// lese Datenfile ein
    InOut inout;
    inout.get_array2d_col(engineDataName,3,fminDatasheet,df,nf,pemin,dpe,npe,specConsField);

      // transform to SI units:
      // motor frequency 1/min => 1/s

      fminDatasheet /= 60;
      df /= 60;
      fmaxDatasheet=fminDatasheet+(nf-1)*df;
      fmin= fminDatasheet;  // may be overwritten by carData
      fmax= fmaxDatasheet;
      
       // effective pressure bar => Pa=N/m^2=10^{-5}*bar

      pemin *= 1e5;
      dpe *= 1e5;

      //  transforms g/kWh => liter/(Ws)

      for (int i=0; i<nf; i++){
        for (int j=0; j<npe; j++){
	  specConsField[i][j] *= tabSpeccons2LiterPerSec;
	}
      }

      cout <<"fminDatasheet="<<fminDatasheet
	   <<" fmaxDatasheet="<<fmaxDatasheet<<" pemin="<<pemin
                   <<" specConsField[0][0]="<<specConsField[0][0]
	   <<endl;


      // test for zero bin widths

      if (df<=1e-10){
         cerr<<" Consumption: Error: bin width d(frequency)=0"
	     <<endl;
	 exit(-1);
      }
      if (dpe<=1e-10){
         cerr<<" Consumption: Error: bin width d(pe)=0"
	     <<endl;
	 exit(-1);
      }

      // test data field and interpolation routine getSpecificConsumption

      if(false){
       for (int i=0; i<10; i++){
        for (int j=0; j<10; j++){
	  double f=fmin+(1.0001*i-1)*df;
	  double pe=pemin+(1.0001*j-1)*dpe;
	  cout <<"f="<<f<<" pe="<<pe
	       <<" specCons(g/kWh)="
               <<static_cast<int>(getSpecificConsumption(f,pe)/tabSpeccons2LiterPerSec)
	       <<endl;
	}
       }
      }

} // end readEngineMap


void Consumption::initialize(){

  // Wirkungsgrad (Bewegungsleistung/Brennwert) Benzin: 15-35% 
  //(http://www.ch-forschung.ch/index.php?artid=122)

    consumedFuel=0;
    consumptionRate=0;
    travelledDistance=0;
    travelledTime=0;


}

double Consumption::getSpecificConsumption(double f, double pe){
  double peIn=max(pe,pemin);
  double fIn=max(f,fmin);
  double ir=min( max( (fIn-fmin)/df, 0.), nf-1-1e-10);
  double jr=min( max( (peIn-pemin)/dpe, 0.), npe-1-1e-10);
  int i=static_cast<int>(ir);
  int j=static_cast<int>(jr);
  double ifrac=ir-i;
  double jfrac=jr-j;
  double specCons=(1-ifrac)*( (1-jfrac)*specConsField[i  ][j  ]
			    +jfrac *specConsField[i  ][j+1])
                 +ifrac*( (1-jfrac)*specConsField[i+1][j  ]
			    +jfrac *specConsField[i+1][j+1]);
  if((f<fmin)||(f>fmax)){
     specCons *=100; // mt mar07: hohe "Strafe" fuer unerlaubten Arbeitsbereich
  }
  return specCons;
}


//#####################################
// fuel flow for given gear in liters/s; acc and v in SI units
  // gear=0 ("Idle gear"), 1 (first gear),2,3,...
// if gear=1 and motor rpm Idle or below, gear 0 assumed as well
// central method!
//##################################### //
double Consumption::getFuelFlow(double v, double acc, double jerk, int gear, 
				bool useEngineDataSheet){

  bool withWarnings=false;
  int gearIndex=gear-1;
  double fuelFlow=0;
  
  double specCons=0; // specific consumption in m^3/(Ws)
  double fmot=0;
  double pe=0;
  double pe_el=0;

  // powEl=electr power (loss incl.)
  // "Schubabschaltung" (see line defining fuelFlow below):
  // active if powMechEl+0.5*pe_gear*fmot*vol<0 (pe_gear=inner motor friction)
  // This takes into account real balance including battery charge state!
  double forceMech=getForceMech(v,acc); // can be <0
  double powMechEl=v*forceMech + powEl;// can be <0

  if(useEngineDataSheet){
    // specific Consumption according to augmented Jante diagrams

    if(!(rdyn>0)){
      cerr<<" Consumption.getFuelFlow.Error: useEngineDataSheet=true "
	  <<" but no car parameters read!\n";
      exit(-1);
    }
    double rdyn2pi=2*PI*rdyn;
    fmot=(gearIndex>-1) ? phi[gearIndex]*v/rdyn2pi : fmin;

    pe_el = 2*powEl/(vol*fmot);  // part of pe due to electrical power
    pe=pe_gear+pe_el+ 2*rdyn2pi/(phi[gearIndex]*vol) * forceMech;

    //if(pe<pemin){pe=pemin;} // mt mar07: auskommentiert!
    //if(fmot<fmin){fmot=fmin;} // mt mar07: auskommentiert!

    specCons=getSpecificConsumption(fmot,pe); // m^3 Benzin/(Ws)
    fuelFlow=specCons*max(powMechEl+0.5*pe_gear*fmot*vol, 0.);


  } // end if useEngineDataSheet


  //####################################################################
  //!! MT mar15 simplified analytic formula
  //####################################################################

  else{ // simplified analytic formula for specific Consumption without Jante
    double eta0   =0.20; // idling efficiency mech/therm
    double etaMax =0.35; // maximum efficiency mech/therm
    powRel_etaMax = 0.52; // rel. motor power at maximum efficiency
    fRel_etaMax = 0.4; // rel. rotation rate f/fmax at maximum efficiency

    // define fmin, fmax only if not given in .carData1 file
    // MT mar15 Not Yet Used here !!!
    if(!((fmin>0)&&(fmin<50))){fmin   = 900/60.;} // idling rotation rate per s
    if(!((fmax>10)&&(fmax<200))){fmax = 6300/60.;} // max rot rate


    specCons0=1./(CALORIC_DENS*eta0); // liter/Ws
    specConsMin=1./(CALORIC_DENS*etaMax);
    double specCons2=(specCons0-specConsMin)/pow(powRel_etaMax,2);

    specCons=specConsMin + specCons2*pow(powMechEl/powMax-powRel_etaMax, 2);


    //!! main central result!
    fuelFlow=specCons*max(powMechEl+0.5*pe_gear*fmot*vol, 0.); // liter/s


    
    // MT mar 15: !!! JERK: Approximate inefficiencies due to instationarities 
    // by punishing jerks

    double jerk4doublingConsumpt=1;
    double acc4doublingConsumpt=10;
    double increaseByJerk=fuelFlow*pow(jerk/jerk4doublingConsumpt, 2);
    double increaseByAcc=fuelFlow*pow(acc/acc4doublingConsumpt, 2);

    fuelFlow += increaseByJerk+increaseByAcc;


    // checks

    if(false){
      cout <<"getFuelFlow analytic: v="<<v<<" acc="<<acc<<" gear="<<gear
	   <<" useEngineDataSheet="<<useEngineDataSheet<<" specCons="<<specCons
	   <<" power="<<max(powMechEl+0.5*pe_gear*fmot*vol, 0.)
	   <<" fuelFlow="<<fuelFlow<<endl;
    }
  } // end simplified analytic formula




  //####################################################################
  // ### check if motor regime can be reached; otherwise increase 
  // fuelFlow prohibitively (mar15: only used if useEngineDataSheet)
  //####################################################################

  // required power too high

  if(useEngineDataSheet){
    if((powMechEl-powEl)>powMax){ 
      if(withWarnings){
      cout <<"  Consumption.getFuelFlow: warning:"
	   <<" v_kmh="<<(3.6*v) <<" acc="<<acc
           <<"\n    => Required power="<<(0.001*(powMechEl-powEl))
           <<" kW > maximum power!\n";
      }
      fuelFlow=FUELFLOW_ERROR;
    }
  }


  // engine rotation rate too high

  if(useEngineDataSheet&&(fmot>fmax)){ 
    if(withWarnings){
      cout <<"  Consumption.getFuelFlow: warning:"
	   <<" v_kmh="<<(3.6*v) <<" acc="<<acc<<" gear="<<gear
	   <<" motor frequency ="<<static_cast<int>(fmot*60) <<"/min too high!\n";
    }
    fuelFlow=FUELFLOW_ERROR;
  }
  
  // engine rotation rate too low

  if(useEngineDataSheet&&(fmot<fmin)){
    if(gear==1){ // allowed for first gear; then "Leerlauf" or "Kupplung"
          fmot=fmin;
          fuelFlow=powEl*LIMIT_SPEC_CONS;
	  //cout <<"v="<<v<<" gear="<<gear<<" fuelFlow="<<fuelFlow<<endl;
    }
    else{
      fuelFlow=FUELFLOW_ERROR;
      //if(withWarnings){
      if((gear==5)&&(v>13)&&(v<17)){
        cout <<"  Consumption.getFuelFlow: warning:"
	     <<" v_kmh="<<(3.6*v) <<" acc="<<acc<<" gear="<<gear
	     <<" motor frequency ="<<static_cast<int>(fmot*60) <<"/min too low!\n";
      }
    }
  }


  //####################################################################
  //  Test
  // BUG DOS sometimes; solved by replacing "+=" by own variable
  // in RoadSection.write_FuelConsumption; see BUG DOS there
  //####################################################################

 
  if(testFuel(v)){
 
      //if((acc<0)&&(v>11)&&(v<13)&&(gear==5) ){// mt mar07


      cout<<"\nIn core Consumption.getFuelFlow: v="<<v
          <<" acc="<<acc<<" gear="<<gear<<endl
	  <<"  fmot="<<static_cast<int>(60*fmot)<<"/min,"
	  <<" fmin="<<(60*fmin)
	  <<" pe="<<(pe*1e-5)<<"hPa,"
	  <<" pe_el="<<(pe_el*1e-5)<<"hPa,"
	// <<" pe_gear="<<(pe_gear*1e-5)<<"hPa,"
	  <<" specific cons="<<(specCons/tabSpeccons2LiterPerSec)<<"g/kWh"
	  <<"\n  max(powMechEl+powMotFrict,0)="
          <<(max(powMechEl+0.5*pe_gear*fmot*vol, 0.))
	  <<" fuelFlow="<<fuelFlow
	  <<" l/100km="<<(1e5*fuelFlow/v)
	  <<endl;
      //exit(-1);
  }

  return fuelFlow;

} // getFuelFlow




  //####################################################################
double Consumption::getMinFuelFlow(double v, double acc, double jerk, 
				   int& gear, bool useEngineDataSheet){
  //####################################################################

  double fuelFlow=FUELFLOW_ERROR;
  for (int testgear=5; testgear>0; testgear--){ // direction reversed!
    double fuelFlowGear=getFuelFlow(v,acc,jerk,testgear,useEngineDataSheet);
    if(fuelFlowGear<fuelFlow){
      gear=testgear;
      fuelFlow=fuelFlowGear;
      if(false){
        cout <<"Consumption.getminFuelFlow: useEngineDataSheet="<<useEngineDataSheet
	     <<" v="<<v<<" acc="<<acc<<" gear="<<gear
             <<" fuelFlowGear="<<fuelFlowGear<<endl;
      }
    }
  }
  //if(false){
    if((v<4)&&(fuelFlow>=0.999*FUELFLOW_ERROR)){
    cerr <<"Consumption::getMinFuelFlow(v="<<v<<", acc="<<acc<<"):\n "
	 <<" this (v,acc) cannot be reached with any gear!\n";
    cout <<"getFuelFlow(v,acc,jerk,1,useEngineDataSheet)="
	 <<getFuelFlow(v,acc,jerk,1,useEngineDataSheet)<<endl;
  }
  return fuelFlow;
}

double Consumption::getFuelFlowFixedGearscheme(double v, double acc, double jerk,
				   int& gearFixed, bool useEngineDataSheet){

  double dv=5.667; // velocity intervall for each gear (20 km/h=5.667)
  gearFixed=static_cast<int>(v/dv + 1);
  if(gearFixed>5){gearFixed=5;}

  double fuelFlow=getFuelFlow(v,acc,jerk,gearFixed,useEngineDataSheet);

  if(fuelFlow>=0.999*FUELFLOW_ERROR){
    fuelFlow=getMinFuelFlow(v,acc,jerk,gearFixed,useEngineDataSheet);
    cerr <<"Consumption::getFuelFlowFixedGearscheme(v="<<v
         <<", acc="<<acc<<"):\n "
	       <<" this (v,acc) cannot be reached with gear "<<gearFixed<<"!\n"
	       <<" schalte runter!\n";
  }

  //if(testFuel(v)){
  if(false){

    //if(true){
    cout <<" in Consumption.getFuelFlowFixedGearscheme:"<<endl
	 <<" v="<<v<<" acc="<<acc<<" jerk="<<jerk<<endl
	 <<" gearFixed="<<gearFixed
	 <<" useEngineDataSheet="<<useEngineDataSheet
	 <<" fuelFlow="<<fuelFlow<<endl;
    //exit(-1);
  }
  return fuelFlow;
}


void Consumption::writeConsumptFieldsDatasheet(int gear, bool useEngineDataSheet){
  // Jante-Diagramm ...
  cout <<"writeConsumptFieldsDatasheet: gear="<<gear<<" useEngineDataSheet="<<useEngineDataSheet<<endl;

  if(!useEngineDataSheet){
    cerr<<"Consumption.writeConsumptFieldsDatasheet: Error: "
	<<" need data sheet to use this function"<<endl;
    exit(-1);
  }

  bool determineOptimalGear=(gear<1);
  char filename[199];
  if(determineOptimalGear){
    sprintf(filename, "%s.JanteGearOpt", projectName);
  }
  else{
    sprintf(filename, "%s.Jante%i", projectName, gear);
  }
  ofstream  outfile (filename, ios::out);
  if(!outfile)  
  {
    cerr<<"Consumption.writeConsumptFieldsDatasheet: Error: file "<<filename
	<<"not accessible for writing!"<<endl;
    exit(-1);
  }

  outfile<<"# Fuel consumption based on engine data sheet:\n"
	 <<"# v(km/h)  acc(m/s^2) \tforceMech(N)\tpowMech(kW)\t"
	 <<"fuelFlow(l/h)\tconsump(liters/100km)\tGear"
         <<endl;

  const int NV=41; //101 // mt mar07
  const int NACC=31; //61
  double jerk=0; // for stationary situations
  double accmin=-1;
  double accmax=2;
  double vmin_kmh=0;
  double vmax_kmh=200;
  double dv_kmh=(vmax_kmh-vmin_kmh)/(NV-1);
  double dacc=(accmax-accmin)/(NACC-1);
  for (int iv=0; iv<NV; iv++){
    for (int iacc=0; iacc<NACC; iacc++){
      double v_kmh=vmin_kmh+iv*dv_kmh;
      double v=v_kmh/3.6;
      double acc=0.001*static_cast<int>(1000*(accmin+iacc*dacc));
      double forceMech=getForceMech(v,acc);
      double powMechEl=max(v*forceMech + powEl, 0.); 
      double fuelFlow=100000;
      if(determineOptimalGear){ // v=const => min(consump)=min(fuelFlow)
	fuelFlow=getMinFuelFlow(v,acc,jerk,gear,true);  
                         // => reference to optimum gear!
      }
      else{
        fuelFlow=getFuelFlow(v,acc,jerk,gear,true);
      }

      double consump_100km=1e5*fuelFlow/max(v,0.001);
      double fuelFlow_lh=3600*fuelFlow;

      outfile<<v_kmh<<"\t\t"<<acc<<"\t"<<forceMech
             <<"    \t"<<(0.001*powMechEl)<<"   \t"<<fuelFlow_lh
             <<"\t\t"<<consump_100km <<"\t"<<gear<<endl;
    }
    outfile<<endl;
  }
  outfile.close();
  //exit(0);
}



void Consumption::writeConsumptFieldsAnalytic(){

  char filename[199];
  sprintf(filename, "%s.consumpAnalytic%iKW", projectName, static_cast<int>(0.001*powMax));

  ofstream  outfile (filename, ios::out);
  if(!outfile)  
  {
    cerr<<"Consumption.writeConsumptFieldsAnalytic: Error: file "<<filename
	<<"not accessible for writing!"<<endl;
    exit(-1);
  }
  outfile<<"#Min instantaneous distance related Fuel consumption [l/100 km]"<<endl
	 <<"# output from writeConsumptFieldsAnalytic"<<endl
	 <<"# v[km/h]\tacc[m/s^2]\tgear(n.y.relevant)\tF[N]\tfuelFlow[l/s]\tfuel100 [l/100km]"
         <<endl;

  if(true)cout <<"cForce="<<cForce<<" dForce="<<dForce<<" eForce="<<eForce<<" mveh="<<mveh<<endl;

  const int NV=11;
  const int NACC=11;
  double jerk=0;  // stationary situations
  double accmin=-2;
  double accmax=2;
  double vmin_kmh=0;
  double vmax_kmh=200;
  double dv_kmh=(vmax_kmh-vmin_kmh)/(NV-1);
  double dacc=(accmax-accmin)/(NACC-1);
  for (int iacc=0; iacc<NACC; iacc++){
    for (int iv=0; iv<NV; iv++){
      double v_kmh=vmin_kmh+iv*dv_kmh;
      double v=v_kmh/3.6;
      double acc=accmin+iacc*dacc;
      //int gearOpt; // defined in getMinInstConsumption100km(...)
      int gear=5;

      //double flow=getMinFuelFlow(v,acc,jerk,gearOpt,false);
      double flow5=getFuelFlow(v,acc,jerk,gear,false); // gear does not yet play role
      //double fuel100=getMinInstConsumption100km(v,acc,jerk,gearOpt,false);
      double fuel100=getInstConsumption100km(v,acc,jerk,gear,false);
      double forceMech=getForceMech(v,acc);
      outfile<<v_kmh<<"\t\t"<<acc<<"\t\t"<<gear<<"\t" 
             <<forceMech<<"\t"<<flow5<<"\t"<<fuel100<<endl;
    }
    outfile<<endl;
  }
  outfile.close(); 
}



void Consumption::update(double v, double acc, double jerk){

  this->v=v;
  this->acc=acc;
  travelledDistance += v*dt;

  // select gear !

  double dvgear=5;
  int gear=static_cast<int>(v/dvgear) + 1;
  if(gear>5){gear=5;}

  consumptionRate=getFuelFlow(v,acc,jerk,gear,useEngineDataSheet);
  consumedFuel += consumptionRate*dt;
}


  // Test methods of this class with some specific examples
void Consumption::performSomeTests(){
  int n=11;
  double vField[]={0,0,0,15,15,15,30,30,30,20,20};
  double aField[]={-0.5,0,1,-0.5,0,1,-0.5,0,1,0.5,0.5};
  int gearField[]={1,1,1,4,4,4,5,5,5,3,5};

  double jerk=0; // stationary situations
  cout <<"Consumption.performSomeTests() ...\n"<<endl;

  for (int i=0; i<n; i++){
    double v=vField[i];
    double a=aField[i];
    int gear=gearField[i];
    int gearOpt;
    int gearFixed;
    bool jante=true;
    cout <<"\nv="<<(3.6*v)<<" km/h,"
	 <<"a="<<     a <<" m/s^2,"
	 <<"gear="<<gear<<", Fuel flows in liter/h, ..Cons.. in liter/100km:"
	 <<endl;


      cout
	 <<"\n getForceMech(v,a) [N]                            = "<<getForceMech(v,a)
	 <<"\n getFuelFlow(v,a,jerk,gear,jante)                      = "
         <<(3600*getFuelFlow(v,a,jerk,gear,jante))<<" l/h"
	 <<", gear="<<gear
	 <<"\n getFuelFlow(v,a,jerk,gear,!jante)                     = "
         <<(3600*getFuelFlow(v,a,jerk,gear,!jante))<<" l/h"
	 <<", gear="<<gear<<" (not yet relevant for !jante)"
	 <<"\n getMinFuelFlow(v,a,jerk,gearOpt,jante)                = "
	 <<(3600*getMinFuelFlow(v,a,jerk,gearOpt,jante))<<" l/h"
	 <<", gearOpt="<<gearOpt
	 <<"\n getFuelFlowFixedGearscheme(v,a,jerk,gearFixed,jante)  = "
	 <<(3600*getFuelFlowFixedGearscheme(v,a,jerk,gearFixed,jante))<<" l/h"
	 <<", gearFixed="<<gearFixed
	 <<"\n getInstConsumption100km(v,a,jerk,gear,jante)          = "
	 << getInstConsumption100km(v,a,jerk,gear,jante)<<" l/100km"
	 <<", gear="<<gear
	 <<"\n getMinInstConsumption100km(v,a,jerk,gear,jante)       = "
	 << getMinInstConsumption100km(v,a,jerk,gearOpt,jante)<<" l/100km"
	 <<", gearOpt="<<gearOpt
	 <<"\n getMinInstConsumption100km(v,a,jerk,gear,!jante)      = "
	 << getInstConsumption100km(v,a,jerk,gear,!jante)<<" l/100km"
	 <<", gear="<<gear<<" (not yet relevant for !jante)"
	 <<endl;
    
  }
}






