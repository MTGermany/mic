// c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
#include <fstream>
using namespace std;

//##################################
#define __TIME_MEASURE__
#ifdef __TIME_MEASURE__
   #include <time.h>
   #define TIME_MEASURE(code) code
#else
   #define TIME_MEASURE(code) /* - */
#endif
//##################################

#include "RoadSection.h"
#include "CyclicBuffer.h"
#include "InOut.h"
#include "RandomUtils.h"




double testFuelRoadSec(double v){
  return ((v<28.9)&&(v>28.1));// signal v=28.8713 // mt jun18
  //return false;
}

//#######################################################################
// constructor
//######################################################################

RoadSection::RoadSection(const char projectName[], ProjectParams* proj)
{
  cout <<"Entering RoadSection Cstr ..."<<endl;
  double randomNumber= myRand();
  cout <<"randomNumber="<<randomNumber<<endl;

  this->proj=proj;
  this->xmin=proj->get_xmin();
  this->xmax=proj->get_xmax();
  this->tstart=proj->get_tstart();
  this->dt=proj->get_dt();
  //this->fluct=Fluct(projectName, proj->get_dt());
  sprintf(this->projectName, "%s", projectName);

  flowCtrlExists=(proj->get_choice_BCup()==4);

  //init. global variables for accumulated quantities:
  //(also done in first call of method)

  accum_fuel_l=0; 
  accum_travtime_s=0;
  accum_acc=0;
  accum_jerk=0;
	
	//init fileHandles
	FH_fuel=0;
	FH_tt=0;
	FH_3dmic=0;
	FH_3dmac=0;


  //#############################################################
  // get flow-conserving road inhomogeneities (default: no inhomogeneities) 
  //#############################################################

  get_flowconsInhomog("alpha_v0", x_v0, arr_alpha_v0, n_v0);
  get_flowconsInhomog("alpha_T", x_T, arr_alpha_T, n_T);


  //#######################################################################
  // Test if fuel consumption should be calculated
  //#######################################################################
  get_fueldata("1");   // Assocoation to actual engines in run scripts:
                       // now (Jan 04): "1"="Passat2.0"

  // #######################################################
  // set up ramp infrastructure (default: no ramps)
  // ######################################################
  
  cout<< "RoadSection Cstr: Setting up ramp infrastructure ..."<<endl;
  
  get_ramps();
  
  cout<<"RoadSection Cstr: n_ramps="<<n_ramps<<" flow at t=300 s .."<<endl;
  for (int irmp=0; irmp<n_ramps; irmp++){
      cout<<" ramp[irmp].getLength()="<<ramp[irmp]->getLength()
	  <<" ramp[irmp].getFlow(300)="<<ramp[irmp]->getFlow(300)<<endl;
  }
  counter_rmpveh=0;
  counter_crashs=0; 
    

  //#######################################################
  // get parameters of the vehicle population (default: 1-type IDM)
  //#######################################################

  cout<< "RoadSection Cstr: Determining Vehicle population ..."<<endl;
  
  het = new Heterogen(projectName, proj);

  //#######################################################
  // get info about external lane-change effects 
  // (dropping or lifting vehicles) !!AFTER definition of het!
  //#######################################################
    
  cout<<"RoadSection Cstr: Determining lane-changing vehicles ..."<<endl;
  
  vehDropLift = new VehicleDropLift(projectName, proj, het);
 
  //#######################################################
  // get info about external acceleration influences 
  // (sudden braking of certain vehicles etc)
  //#######################################################

  cout<<"RoadSection Cstr: Determining external control..."<<endl;
  
  get_externalControl(); //=> sets externalCtrlExists
  if(externalCtrlExists){
      cout<<"RoadSection Cstr: following vehicles externally controlled:"<<endl;
      for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++)
	{
	  cout<<" i_vehControlled["<<iCtrl<<"]="
	      <<i_vehControlled[iCtrl]
	      <<endl;
	}
    }

  get_regionControl(); //=> sets regionCtrlExists
  if(regionCtrlExists){
      cout<<"RoadSection Cstr: externally controlled regions exist!"<<endl;
  }  

  //#######################################################
  // get boundary conditions (default: free upstream and downstream)
  //#######################################################

  boundary_up   = new Boundary(projectName, true,  proj); 
  boundary_down = new Boundary(projectName, false, proj); 

  if(true)
    {
      cout<<"RoadSection Cstr: Boundary conditions:"<<endl
	  <<" choice_BCup="<<proj->get_choice_BCup()
	  <<" choice_BCdown="<<proj->get_choice_BCdown()<<endl
	  <<" boundary_up.getFlow(300)="<<boundary_up->getFlow(300)
	  <<" boundary_down.getVel(0)="<<boundary_down->getVel(0)<<endl
	  <<" boundary_up.get_nWait()="<<boundary_up->get_nWait()
	  <<" boundary_up.ctrl_getQcap()="<<boundary_up->ctrl_getQcap()
	  <<endl;
    }


 
  // #############################################################
  // Get information about which output is wanted how
  // ndtout_3d and nxout_3d --> micdat (write_3Dmic) 
  // ndtout_3d  --> mac_output3D (write_3Dmac)
  // ndtout_2d  --> mac_output2D (write_2Dmac), TravelTime, Fuel, Comfort
  // ndtout_FCD --> floating cars
  // #############################################################

  double epsilon = 0.1; //to avoid cast problems here

  ndtout_3D  = static_cast<int>( max( (proj->get_dtout_3d())/dt +epsilon, 1.) ); 
  nxout_3D   = static_cast<int>( (xmax-xmin)/(max(1.,proj->get_dxout_3d())) );
  ndtout_2D  = static_cast<int>( max( (proj->get_dt_tseries())/dt +epsilon, 1.) );
  ndtout_FCD = static_cast<int>( max( (proj->get_dtout_FCD())/dt +epsilon, 1.) );
 
  if(true)
    {
      printf("\nRoadSection cstr:: output options:\n");
      printf("dtout_3d=%.3f, dxout_3d=%.3f, dnout_3d=%i, dt_tseries=%.3f, dtout_FCD=%.3f\n",
	     proj->get_dtout_3d(), proj->get_dxout_3d(), proj->get_dnout_3d(), proj->get_dt_tseries(), proj->get_dtout_FCD());
      printf("\nRoadSection cstr:: dt=%.3f --> ndtout_3D=%i, nxout_3D=%i, ndtout_2D=%i, ndtout_FCD=%i\n",
	     proj->get_dt(),ndtout_3D,nxout_3D,ndtout_2D,ndtout_FCD);
      printf("\nchosen output options:\ntravelTime_output=%s\n",proj->with_travelTime_output()? "true":"false");
    }
		

  // #############################################################
  // check if floating-car info and create a FloatingCars instance
  // #############################################################

  floatCars = new FloatingCars(projectName, proj); 
  cout<<"floatCars->getCarNumber()="<<floatCars->getCarNumber()<<endl;
  
  // #############################################################
  //arne: (fuer LOS project) --> sollte man per schalter aktivieren koennen
  // #############################################################

  trajLOS=0; //pointer init.
  if( proj->with_trajectory_los() ) trajLOS = new TrajectoryLOS(projectName, dt);
  

  // #############################################################
  // detectors 
  // #############################################################

  get_detectors();
  
  //#############################################################
  // new: distributed parameters for v0 and T
  // these parameters can easily varied by global alpha_v0 and alpha_T
  // (these two parameters are used by nearly all models!)
  // arne feb 06 (used for TGF05 )
  //#############################################################
 

  alpha_T_distr=0; //pointer init.
  alpha_v0_distr=0;

  distrSpan_v0_T=proj->distributionSpan_v0_T(); //new parameter in proj

  if(distrSpan_v0_T!=0.0)
    {
      cout<<"###################################################################"<<endl;

      alpha_T_distr = new double[NVEHMAX];  //arne, feb06
      alpha_v0_distr= new double[NVEHMAX];

      cout<<"RoadSection:: T and v0 via alpha_T_distr and alpha_v0_distr --> applied to all models!"<<endl;
      cout<<"RoadSection::  uniformly distributed with +/- "<<distrSpan_v0_T<<endl;
      for(int i=0;i<NVEHMAX;i++)	{
	      alpha_T_distr[i]  = uniform(1-distrSpan_v0_T, 1+distrSpan_v0_T);
	      alpha_v0_distr[i] = uniform(1-distrSpan_v0_T, 1+distrSpan_v0_T);
	      if(false)cout<<i<<".veh: alpha_T_distr="<<alpha_T_distr[i] <<" alpha_v0_distr="<<alpha_v0_distr[i]<<endl;
	    }
      cout<<"###################################################################"<<endl;
    }

  cout <<"End RoadSection Cstr"<<endl<<endl;

}// of constructor


  // #############################################################
  // initialize vehicles on the RoadSection
  // #############################################################
  //
  //     choice_init=0:  microscopic IC from 
  //                     <projectName>.ICsingle; 
  //     choice_init=1:  rho = rhoinit + asymmetric dipole-like peturbation 
  //                     of fixed width and positive amplitude ampl_init
  //                     as used by Kerner; velocity v=const; flow=rho*v.
  //     choice_init=2:  rho from file <projName>.IC
  //     choice_init=3:  rho,v from file <projName>.IC
  //
  // #############################################################

RoadSection::~RoadSection()
{
  cout <<"\n\nAt deconstructor of RoadSection:  total number of accidents="
       <<counter_crashs<<endl<<endl;

  //delete own objects (not proj and fluct!)
  if(fuelConsumption) delete fuelConsumption;
  if(boundary_up) delete boundary_up;
  if(boundary_down) delete boundary_down;
  if(het) delete het;
  if(floatCars) delete floatCars;
  if(vehDropLift) delete vehDropLift;
  if(cyclicBuf) delete cyclicBuf;

  //delete array of detectors:
  for (int idet=0; idet<n_det; idet++)
    {
      if(detector[idet]) delete detector[idet];
    }
  for(int irmp = 0; irmp<n_ramps; irmp++)
    {
      if(ramp[irmp]) delete ramp[irmp];
    }
  for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++)
    {
      if(externalControl[iCtrl]) delete externalControl[iCtrl];
    }
  if(trajLOS) delete trajLOS;
  if(alpha_T_distr) delete[] alpha_T_distr;
  if(alpha_v0_distr) delete[] alpha_v0_distr;
	
	//close FileHandles:
	if(FH_fuel!=0) fclose(FH_fuel);
	if(FH_tt!=0) fclose(FH_tt);
	if(FH_3dmic!=0) fclose(FH_3dmic);
	if(FH_3dmac!=0) fclose(FH_3dmac);
	for(unsigned int i=0; i<FH_2dmac_vec.size(); i++){
		if(FH_2dmac_vec.at(i)!=0) fclose(FH_2dmac_vec.at(i));
	}
	
	cout<<"Deconstructor of RoadSection finished ... \n";
}

// ##################################################
void RoadSection::initialize(int choice_init)
// ##################################################
{
  bool testinit=false;
  if(testinit)
    { 
      cout << "RoadSection.initialize: "
	   <<" proj->get_choice_init="<<proj->get_choice_init()
	   <<" choice_init="<<choice_init
	   <<": initializing vehicles ..." << endl;
    }

  bool isMacroIC = ((choice_init==1)||(choice_init==2)||(choice_init==3));
  bool isMicroIC = ((choice_init==0));

  if(testinit) cout <<"isMacroIC="<<isMacroIC<<" isMicroIC="<<isMicroIC<<endl;

  // #####################################################
  // initialize with macro IC
  // ####################################################

  if(isMacroIC)
    { 
      //macro IC
      double rhoinit[NVEHMAX];       // intermediate macroscopic fields
      double Qinit[NVEHMAX];         // !! rho,Q mac from 0..NVEHMAX,
      int nxMacro=int((NVEHMAX-1)/10); // NVEHMAX=50001, 5000 vals enough
      cout<<" RoadSection.initialize:macroIC: nxMacro="<<nxMacro<<endl;
      generateMacroFields(choice_init, rhoinit, Qinit, nxMacro);

      // Generate vehicles from macro-density and flow
      
      double xloc   = xmax;  // start from behind
      //double xlocmin = ( (proj->get_choice_BCup()) == 3 ) ? -SMALL_VAL :0; 
      double xlocmin =0;
      
      imin=0;
      imax=0;
      
      while(xloc>xlocmin)
	{
	  if (imax>NVEHMAX-1) error("initial cond imply imax > NVEHMAX-1!");
	  
	  double randomNumber= myRand();
	  int type           = het->get_type(randomNumber);
	  if(testinit)
	    {
	      cout <<"RoadSection.initialize (macro): randomNumber= "
		   <<randomNumber<<" type="<<type<<endl;
	    }
	  //bool hasReactTime  = (het->get_modelNumber(type)==2 || het->get_modelNumber(type)==3); //also OVM
	  // generateMacroFields: rho guaranteed to be > RHOMIN, v to be < VMAX
	  double rholoc = intpextp(rhoinit, nxMacro,  xloc, xmin, xmax);
	  double vloc   = intpextp(Qinit, nxMacro, xloc, xmin, xmax) / rholoc;
	  // MT 2011-04-13
	  int modelNumber=het->get_modelNumber(type);
	  bool isCA=( (modelNumber==11) || (modelNumber==13) || (modelNumber==15));
	  double xinsert=(!isCA) ? xloc : (int)(xloc+0.5);
	  double vinsert=(!isCA) ? vloc : (int)(vloc+0.5);
	  if(modelNumber==15){
	    double lVehAdhoc=7.5; // hack!!
	    xinsert=lVehAdhoc* ((int)(xloc/lVehAdhoc+0.5));
	    vinsert=lVehAdhoc* ((int)(vloc/lVehAdhoc+0.5));
	  }
	  veh[imax] = Vehicle(xinsert, vinsert, proj, het->get_fluct(type),
			      het->new_pmodel(type), modelNumber,
			      het->get_setNumber(type));
	  // end MT 2011-04-13
	  imax++;
	  xloc -= 1./ rholoc;
	  //cout <<"xloc="<<xloc<<endl;
	}
      imax--; // because imax imcremented after loop control

      // #############################################################
      // MT-2024: New correction for discretisation errors in macro-IC
      // to avoid stitching artifacts if there shoul be none
      // #############################################################
      
      bool isRing=(proj->get_choice_BCup()==3)
	&&(proj->get_choice_BCdown()==3);
      
      if(isRing&&(rhoinit[nxMacro-1]==rhoinit[0])){
	cout <<"\n\n====================================\n"
	     <<"RoadSection.initialize, periodic BC,"
	     <<" same density at stitching ends: In correction ...\n";
	double dxlast=veh[imax-1].getPos()-veh[imax].getPos();
	double dxfirst=veh[0].getPos()-veh[min(1,imax)].getPos();
	double dxstitch=xmax-veh[0].getPos()+veh[imax].getPos();
	double randomNumber= myRand();
	int type           = het->get_type(randomNumber);
	int modelNumber=het->get_modelNumber(type);

	cout<<"before correction: xfirst="<<veh[0].getPos()
	    <<" xlast="<<veh[imax].getPos()<<endl;
	cout<<"dxfirst="<<dxfirst<<" dxlast="<<dxlast
	    <<" dxstitch="<<dxstitch<<endl;

	if(dxstitch>0){// should be essentially zero; first=last car

	  // For some F... reason this is needed although I do not know why
	  if(true){
	    imax++;
	    veh[imax] = Vehicle(veh[imax-1].getPos()-dxlast,
			      veh[imax-1].getVel(),
			      proj, het->get_fluct(type),
			      het->new_pmodel(type), modelNumber,
			      het->get_setNumber(type));
	  }

	  cout<<"Before vehicle shift:"
	    <<" veh[0].getPos()="<<veh[0].getPos() <<endl
	    <<" veh[imax].getPos()="<<veh[imax].getPos() <<endl;
	  dxstitch=xmax-veh[0].getPos()+veh[imax].getPos();
	  double xminveh=veh[imax].getPos();
	  if(xminveh>0){// should be very slightly negative
	    //if(false){
	    for(int i=1; i<imax+1; i++){
	      veh[i].setPos(veh[i].getPos()-i/imax*xminveh*1.00001);
	    }
	  }
	  
	}
	cout<<"After stitch mismatch correction:"
	    <<" veh[0].getPos()="<<veh[0].getPos() <<endl
	    <<" veh[imax].getPos()="<<veh[imax].getPos() <<endl;
	  
	cout<<endl;
	
      }// end correction stitching artifacts
    }

  // #############################################################
  // or initialize with micro IC
  // #############################################################

  else if(isMicroIC)
    {
      // Microscopic IC
      int    n_vehinit;             // Number of positioned vehicles
      double posIC[NBC], vIC[NBC];  // Positions and corresp. initial densities
      int typeIC[NBC];              // corresp. types within model
      
      char in_fname[199];
      InOut inout;
      sprintf(in_fname,"%s.ICsingle",projectName);
      inout.get_array(in_fname, n_vehinit, posIC, vIC);
      inout.get_col(in_fname, 3, n_vehinit, typeIC);
      
      imin=0;
      imax=n_vehinit-1;  
      for (int iveh=0; iveh<=imax; iveh++)
	{
	  double x          = posIC[imax-iveh];
	  double v          = vIC[imax-iveh];
	  int type          = typeIC[imax-iveh];
	  //bool hasReactTime = (het->get_modelNumber(type)==2 || het->get_modelNumber(type)==3); //also OVM
	  
	  if(type>het->get_ntypes()){
	      cerr<<" RoadSection initialize: type="<<type
		  <<" larger than number "<<het->get_ntypes()
		  <<" of data lines in .heterog file!\n";
	      exit(-1);
	  }

	  if(het->get_modelNumber(type)==15){
	    double lVehAdhoc=7.5; // hack!!
	    x=lVehAdhoc* ((int)(x/lVehAdhoc+0.5));
	    v=lVehAdhoc* ((int)(v/lVehAdhoc+0.5));
	  }

	  
	  veh[iveh] = Vehicle(x, v, proj, het->get_fluct(type),
			      het->new_pmodel(type), 
			      het->get_modelNumber(type),
			      het->get_setNumber(type));
	}
    }



  // ###############################################
  //
  // cyclic buffer for present and past states (arne feb 06)
  // (1) without reaction times --> constructor without cyclic buffer (fastest)
  // (2) with fixed Treact --> cstr with precalculated past state (fast, old HDM implementation)
  // (3) with distributed Treact --> slowest version of cyclic buffer 
  //
  // choice is based on vehicles on road as representatives for heterogen
  // #################################################
    
  double T_react=0; 
  bool with_distributed_T_react=false;

  for (int iveh=imin; iveh<=imax; iveh++)
    {
      double individual_T_react = veh[iveh].p_model->get_T_react();
      if(false){
	cout<<"iveh = "<<iveh<<"  T_react = "<<individual_T_react<<endl;
      }
      if(individual_T_react > SMALL_VAL)
	{ 
	  T_react = individual_T_react;
	  if(veh[iveh].p_model->with_distributed_T_react())
	    {
	      with_distributed_T_react=true;
	    }
	}
    }
  
  bool someVehHaveReact = (T_react>SMALL_VAL);
  //check for ALL 
  // with_distributed_T_react = het->HDM_with_distributed_T_react();

  if(someVehHaveReact)
    { 
      if(with_distributed_T_react)
	{
	  cout<<"RoadSection: some have distributed reaction times !"<<endl;
	  cyclicBuf = new CyclicBuffer(dt, veh, imin, imax);
	}
      else
	{	 
	  cout<<"RoadSection: all vehicles have an identical reaction time !"<<endl;
	  cyclicBuf = new CyclicBuffer(T_react, dt, veh, imin, imax);
	}
    }
  else
    {
      cout<<"RoadSection: no HDM with reaction time --> create lean cyclic buffer dt="<<proj->get_dt()<<endl;
      cyclicBuf = new CyclicBuffer(veh, imin, imax);
    }


  // #####################################################
  // Adapt for periodic boundary conditions (cf. ProjectParams.cc)
  // ####################################################

  // (e.g. xmax=10000, rho=0.01 => 1st veh at 0, last at 10000)
  // (see also Projectparams Cstr.)
  
  if(proj->get_choice_BCup()==3)
    {
      //imax--; //!! <martin jun08> commented out 
    }

  // #####################################################
  // Test and Check result
  // #####################################################
  
  if( (!isMacroIC) && (!isMicroIC) )
    {
      cerr<<"Sorry, choice_init="<<choice_init
	  <<"; only choice_init=0,1,2,3 implemented"<<endl;
      exit(-1);
    }


  if(testinit){
  //if(true){
      cout << endl << endl
	   << "RoadSection Cstr: After initialization: " 
	   << endl;
      cout << "imax = " << imax <<" Inserted vehicles:"<< endl;
      for (int iveh=imin; iveh<=imax; iveh++)
	{
	  cout <<" iveh= "<<iveh
	       <<"      x= "<<veh[iveh].getPos()
	       <<"      v= "<<veh[iveh].getVel()
	       <<" modelnumber= "<<veh[iveh].getModelNumber()
	       <<" p_model->get_modelNumber()= "
	       <<veh[iveh].p_model->get_modelNumber() // should be the same
	       <<" length="<< veh[iveh].p_model->get_length()
	       <<" T_react="<<veh[iveh].p_model->get_T_react()
	       <<endl;
	}
      exit(0);
    }
} // end RoadSection::initialize


//###################################################################
/* MT apr13: Tests different numerical update schemes: 
  (1) updateEuler(int it)
  (2) updateBallistic(int it) (standard method, here just for code testing)
  (3) updateTrapezoid(int it)
  (4) updateRK4(int it)
  (2*) update(int it) Standard default with full functionality

  All except (2*) sort of quick-hack! 

  Work only 
    - w/o sources/sinks (choice_BC==2, then boundary_up->updateUpstream 
      and boundary_down->updateDownstream does nothing 
      => point (1) of main update(int it) method  not needed
    - w/o memory (CyclicBuffer::updateBuffer does nothing => not needed)
    - w/o ramps (no merges, update lane changes => points (3), (4a) not needed,
    - w/o external dropping/lifting =>  point (3a) not needed,
    - w/o external control => points (4b), (4e) not needed,
      => to include, ctrl has to be called at the intermediate 
      times of the relevant steps! (begin, middle, end)
      => change setState() accordingly (has been partially done as of dec2014)
    - w/o region control => point (4c) not needed,
*/
//###################################################################

    // essential: check for v<0 (!!) (Trapezoid as Ballistic plus acc=0)
    //if(veh[i].getVel()<-20){
    //!!!!!!!!!!!!! Muss die Korrekturen anders machen!! Auch Unterschiede bei 
    //!!!!!!!!!!!!! method 2 vs original, wenigstens bei VW (nicht IDM)
    //!!!!!!!!!!!!! Test andere Modelle ohne Beschleunigung als exog. Var!


    // essential: check for v<0 (!!)

    // IDM_startStopLSA, dt=0.2, Euler:
    //     just set v=0, x+=0 if v<0

    // IDM_startStopLSA, dt=0.2, Ballistic:
    //     Rule (d) of Trapezoid with acc=k1 and min condition of (e) 
    //     leads to identical output (tested for cars 1, 10)

    // IDM_startStopLSA, dt=0.2, Trapezoid:
    // (a) if(false): error if v<0  applies for the first time
    // (b) x -= 0.5*acc*SQR(dt): artifacts since calc. is not correct!
    // (c) x+=0: OK
    // (d) x -= 0.5*v_old^2/acc with acc=k1, 0.5*(k1+k2) or k2: OK
    //     but only 0.5*(k1+k2) is strictly consistent with condition
    // (e) x -= 0.5*v_old^2/acc with acc=min(0.5*(k1+k2), -1e-8): OK and safer
    //     (acc should be <0 since v<0 for the first time;
    //     nevertheless, both v and acc can be minute)

    // IDM_startStopLSA, dt=0.2, RK4:
    //     artifact at begin of braking (before v<0 applies!)
    //     explained! Input not continuous for car >=2 since front vehicle stops !! 
    //     (continuous for car 1, convergence confirmed!)
    // (e) with acc=min(0.5*(k1[j]+2*k2[j]+2*k3[j]+k4[j])/6, -1e-8) consistent+OK

    // IIDM, IDMM, FVDM, PTmodel OK, 
    // ACC differences between Ballistic and default, HDM not OK (is explained)!

    // if setAcc(0) not done if v<0, ACC OK and RK4 with less artifacts
    // but plot artifacts when plotting accel. for this case

    // new-delete[] mechanism would take factor 4 of time

    // cyclicBuf->updatePresence():
    //  information flow only vehicles->cyclicBuffer 
    // => either begin or end at Euler and ballistic



//###################################################################
  // calculates right-hand side k1[], k2[] etc based on the state vector y
  // MT mai13
void RoadSection::calc_rhs(int it, double alpha_v0_global, double alpha_T_global,
			   double rhs[]){
//###################################################################

  // if acc depends on old acc: pos, vel and acc in cyclicBuf
  // need to be consistent! (not the case now)

  // update cyclicBuf: vehicles (old) Pos, Vel and acc ->  cyclicBuf
  // update Acc of vehicles: veh.calcAcc (cyclicBuf + vehicle -> new acc)
  // update Pos, Vel of vehicles (the latter by calcAcc) 


  cyclicBuf->updatePresence(it, veh, imin, imax);
  for(int i=imin+1; i<=imax; i++){
    veh[i].calcAcc(it, i, imin, imax, alpha_v0_global, alpha_T_global, cyclicBuf);
  }
  for(int i=imin; i<imax+1; i++){
    rhs[i-imin] = veh[i].getVel();
  }
  rhs[imax+1-imin]=0; // acceleration of first vehicle = 0
  for(int i=imin+1; i<imax+1; i++){
    rhs[i-imin+imax+1]=veh[i].getAcc();
  }
}


//###################################################################
void RoadSection::setState(const double ynew[], int it){
//###################################################################

  for (int i=imin; i<imax+1; i++){
    veh[i].setPos(ynew[i-imin]);
    veh[i].setVel(ynew[i-imin+imax+1]);

    //!!! not yet correctly implemented 
    // => with ext ctrl nonautonomous equation!
    // need new parameter set (const double ynew[], int it, double frac)
    // or (const double ynew[], double it)=>better since getVel takes double!!
    // must just control the calling inside updateRK4 etc!

    if(externalCtrlExists){
      for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++){
	int iveh=i_vehControlled[iCtrl];
	if( (iveh>=imin)&&(iveh<=imax)
	    &&(externalControl[iCtrl]->ctrlByVel()) ){
	  double vel=externalControl[iCtrl]->getVel(tstart+it*dt); //!!!mar15
	  veh[iveh].setVel(vel);
	  cyclicBuf->set_v(iveh, vel);
	}
      }
    }

  }
}


//###################################################################
void RoadSection::checkCorrectNegativeSpeeds(const double ynew[], 
					     const double yold[], double dtOldNew){
//###################################################################

// if v<0 corrected after every step, RK4 (and also Trapezoid) gets much better
// even other scaling!  discuss in paper!

  for (int i=imin; i<imax+1; i++){
    double vnew=ynew[i-imin+imax+1]; // == veh[i].getVel() since setState(.) called before
    if(vnew<0){
      double vold=yold[i-imin+imax+1];
      double acc=min((vnew-vold)/dtOldNew, -1e-8); // catch zero denominator
      veh[i].setVel(0);
      veh[i].setPos(yold[i-imin] -0.5 * SQR(vold)/acc);
    }
  }
}


//###################################################################
void RoadSection::updateEuler(int it){  // Test: pure Euler update
//###################################################################

  // optional (2b) Check for inconsistencies

  double alpha_v0_global=1;
  double alpha_T_global=1;

  double y[2*NVEHMAX];  // initial state vector (x,v)
  double ynew[2*NVEHMAX]; // state vector (x,v) after complete integr. step
  double k1[2*NVEHMAX]; // first calculation of rhs

  // get y

  for(int i=imin; i<imax+1; i++){
    y[i-imin]       =veh[i].getPos();
    y[i-imin+imax+1]=veh[i].getVel();
  }


  //####################################################

  // k1

  calc_rhs(it,alpha_v0_global,alpha_T_global,k1);
  for (int i=0; i<2*(imax-imin+1); i++){ 
    ynew[i]=y[i]+dt*k1[i];
  }
  setState(ynew,it);
  checkCorrectNegativeSpeeds(ynew,y,dt);

}
//####################################################


//###################################################################
void RoadSection::updateBallistic(int it){ 
 // Code check: should correspond to standard update RoadSection::update(it)
//###################################################################

  // optional (2b) Check for inconsistencies

  double alpha_v0_global=1;
  double alpha_T_global=1;

  double y[2*NVEHMAX];  // initial state vector (x,v)
  double ynew[2*NVEHMAX]; // state vector (x,v) after complete integr. step
  double k1[2*NVEHMAX]; // 1. calculation of rhs

  // get y

  for(int i=imin; i<imax+1; i++){
    y[i-imin]       =veh[i].getPos();
    y[i-imin+imax+1]=veh[i].getVel();
  }

  // k1 (incl quadratic term for x

  calc_rhs(it,alpha_v0_global,alpha_T_global,k1);
  for (int i=0; i<2*(imax-imin+1); i++){ 
    ynew[i]=y[i]+k1[i]*dt
      + ((i<(imax-imin+1)) ? 0.5*k1[i+imax-imin+1]*SQR(dt): 0);
  }
  setState(ynew,it);
  checkCorrectNegativeSpeeds(ynew,y,dt);
}



//###################################################################
void RoadSection::updateTrapezoid(int it){  // Test: full 2nd order Trapezoid rule
//###################################################################

  double alpha_v0_global=1;
  double alpha_T_global=1;

  double y[2*NVEHMAX];  // initial state vector (x,v)
  double ynew[2*NVEHMAX]; // state vector (x,v) after complete integr. step
  double k1[2*NVEHMAX]; // 1. calculation of rhs
  double k2[2*NVEHMAX]; // 2. calculation of rhs

  // get y

  for(int i=imin; i<imax+1; i++){
    y[i-imin]       =veh[i].getPos();
    y[i-imin+imax+1]=veh[i].getVel();
  }


  // k1; cyclicBuf->updatePresence at beginning of calc_rhs" veh->buf

  calc_rhs(it,alpha_v0_global,alpha_T_global,k1);
  for (int i=0; i<2*(imax-imin+1); i++){ 
    ynew[i]=y[i]+dt*k1[i];
  }
  setState(ynew,it); // y->veh
  checkCorrectNegativeSpeeds(ynew,y,dt);

  // k2

  calc_rhs(it,alpha_v0_global,alpha_T_global,k2);
  for (int i=0; i<2*(imax-imin+1); i++){
    ynew[i]=y[i]+0.5*dt*(k1[i]+k2[i]);
  }
  setState(ynew,it); //!!!
  checkCorrectNegativeSpeeds(ynew,y,dt);
}
//####################################################



//###################################################################
void RoadSection::updateRK4(int it){  // Test: standard fourth-order Runge-Kutta method
//###################################################################


  double alpha_v0_global=1;
  double alpha_T_global=1;

  double y[2*NVEHMAX];  // initial state vector (x,v)
  double ynew[2*NVEHMAX]; // state vector (x,v) after complete integr. step
  double k1[2*NVEHMAX]; // 1. calculation of rhs
  double k2[2*NVEHMAX]; // 2. calculation of rhs
  double k3[2*NVEHMAX]; // 3. calculation of rhs
  double k4[2*NVEHMAX]; // 4. calculation of rhs

  // get y

  for(int i=imin; i<imax+1; i++){
    y[i-imin]       =veh[i].getPos();
    y[i-imin+imax+1]=veh[i].getVel();
  }

  // k1

  calc_rhs(it,alpha_v0_global,alpha_T_global,k1); //uses veh+cyclicBuf, not y
  for (int i=0; i<2*(imax-imin+1); i++){ 
    ynew[i]=y[i]+0.5*k1[i]*dt;
  }
  setState(ynew,it); // sets veh+cyclicBuf to new state using ynew
                     // (not consistent for ext ctrl because then 
                     // dtNewOld matters which may be =0.5*dt)
  checkCorrectNegativeSpeeds(ynew,y,0.5*dt); // corr. veh+cyclicBuf to new state
                                             // leaves ynew unchanged!
 
 //!! possibly reformulate only using y,ynew changing ynew:

  // calc_rhs(it,alpha_v0_global,alpha_T_global,k1);
  // for (int i=0; i<2*(imax-imin+1); i++){ ynew[i]=y[i]+0.5*k1[i]*dt;}
  // checkCorrectNegativeSpeeds(ynew,y,0.5*dt);
  // setState(ynew,it);



  // k2

  calc_rhs(it,alpha_v0_global,alpha_T_global,k2); //uses veh+cyclicBuf, not y
  for (int i=0; i<2*(imax-imin+1); i++){
    ynew[i]=y[i]+0.5*k2[i]*dt;
  }
  setState(ynew,it);
  checkCorrectNegativeSpeeds(ynew,y,0.5*dt);

  // k3

  calc_rhs(it,alpha_v0_global,alpha_T_global,k3);
  for (int i=0; i<2*(imax-imin+1); i++){
    ynew[i]=y[i]+k3[i]*dt;
  }
  setState(ynew,it);
  checkCorrectNegativeSpeeds(ynew,y,dt);

  // k4

  calc_rhs(it,alpha_v0_global,alpha_T_global,k4);
  for (int i=0; i<2*(imax-imin+1); i++){
    ynew[i]=y[i]+dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
  }
  setState(ynew,it);
  checkCorrectNegativeSpeeds(ynew,y,dt);
}



//###################################################################
void RoadSection::update(int it){ // standard ballistic update
//###################################################################

  //cout <<"in RoadSection.update before section 0: it="<<it<<endl;
  double time=tstart+it*dt;
  //############################################################
  // (0) Runtime measurement for tests 
  //############################################################

  TIME_MEASURE(
	       static int it_start=100;
	       static int it_end=static_cast<int>((proj->get_tend()-proj->get_tstart())/proj->get_dt());
	       static clock_t start;
	       static clock_t end;
	       if(it==it_start) start = clock();
	       if(it==it_end) 
		 {
		   end = clock();
		   cout<<"\n\n\tPERFORMANCE: start="<<start<<", end="<<end<<" of "<<CLOCKS_PER_SEC
		       <<" CLOCKS_PER_SEC"<< endl;
		   cout<<"\tFor it_start="<<it_start<<" till it_end="<<it_end<<" --> "
		       <<(end-start)/static_cast<double>(CLOCKS_PER_SEC)<<" s\n\n\n";
		 }
	       )


  if(it%ndtout_3D==0)
    {
      cout<<"RoadSection.update:"
	  <<" ndtout_3D="<<ndtout_3D
	  <<" t=" << (time)
	  <<" imin="  << imin
	  <<" imax="  << imax
	  <<" x[imin]="<<veh[imin].getPos()
	  <<" x[imax]="<<veh[imax].getPos()
	  <<endl;
    }

  //############################################################
  // (1) Calculate boundary conditions
  //############################################################

  bool check=false;

  // (1a) BCup

  if(check)
    { 
      cout <<"before (1a): veh[1].getVel()="<<veh[1].getVel()
	   <<" veh[2].getVel()="<<veh[2].getVel()<<endl;
    }

  int imaxOld=imax;
  
  //imin and imax are called by reference --> changed in boundary update!
  boundary_up->updateUpstream(veh, imin, imax, het, time);


  // initialize cyclic buffer for new vehicles
  if( imax>imaxOld )
    { 
      cyclicBuf->initializeBuffer(veh, imaxOld+1, imax);
    }
  

  // (1b) BCdown
  if(check)
    { 
      cout <<"before (1b): veh[1].getVel()="<<veh[1].getVel()
	   <<" veh[2].getVel()="<<veh[2].getVel()<<endl;
    }
  
  //imin and imax are called by reference --> changed in boundary update!
  boundary_down->updateDownstream(veh, imin, imax, time);

  //############################################################
  // (2a) update CyclicBuffer
  // - update of present states (also called after Merging)
  // - update of past states (only called once in an update)
  //############################################################

  if(check)
    { 
      cout <<"before (2): veh[1].getVel()="<<veh[1].getVel()
	   <<" veh[2].getVel()="<<veh[2].getVel()<<endl;
    }

  cyclicBuf->updatePresence(it, veh, imin, imax);
  cyclicBuf->updateBuffer(it, veh, imin, imax);

  if(false)
    {  
      cout<<"RoadSection: it="<<it<<" and it*dt="<<it*dt<<" and imin="<<imin<<" and imax="<<imax<<endl;
      cout<<" pos["<<imin<<"]="<<veh[imin].getPos()  
	  <<" v["<<imin<<"]="<<veh[imin].getVel()    
	  <<" acc["<<imin<<"]="<<veh[imin].getAcc()  
	  <<endl;     
      for (int j=imin+1; j<=imax; j++)
	{              
	  cout<<" pos["<<j<<"]="<<veh[j].getPos()  
	      <<" s["<<j<<"]="<<cyclicBuf->get_s(j)
	      <<" v["<<j<<"]="<<veh[j].getVel()    
	      <<" acc["<<j<<"]="<<veh[j].getAcc()  
	      <<endl;                              
	}                                              
    }                                                
                                       
  //############################################################
  // (2b) Check for inconsistencies, errors and crashes (order is OK; need s[]!)
  // Error flow: 
  //   if (acc*dt<-v) => acc=-v/dt in Vehicle.calcAcc  !!NIX in IDM etc!
  //   if (v<0)       => v=0       in Vehicle.updatePosVel
  //   if (s<=0)      => exit program with error message here!
  //############################################################
  
  for (int i=imin+1; i<=imax; i++){
      if(cyclicBuf->get_s(i)<0 ){
	counter_crashs++;
	cout <<"RoadSection.update: Crash of vehicle "
	     << i << " at t="<<it*dt<<"s, x = " << veh[i].getPos() << "m" 
	     << ", crash number "<<counter_crashs<<endl;
	if(false){
	  for (int j=static_cast<int>(max(imin,i-10)); 
	       j<=static_cast<int>(min(imax, i+10)); j++){

	      cout<<" modelNumber="<<veh[j].getModelNumber()
		  <<" setNumber="<<veh[j].getSetNumber()
		  <<" pos["<<j<<"]="<<veh[j].getPos()
		  <<" s["<<j<<"]="<<cyclicBuf->get_s(j)
		  <<" v["<<j<<"]="<<veh[j].getVel()
		  <<" acc["<<j<<"]="<<veh[j].getAcc()
		  <<endl;
	  }
	}

	//exit(-2); //CRASH_EXIT
        take_out(i); // MT apr14; crashed vehicles are taken out!
      }
  }


  //############################################################
  // (3) Perform merges/diverges from ramps
  //############################################################

  if(check)
    { 
      cout <<"(3) perform merges/diverges from ramps"<<endl;
    }
  
  // calculate possible merges/diverges (parallel update)
  //cout <<"n_ramps="<<n_ramps<<endl;

  for(int irmp = 0; irmp<n_ramps; irmp++)
    {
      ramp[irmp]->update(time, cyclicBuf->get_xp(), cyclicBuf->get_sp(), imin, imax);  
    }

  // do actual merges/diverges (parallel update)
  
  bool mergesDivergesHappened=false;
  for(int irmp = 0; irmp<n_ramps; irmp++)
    {
      // perform merging
      if( ramp[irmp]->mergeIsOK() )
	{
	  mergesDivergesHappened=true;
	  double xMerge=ramp[irmp]->getChangePos();
	  int iFront=ramp[irmp]->getFrontVehIndex();
	  if(false)
	    {
	      cout<<" ramp["<<irmp<<"]: it="<<it<<" Vehicle entering ..."<<endl
		  <<" xMerge="<<xMerge
		  <<" iFront="<<iFront
		  <<" xveh[iFront]="<<veh[iFront].getPos()
		  <<" s[iFront+1]="<<cyclicBuf->get_s(iFront+1)
		  <<endl;
	    }
	  set_in(xMerge,iFront);
	  counter_rmpveh++;
	}
      
      // perform diverging (presently not implemented!!!)
      
      else if(ramp[irmp]->divergeIsOK())
	{
	  mergesDivergesHappened=true;
	  int iDiverge=ramp[irmp]->getDivergeVehIndex();
	  cout<<" ramp["<<irmp<<"]: Vehicle leaving ..."<<endl
	      <<" iDiverge="<<iDiverge
	      <<" xveh[iDiverge]="<<veh[iDiverge].getPos()
	      <<endl;
	  take_out(iDiverge);
	  counter_rmpveh--;
	}
    }
  
  //############################################################
  // (3a) Perform external dropping/lifting of vehicles
  // (simulating externally controlled lane changes)
  //############################################################

  if(check)
    { 
      cout <<"(3a) perform external dropping/lifting"<<endl;
    }

  if(!vehDropLift->noVehDroppedOrLifted())
    {
      vehDropLift->update(veh, imin, imax, time);
    }



  //############################################################
  // (4) Calculate new accelerations
  //############################################################

  if(check)
    { 
      cout <<"(4) calculate new accelerations"<<endl;
    }

  // (4a) Calculate local neighbourhood parameters for calcAcc
  // update x, s only if lane changes to/from ramps happened

  if(mergesDivergesHappened)
    {
      cyclicBuf->updatePresence(it, veh, imin, imax);
      cyclicBuf->updateBuffer(it, veh, imin, imax);
      //MT mar07: Also past of cyclic buffer must be updated to take care of the
      // reordering after the lane change!
    }




  // (4b) if(externalCtrlExists)&&ctrlByVel()
  //  then set some velocities  externally (overriding the BC!)

  // if(externalCtrlExists)&&ctrlByJump()
  //  then simulate cutins/cutouts by externally moving vehs

  if(externalCtrlExists){
    for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++){
      int iveh=i_vehControlled[iCtrl];
      if( (iveh>=imin)&&(iveh<=imax) ){ // imax-imin+1 vehicles

	// speed control

	if(externalControl[iCtrl]->ctrlByVel()){
	  double vel=externalControl[iCtrl]->getVel(time);
	  veh[iveh].setVel(vel);
	  cyclicBuf->set_v(iveh, vel); //!!

	  // special treatment for jumps in the .Bosch format

	  if(externalControl[iCtrl]->newTargetDetected(time)){
	    double oldPos=veh[iveh].getPos();
	    double newPos=oldPos+externalControl[iCtrl]->getGapBackJump(time);
	    veh[iveh].setPos(newPos);	    
	    cout <<"newTargetDetected! oldPos="<<oldPos
		 <<" newPos="<<newPos<<endl;
	  }
	}

	// jump control (aug19)

	if(externalControl[iCtrl]->ctrlByJump()){
	  if(externalControl[iCtrl]->checkNewJump(time)){
	    veh[iveh].setVelJump(externalControl[iCtrl]->getVelJump());
	    veh[iveh].setAlphaV0(externalControl[iCtrl]->getAlphaV0Jump());
	    double laggap=externalControl[iCtrl]->getGapBackJump(time);
	    double vehlength=veh[iveh+1].getLength();
	    if(iveh<imax){
	      veh[iveh].setPos(veh[iveh+1].getPos()+laggap+vehlength);
	    }
	    if(true){
	      cout <<"RoadSection.update (4b): external jump ctrl: new jump!"
		   <<" t="<<time
	           <<" iveh="<<iveh
		   <<" new laggap="<<laggap
	           <<" speed="<<veh[iveh].getVel()
		   <<" alpha_v0="<<veh[iveh].getAlphaV0()
		   <<endl;
	    }

	  }

	}// jump control
	  
      }// iveh in range check
    }// iCtrl loop
  }//externalControlExists

  if(false){
      cout <<"after (4b): "
	   <<" imin="<<imin<<" veh[imin].getPos()="<<veh[imin].getPos()
	   <<" veh[1].getVel()="<<veh[1].getVel()
	   <<" veh[2].getVel()="<<veh[2].getVel()<<endl;
  }



  // #####################################
  // (4c) martin mai08 if(regionCtrlExists)
  //  then influence the vehicles externally in these regions
  // #####################################

  //if(false){
  if(regionCtrlExists){
    //cout <<"n_regionsControlled="<<n_regionsControlled<<endl;
    for (int iCtrl=0; iCtrl<n_regionsControlled; iCtrl++){
      //cout <<"\nin RoadSection: Updating region control "<<iCtrl<<" ..."<<endl;
      regionControl[iCtrl].update(veh, imin, imax, time);

      if(regionControl[iCtrl].singleVehControlled()){
	if(regionControl[iCtrl].singleVehJustFound()){
	   cout <<"RoadSection.update, in section 4c:"
		<<" new event-based single-vehicle control found!"
		<<" creating new instance of ExternalControl "
		<<" externalControl["<<n_vehControlled<<"] ..."<<endl;
	   if(n_vehControlled>=NRMPMAX){
	     cerr<<"Roadsection.update, at 4c: error: new instance of"
		 <<" ExternalControl has index "<<n_vehControlled
		 <<" above NRMPMAX"<<endl;
	     exit(-1);
	   }
	

	   
	   n_vehControlled++;
	   externalCtrlExists=true;
	   i_vehControlled[n_vehControlled-1]=regionControl[iCtrl].get_singleVehIndex();

	   externalControl[n_vehControlled-1] = 
	     new ExternalControl(projectName, 
			     iCtrl+1,
			     i_vehControlled[n_vehControlled-1]);
	   //exit(0);
	   
	}// singleVehJustFound

    
	else if(regionControl[iCtrl].singleVehControlEnded(it*dt)){
	  n_vehControlled--;
	  cout <<"singleVehControlEnded: new n_vehControlled="
	       <<n_vehControlled<<endl;
	  if(n_vehControlled==0){
	    externalCtrlExists=false;
	  }
	}
      }

    }
  }

  //##########################################
  // (4d) Calculate flow-conservative bottlenecks for calcAcc
  // and do accelerations (c,d in one loop)
  //##########################################

  // martin mai08: global alpha's now obsolete; now in RegionControl
  // Nevertheless kept for
  // backwards compatibility and Arne's calculation of distributed alpha's
  //
  // Notice: regionControl directly influences vehicle states => veh[i].calcAcc

  bool v0_is_xdep = (n_v0>0);
  bool T_is_xdep  = (n_T>0);
  //arne: profiling hat gezeigt dass diese schleife teuer ist !!!!!!!!!!!!!!!
  // martin: Kein Wunder: Hier finden ja nahezu alle Berechnungen statt !!!!!!!!!!!!

  
  for(int i=imin+1; i<=imax; i++){
    double alpha_v0_global = (v0_is_xdep) ? intpextp(x_v0, arr_alpha_v0, n_v0, veh[i].getPos()) : 1;
    double alpha_T_global  = (T_is_xdep)  ? intpextp(x_T,  arr_alpha_T,  n_T,  veh[i].getPos()) : 1;

    if(distrSpan_v0_T != 0.0){
	    alpha_v0_global *= alpha_v0_distr[i];
	    alpha_T_global  *= alpha_T_distr[i];
    }
  
    //###################################################
    //###################################################
    // (4d1) do calcAcc!!!
    //###################################################


    veh[i].calcAcc(it, i, imin, imax,
		   alpha_v0_global, alpha_T_global, cyclicBuf);

    //###################################################
  }
  
  if(proj->get_choice_BCup()!=3)
    {
      veh[imin].setAcc(0);
    }


  // (4e) if(externalCtrlExists)&&ctrlByAcc
  //  then influence some accelerations externally
  
  if(externalCtrlExists){
    for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++){
      int iveh=i_vehControlled[iCtrl];
      if( (iveh>=imin)&&(iveh<=imax)){

        if(externalControl[iCtrl]->ctrlByAcc()){
	  double acc = externalControl[iCtrl]->getAcc(time);
	  if(acc*dt<-veh[iveh].getVel()){
	    acc=-veh[iveh].getVel()/dt;
  	  }
	  veh[iveh].setAcc(acc);
	  if(false){
	    cout<<"RoadSection.update: external control for vehicle "
	    <<iveh<<": acc="<<acc<<endl;
	  }
        }
      }
    }
  }



  // (4f) MT mar15 //!!!
  // new correction to avoid crashes by speed control
  // in this case, normal model overrides

  // correction activates if Gipps model speed for b=bcrit, s0=0 and T=1 s 
  // is below actual speed (need absolute speed boundary generic for most models)
  // to make the override conservative, the parameters are set aggressive
  // !! works only for normal update(), not for updateBallistic etc

  bool avoidCrashesByCtrl=true;
  double bGipps=10; // critical Gipps model deceleration for activating correction
  double TGipps=1; // critical Gipps time gap/reaction time for activating correction
  if(avoidCrashesByCtrl){
    if(externalCtrlExists){
      for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++){
        int iveh=i_vehControlled[iCtrl];
        if( (iveh>=imin)&&(iveh<=imax)){
          if(externalControl[iCtrl]->ctrlByVel()){
	    double v=veh[iveh].getVel();
	    double vl=veh[iveh-1].getVel();
	    double gap=veh[iveh-1].getPos()-veh[iveh-1].getLength()-veh[iveh].getPos();
	    double vUpperGipps=-bGipps*TGipps
	      +sqrt(bGipps*bGipps*TGipps*TGipps+vl*vl+2*bGipps*gap);

	    if(v>vUpperGipps){// action
	      veh[iveh].setVel(vUpperGipps);  
	      cout <<"RoadSection.update, overriding ctrlByVel for crash avoidance:"
		   <<endl;
	      cout <<" t="<<it*dt<<" iveh="<<iveh<<" s="<<gap<<" v="<<v
		   <<" accModel="<<veh[iveh].getAcc()<<" vCorr="<< veh[iveh].getVel()<<endl;
	    }
	  }
          if(externalControl[iCtrl]->ctrlByAcc()){
	    // todo
	  }
        }
      }
    }
  }




  //######### !!! Test/test/check model acceleration ####################

  //if(it<=20){
  if(false){
      cout<<"\nRoadSection.update, it="<<it
	  <<": After calculating accelerations ...imin="<<imin<<" imax="<<imax<<"\n";
      
      //for (int i=imax-10; i<=imax; i++)
      for (int i=imin+1; i<=imax; i++){
         if(i<10){
	   // if((veh[i].getPos()>1000)&&(veh[i].getPos()<3000)){

                   cout 
		<< "xveh["<<i<<"]="<<veh[i].getPos()
		 //<< " xvehOld["<<i<<"]="<<cyclicBuf->get_x(i,it,Treact)
		<<" s["<<i<<"]="<<cyclicBuf->get_s(i)
		<<" v["<<i<<"]="<<veh[i].getVel()
		<<" dv["<<i<<"]="<<veh[i-1].getVel()-veh[i].getVel()
	                //<<" vvehOld["<<i<<"]="<<cyclicBuf->get_v(i,it,Treact)
		//<<" veh["<<i<<"].p_model->get_modelNumber()= "
		// <<veh[i].p_model->get_modelNumber()
		 <<" acc["<<i<<"]="<<veh[i].getAcc()
		 <<endl;
	    }
	}
    }
  


  //############################################################
  // (5) update velocities and positions
  //############################################################

  
  for (int i=imin; i<=imax; i++){
      veh[i].updatePosVel();
  }


  //############################################################
  // (6) update secondary calculations (fuel consumption etc)
  //############################################################

  //floatCars->update(veh, imin, imax); //actually empty method body!

  //arne: !!!!!!!!!!!!!!!!!!! 
  //das kostet zeit, da jedes fahrzeug einmal in eine datei appended wird!!!
  // muss noch optimiert werden wenn es wieder gebraucht wird ....
  if(trajLOS) trajLOS->update(time, veh, imin, imax);
    
}


// #############################################################
// Generate (intermediate) macro fields from macro IC
// #############################################################

void RoadSection::generateMacroFields(int choice_init, double rho[], 
				      double Q[], int nxMacro){
  double RHOMIN=0.0001;  // (veh/m)
  double VMAX=50;        // (m/s)
  MicroModel* p_referenceModel = het->new_pmodel(0);

  //cout <<"RoadSection::generateMacroFields: rhomax="
  //    <<p_referenceModel->get_rhomax()<<endl;
  
  InOut inout;
  double dxVirtual = (xmax-xmin) / (nxMacro);
  
  if(choice_init==1)
    {  
      cout <<"RoadSection::generateMacroFields:in choice_init==1: dxVirtual ="
	   <<dxVirtual <<endl;
      double wplus  = 32200./160.;  // L/160 in Eq( 10) with L=32.2 km
      double wminus = 32200./40.;   // L/40  in Eq( 10) with L=32.2 km
      double dw     = 32200./32.;   // 5L/16-11L/32 = dist between pos/neg hump
      double rho_init=proj->get_rho_init();
      double xrelCenter=proj->get_xrelCenter();
      double ampl_init=proj->get_ampl_init();
      double xCenter=xmin+(xmax-xmin)*xrelCenter;
      
      for(int ix=0; ix<=nxMacro;  ix++)
	{
	  double x=xmin+ix*dxVirtual-xCenter;
	  rho[ix] = rho_init 
	    + ampl_init / SQR( cosh( x/wplus))
	    - wplus/wminus * ampl_init / SQR( cosh( (x-dw)/wminus));
	  Q[ix] = rho[ix] * p_referenceModel->get_veq(rho[ix]); 
	  if(rho[ix]<RHOMIN){rho[ix]=RHOMIN;}
	  if(Q[ix]>VMAX*rho[ix]){Q[ix]=VMAX*rho[ix];}
	  if(false)
	    {
	      cout <<"ix="<<ix<<" x="<<x<<" rho="<<rho[ix]<<" Q="<<Q[ix]<<endl;
	    }
	}
    }
  
   // ############# Macroscopic IC from file 
  
  else if((choice_init==2)||(choice_init==3))
    {
      // rho from file; if choice_init==3, also v from file
      int    n_jumps;               // How many jumps in the initial conditions
      double pos[NBC], rhoIC[NBC];  // Positions and corresp. initial densities
      double QIC[NBC];
      
      // read in rhoIC, possibly QIC and convert to SI

      char in_fname[199];
      sprintf(in_fname,"%s.IC",projectName);
      inout.get_array(in_fname, n_jumps, pos, rhoIC);
      for(int i=0; i<n_jumps;  i++){rhoIC[i] /= 1000.;}
      if(choice_init==3){
        inout.get_col(in_fname, 3, n_jumps, QIC);
        for(int i=0; i<n_jumps;  i++){
	  QIC[i]/=3600.;
	  cout <<"QIC[i]="<<QIC[i]<<endl;
	}
      }
      // purge lines with zero entries of density (superfluous numbers)
      // martin mai08

      int purgecount=0;
      for(int i=n_jumps-1; i>=0;  i--){
	if(!(rhoIC[i]>=SMALL_VAL)){
	  purgecount++;
	  for (int j=i+1; j<n_jumps; j++){
	    pos[j-1]=pos[j];
	    rhoIC[j-1]=rhoIC[j];
	    QIC[j-1]=QIC[j];
	  }  
	}
      }
      n_jumps -= purgecount;

      //</martin>
     

      // initial density
      
      for(int ix=0; ix<=nxMacro;  ix++)
	{
	  rho[ix] = intpextp (pos,rhoIC,n_jumps, ix*dxVirtual);
	}

      // initial flow

      if(choice_init==2)
	{
	  for(int ix=0; ix<=nxMacro;  ix++){
	    Q[ix] = rho[ix] * p_referenceModel->get_veq(rho[ix]);
	  }
	}
      else
	{
	  for(int ix=0; ix<=nxMacro;  ix++)
	    {
	      Q[ix] = intpextp (pos, QIC, n_jumps, ix*dxVirtual);
	    }
	}
    }

  //cout<<" End RoadSection::generateMacroFields: nxMacro="<<nxMacro<<endl;
  
  if(false)
    {
      for (int ix=0; ix<=nxMacro;  ix++)
	{
	  cout <<"ix="<<ix
	       <<" x="<<(ix*dxVirtual)
	       <<" rho[ix]="<<rho[ix]
	       <<" Q[ix]="<<Q[ix]
	       <<endl; 
	}
    }
}



//###################################################################
//this routine is called from Main!
//
void RoadSection::writeAll(int it)
//###########################################################
{
  // microscopic: x,v of each vehicle!
  // output to "micdat" 
  // arne: geaendert von ndtout_2d
  //cout <<"in RoadSection.writeAll: it="<<it<<" ndtout_3D="<<ndtout_3D<<endl;
  if( proj->with_mic_output() && it%ndtout_3D==0 ){
    write_3Dmic(it, proj->get_dnout_3d()); // in write_3Dmic condition t>tmax
  }

  // time series of real macroscopic density ,v etc
  // write to "x*_macro"
  if( it%ndtout_2D==0 ) 
    {
      //if(it==0){cout <<"Writing initial micro data ..."<<endl;}
      //else{cout <<"Writing micro data ..."<<endl;}
      //cout <<"Writing micro data ..."<<endl;
      write_2Dmac(it);
    }
 
  //write to ".dat"
  if(  proj->with_mac_output() && it%ndtout_3D==0 )
    {
      if(it==0){cout <<"Writing initial macro 3D data ..."<<endl;}
      if (it>0){write_3Dmac (it);}
      if(it==0){ cout<<"after Writing initial macro 3D data \n";}
    }

  // also update of detectors !!!
  for (int idet=0; idet<n_det; idet++)
    {
      detector[idet]->update(it, veh, imin, imax);
    }

  if( floatCars->floatCarsExist() && (it%ndtout_FCD==0))
    { 
      floatCars->write(veh, imin, imax, it, dt*ndtout_FCD);
    }

  if(flowCtrlExists)
    { 
      //cout <<"Writing flow-ctrl data ..."<<endl;
    }

  if( proj->with_travelTime_output() && (it%ndtout_2D==0) )
    { 
      //cout <<"Writing travelling time ..."<<endl;
      write_TravTimes(it);
    }
  
 
  if(calcFuelConsumption && (it%ndtout_2D==0) ){
    write_FuelConsumption(it);
  }
} // writeAll


//###################################################################
void RoadSection::write_FuelConsumption(int it)
//###################################################################
{
  int gear;  // is reference to optimal gear in getMinFuelFlow

  static int it_prev_fuel;

  char   out_fname[199];
  double fuelRate_lInvs   = 0; // fuel consumption rate of all vehs (l/s)

  const double vmin      = 5; // minimal velocity (m/s) assumed for travtime
  double travtime_s = 0; // instantaneous travtime (s)

  if(it==0){
      accum_fuel_l=0;
      it_prev_fuel=0;
  }

  for (int i=imin+1; i<imax; i++){
      double dist = veh[i-1].getPos() - veh[i].getPos();
      double v=max(veh[i].getVel(), vmin);
      travtime_s += dist/max(v,vmin);
      double acc=veh[i].getAcc();
      double jerk=veh[i].getJerk();

      double fuelFlow=0; // BUG DOS if not defined directly but by "+="
      bool fixedScheme=true; //!!

      // fixed-gear scheme

      if(fixedScheme){
        fuelFlow=fuelConsumption->getFuelFlowFixedGearscheme
	  (v,acc,jerk,gear,useEngineDataSheet);
      }

      // optimal-gear scheme

      else{
	fuelFlow=fuelConsumption->getMinFuelFlow
	  (v,acc,jerk,gear,useEngineDataSheet);
      }

      fuelRate_lInvs += fuelFlow;


      // inner test

      if(false){
      //if(testFuelRoadSec(v)){



	cout <<"\nInner loop RoadSection.write_FuelConsumption: iveh="<<i
	     <<" v="<<v<<" acc="<<acc
	  // <<" jerk="<<jerk
             <<endl
	     <<" gear="<<gear<<" useEngineDataSheet="<<useEngineDataSheet
             <<" fixedScheme="<<fixedScheme<<endl
	     <<" fuelFlow="<<fuelFlow
	     <<endl;

      }
  }
  accum_fuel_l += fuelRate_lInvs*dt*(it-it_prev_fuel);

  // test

  if(false){
  //if(true){


    cout <<"RoadSection.write_FuelConsumption: it="<<it
         <<" it_prev_fuel="<<it_prev_fuel
         <<" fuelRate_lInvs="<<fuelRate_lInvs
         <<" accum_fuel_l="<<accum_fuel_l<<endl;
  }

  it_prev_fuel=it;

	
  sprintf(out_fname,"%s.fuel",projectName);
  
  if (FH_fuel==0){
			FH_fuel = fopen(out_fname,"w");
			filecheck(FH_fuel, out_fname);
      fprintf(FH_fuel, "# t(min) \t fuelRate(l/s) \t accum_fuel(l)\t ntot\t travtime(s)\n");
    }

  fprintf(FH_fuel, "%.3f\t\t  %.3f\t\t %.2f\t\t  %i\t%.2f\n", 
	  it*dt/60, fuelRate_lInvs, accum_fuel_l, imax-imin, travtime_s);
		
	fflush(FH_fuel);
  

} // end write_FuelConsumption


//###################################################################
void RoadSection::write_TravTimes(int it) // only called internally in this.writeAll(it)
//###################################################################
{
  static int it_prev_tt;
  char out_fname[199];
  const double vmin = 5; // minimal velocity (m/s) assumed for travtime
  double travtime_s = 0; // instantaneous travtime (s)
  
  if(it==0)
    {
      accum_travtime_s=0;
      it_prev_tt=0;
    }

  for (int i=imin+1; i<imax; i++)
    {
      double dist = veh[i-1].getPos() - veh[i].getPos();
      double v = veh[i].getVel();
      travtime_s += dist/max(v,vmin);
      //cout <<"i="<<i<<" travtime_s="<<travtime_s<<endl;
    }

  accum_travtime_s += (imax-imin)*dt*(it-it_prev_tt);
  it_prev_tt=it;
	
	//oct06: comfort now in tt output....
	static double tau_0 =  1.0; //weight factor for jerk in units of time (s)
	double accSqr, jerkSqr;
	calc_DrivingComfort(it, tau_0, accSqr, jerkSqr);
	//end oct06
	
  sprintf(out_fname,"%s.tt",projectName);
            
  if(FH_tt==0)
    {
			FH_tt  = fopen(out_fname,"w");
			filecheck(FH_tt,out_fname);
			
			//Oct06: comfort now in tt output!!!s
			fprintf(FH_tt, "# (instantaneous) AVERAGE comfort in units of squared acceleration:  (m/s^2)^2\n");
      fprintf(FH_tt, "# accumulated comfort (NOT AVERAGED) in units of time*squared acceleration:  s*(m/s^2)^2\n");
      fprintf(FH_tt, "# weight factor for acceleration and jerk tau_0 = %.3f\n", tau_0);
			fprintf(FH_tt, "# WARNING: Comfort related quantities are still experimental (oct06)\n");
			
      fprintf(FH_tt,"#t(min)\tITT(min)\taccumTT(min)\tntot\tactual comfort\tact jerk\taccum_comfort\taccum_jerk\n");
    }
    
  fprintf(FH_tt, "%6.1f\t%6.1f\t\t%6.1f\t\t%4i\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n", 
	  it*dt/60, travtime_s/60, accum_travtime_s/60, imax-imin, accSqr, jerkSqr, accum_acc, accum_jerk);
		
	fflush(FH_tt);

} // end write_TravTimes

//###################################################################
//
// ARNE: EXPERIMENTAL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//comfort calculated from acceleration and jerk
//parameter: tau_0: weight factor for jerk!!
//
void RoadSection::calc_DrivingComfort(int it, double tau_0, double& accSqr, double& jerkSqr)
//###################################################################
{
 
  // static double accum_comfort; //accumulated comfort
  static int it_prev_cmf;

  //init static variables
  if(it==0)
    {
      accum_acc=0;
      accum_jerk=0;
      it_prev_cmf=0;
    }

  //  double comfort = 0; //actual comfort 
  accSqr=0;
  jerkSqr=0;
  
  int n_ignore=0;
  for(int i=imin; i<imax; i++)
    {
      //ignore vehicles in ramp section
      //droping mechanism induces to much perturbation!!!!!
      //extend length because vehicles are affected outside the merging zone!
      //works also for no ramp!
      
      bool ignore_veh=false; 
      for (int irmp=0; irmp<n_ramps; irmp++){
	if( fabs( x_ramp[irmp]-veh[i].getPos()) < (l_ramp[irmp]+100.)/2. )
	  {
	    ignore_veh=true;
	    n_ignore++;
	    //cout<<"ignore veh at "<<veh[i].getPos()<<endl;
	  }
      }
      
      if(!ignore_veh)
	{
	  accSqr  += SQR(veh[i].getAcc());
	  jerkSqr += SQR(veh[i].getJerk());
	  if(false && (fabs(veh[i].getAcc())>10 || fabs(veh[i].getJerk())>10) ){
	    printf("\nit=%i\tt=%.2f for veh i=%i at x=%.2f: acc=%.5f\tjerk=%.5f\n",
		   it,(it*dt),i,veh[i].getPos(),veh[i].getAcc(),veh[i].getJerk());
	  }
	}
    }
  
  //DEBUGGING: ramp veh cause big jerk !!!
  //    if(false && (fabs(acc)>10 || fabs(jerk)>10)){
  //      printf("\nit=%i\tt=%.2f for veh i=%i at x=%.2f: acc=%.5f\tjerk=%.5f\n",it,(it*dt),i,veh[i].getPos(),acc,jerk);
  //    }
  //comfort += SQR(acc)+SQR(tau_0*jerk); //comfort is sum of squares
    
  //accululated is for whole sim, so one has to weight with absolute number of vehicles
 
  accSqr  /= (imax-imin-n_ignore);
  jerkSqr /= (imax-imin-n_ignore);

  accum_acc  += accSqr*dt*(it-it_prev_cmf); //time integration for whole output time interval
  accum_jerk += jerkSqr*dt*(it-it_prev_cmf); 
  
  //actual comfort for just one timestep is possible to normalize
  //comfort   /= (imax-imin);
  
  it_prev_cmf=it; //to remember
 
}

 

//###################################################################
void RoadSection::write_3Dmic(int it, int dnveh)
//###################################################################
{
  //cout <<"in RoadSection::write_3Dmic: t="<<it*dt<<endl;
  //double tmin=1800;   // start writing after t=tmin s //!!!
  double tmin=-1;  // start writing after t=tmin s //!!!

  char out_fname[199];

  sprintf(out_fname,"%s.micdat",projectName);
 
  if (FH_3dmic==0){
		FH_3dmic = fopen(out_fname,"w");
    fprintf(FH_3dmic,"# t(s)\t iveh\t x(m) \t v(m/s) \t s(m) \t a(m/s^2)\tSet number\t vehID\n");
  }

  // <MT mai 11>
  // offset= quick hack to get trajectories right if dnout>1;
  // only correct for locations upstream of the most upstream ramp!
  // Correct implementation: Make counter for each on- and offramp and,
  // to determine the value of the x-dependent offset, apply only these counters
  // that are connected to ramps which are more downstream than the actual
  // location x
  int offset=counter_rmpveh-dnveh*static_cast<int>(counter_rmpveh/dnveh);
  int iminout=offset+dnveh*(1+static_cast<int>(imin/dnveh));
  int nvehout=(static_cast<int>((imax-iminout)/dnveh));
  //cout<<"in RoadSection::write_3Dmic: counter_rmpveh="<<counter_rmpveh<<" offset="<<offset<<endl;

  if(false){
    cout<<" RoadSection.write_3Dmic: imin="<<imin<<" imax="<<imax
	  	<<" iminout="<<iminout
	  <<" nvehout="<<nvehout
	  <<endl;
  }

  if(it*(proj->get_dt())>=tmin){
      for (int j=0; j<=nvehout; j++){
	  int i=iminout+j*dnveh;
	  double x=veh[i].getPos();
	  double v=veh[i].getVel();
	  double s=veh[i-1].getPos()-veh[i-1].getLength()-x;
	  double a=veh[i].getAcc();
	  
	  fprintf(FH_3dmic,"%.1f\t %.i\t %.2f\t  %.3f\t %.2f\t %.3f \t%i \t%i\n",
		  it*dt, i, x, v, s, a, veh[i].getSetNumber(), veh[i].getID());
      }
      fflush(FH_3dmic);
       //fprintf(FH_3dmic, "\n");        // no newline here!
 
  }
  else{
     cout <<"RoadSection.write_3Dmic: Start writing only when t>tmin="<<tmin<<endl;
   }  
}


//###################################################################
void RoadSection::write_2Dmac(int it)
//###################################################################
{

  // determine macroscopic quantities

  static double rho[NVEHMAX];  // size NOT NXMAX

  for (int i=imin+1; i<=imax; i++)
  {
    rho[i] = (veh[i-1].getPos()-veh[i].getPos() > veh[i-1].getLength()) ? 
      1/(veh[i-1].getPos()-veh[i].getPos()) : 1./veh[i-1].getLength();
  }
  
  // write them at the virtual detector positions indicated by x_det[]

  for (int idet=0; idet<n_det; idet++)
    {
			if(it==0){
			 char out_fname[199];
       sprintf(out_fname,"%s.x%i_macro", projectName, static_cast<int>(x_det[idet]) );
			 FH_2dmac_vec.push_back(fopen(out_fname,"w"));
			 fprintf(FH_2dmac_vec.at(idet), "# x(km) \t t(h) \t rho(1/km) \t v(km/h) \t Q(1/h) \t a(m^2/s^2) \n");
			}
      
      double x_m       = x_det[idet];
      double t_h       = (tstart+it*dt)/3600.;
      double rho_invkm = intpextp(cyclicBuf->get_xp(),rho,x_m,imin+1,imax,true) * 1000.;
      double v_kmh     = intpextp(cyclicBuf->get_xp(),cyclicBuf->get_vp(),x_m,imin,imax,true) *3.6;
      double Q_invh    = rho_invkm*v_kmh;
      double a_ms2     = intpextp(cyclicBuf->get_xp(),cyclicBuf->get_ap(),x_m,imin,imax,true);
      
      fprintf(FH_2dmac_vec.at(idet), "%.6f\t  %.6f\t  %f\t %f\t %f\t %f\n", 0.001*x_m, t_h, rho_invkm, v_kmh, Q_invh, a_ms2);
      fflush(FH_2dmac_vec.at(idet));
    }
}  // write_2Dmac



//###################################################################
void RoadSection::write_3Dmac(int it)
//###################################################################
{
  char   out_fname[199];

  sprintf(out_fname,"%s.dat",projectName);

  if(FH_3dmac==0)  
    {
			FH_3dmac = fopen(out_fname,"w");
			filecheck(FH_3dmac, out_fname);
      fprintf(FH_3dmac,"# x(km) \t t(h) \t rho(1/km) \t v(km/h) \t Q(1/h) \t a(m/s^2) \n");
    }
  static double rho[NVEHMAX];  // size NOT NXMAX

  for (int i=imin+1; i<=imax; i++)
    {
      rho[i] = (veh[i-1].getPos()-veh[i].getPos() > veh[i-1].getLength()) ? 
	1/(veh[i-1].getPos()-veh[i].getPos()) : 1./veh[i-1].getLength();
    }

  for (int j=1; j<=nxout_3D; j++)
    {
      double x_m       = j*(proj->get_dxout_3d());
      double t_h       = (tstart+it*dt)/3600.;
      double rho_invkm = intpextp(cyclicBuf->get_xp(),rho,x_m,imin+1,imax,true) * 1000.;
      double v_kmh     = intpextp(cyclicBuf->get_xp(),cyclicBuf->get_vp(),x_m,imin,imax,true) *3.6;
      double Q_invh    = rho_invkm*v_kmh;
      double a_ms2     = intpextp(cyclicBuf->get_xp(),cyclicBuf->get_ap(),x_m,imin,imax,true);

      fprintf(FH_3dmac,  "%.6f\t  %.6f\t  %f\t %f\t %f\t %f\n",
	      0.001*x_m, t_h, rho_invkm, v_kmh, Q_invh, a_ms2);
    }

  fprintf(FH_3dmac, "\n");        // newline mean new t step for gnuplot
	fflush(FH_3dmac);
}



//########################################################################
void  RoadSection::get_flowconsInhomog(string varname, double x_tab[], 
				       double var_tab[], int& n_tab)
//########################################################################
{
   char in_fname[199];
   sprintf(in_fname,"%s.%s",projectName, varname.c_str());
   ifstream  infile (in_fname, ios::in);

   if( !infile)
     {
       n_tab     =0;
       x_tab[0]  =0;
       var_tab[0]=1;
     }
   else
     {
       InOut inout;
       inout.get_array (in_fname, n_tab, x_tab, var_tab);
       
       if(n_tab>NVEHMAX){
	 cerr<<" RoadSection::get_flowconsInhomog: length n_tab > NVEHMAX\n";
	 exit(-1);
       }
     }
}

//########################################################################
// MT 2015: complete control newly formulated
void  RoadSection::get_fueldata(string carName) 
//########################################################################
{
  char engineDataName[200];
  char carDataName[200];
  sprintf(engineDataName, "%s.engineData%s", projectName, carName.c_str());
  sprintf(carDataName,    "%s.carData%s",    projectName, carName.c_str());

  ifstream  infileCarData (carDataName, ios::in);
  ifstream  infileEngineData (engineDataName, ios::in);

  calcFuelConsumption=infileCarData.is_open();  // true if file exists
  useEngineDataSheet=infileEngineData.is_open(); // calc with tabulated engine map if file exists

  if(!infileCarData){
    cout<<"RoadSection.get_fueldata: No car data file "<<carDataName <<endl;
    cout<<"   => don't calculate fuel consumption"<<endl;
  }

  if(infileEngineData){
    cout<<"RoadSection.get_fueldata: found engine data sheet file "
	<<engineDataName <<endl
	<<"   => calculate fuel consumption with tabulated engine data sheet"<<endl;
  }
  else{
    cout<<"RoadSection.get_fueldata: found no engine data sheet file "
	<<engineDataName <<endl
        <<"   => calculate fuel consumption with analyticengine map"<<endl;
  }

  if(calcFuelConsumption){
    bool testSimpleFormula=true;
    bool testJanteFormula=true;
    fuelConsumption = new Consumption(proj->get_dt(),projectName,
				      engineDataName,carDataName, 
				      useEngineDataSheet,
				      testSimpleFormula,testJanteFormula);
  }

}


//########################################################################
void  RoadSection::get_ramps()
//########################################################################
{

  char in_fname[199];
  sprintf(in_fname,"%s.rmps",projectName);
  ifstream  infile (in_fname, ios::in);

  if( !infile){ 
    cout <<"RoadSection.getramps: no file "<<in_fname<<" => no ramps!\n";
    n_ramps=0;  // ramp index from 0 ... n_ramps-1
    //exit(0);
  }
  else{
    InOut inout;
    inout.get_array(in_fname, n_ramps, x_ramp, l_ramp);
    if (n_ramps > NRMPMAX){
      cerr<<"RoadSection::get_ramps: Error: number of ramps > NRMPMAX\n";
      exit(-1);
    }

    for (int irmp=0; irmp<n_ramps; irmp++){
      ramp[irmp] = new Ramp(projectName, proj, irmp+1,x_ramp[irmp], l_ramp[irmp]);
    }
  }
}

//########################################################################
void  RoadSection::get_detectors()
//########################################################################
{
  int avgInterval=60; //!!
  char in_fname[199];

  sprintf(in_fname,"%s.detectors",projectName);
	
  ifstream infile (in_fname, ios::in);

  if(!infile){ 
    n_det=0;  //  from 0 ... n_det-1
  }
  else{
		infile.close();
    InOut inout;
    inout.get_array(in_fname, n_det, x_det);
    if (n_det > NDETMAX){
      cerr<<"RoadSection::get_detectors: Error: number of detectors"
	  <<" > NDETMAX\n";
      exit(-1);
    }

    for (int idet=0; idet< n_det; idet++){
      detector[idet] = new Detector(static_cast<int>(x_det[idet]), avgInterval, proj, projectName);
    }
  }
}


//#########################################################
void RoadSection::set_in(double pos, int leadVehIndex)
//#########################################################
{
  bool check=false;
  int icheckmin=leadVehIndex-2;
  int icheckmax=leadVehIndex+2;
  if(check)
    {
      cout<<"before RoadSection.set_in("<<pos<<", "<<leadVehIndex<<"):\n";
      for(int j=icheckmin; j<=icheckmax; j++)
	{
	  cout<<" pos["<<j<<"]="<<veh[j].getPos()
	      <<" v="<<veh[j].getVel() 
	      <<" length="<<veh[j].getLength()      
	      <<endl;
	}
    }

  // "push_back" all vehicles behind merging veh

  imax++;
  for (int i=imax; i>leadVehIndex; i--){  
    veh[i]=veh[i-1];
  }

  //cyclic buffer: must also tell cyclic buffer the reordering 
  //due to the change! see point (4a) of RoadSection.update (this file)


  // increment indices of controlled back vehicles, if any
  // martin sep07

  if(externalCtrlExists){
    for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++){
      if(i_vehControlled[iCtrl]>leadVehIndex){
	i_vehControlled[iCtrl]++;
      }
    }
  }


  //create new vehicle

  double randomNumber= myRand(); // martin mai07: benoetige modelNumber(type) bereits hier
  int type=het->get_type(randomNumber);
  //bool hasReactTime  = (het->get_modelNumber(type)==2);

  // martin mai07: geaendert; !!! RAMP_VEL_REDUCEFACTOR=0.5 (VDT); 0.5 .. 0.7 (sonst)
  double vlead=veh[leadVehIndex].getVel();
  double vcrit=10; /// unterhalb vcrit reducefactor allmahelich nicht mehr aktiv
  double reducefact=1. - (1-RAMP_VEL_REDUCEFACTOR)* min(1.,vlead/vcrit);
  // in constants.h
  
  //Vehicle newVehicle = Vehicle(pos, reducefact*veh[leadVehIndex].getVel(),
  veh[leadVehIndex+1] = Vehicle(pos, reducefact*veh[leadVehIndex].getVel(),
				proj, het->get_fluct(type),
				het->new_pmodel(type), 
				het->get_modelNumber(type),
				het->get_setNumber(type));
  
  // now new vehicle same properties as lead vehicle: move it now!
  //  veh[leadVehIndex+1]=newVehicle;

  if(false)
    {
      cout<<"after RoadSection.set_in("<<pos<<", "<<leadVehIndex<<"):\n";
      for(int j=icheckmin; j<=icheckmax; j++)
	{
	  cout<<" pos["<<j<<"]="<<veh[j].getPos()<<" length="<<veh[j].getLength()
	      <<endl;
	}
    }
}




//########################################################################
  void RoadSection::take_out(int vehIndex)
//########################################################################
{

  //"pull_back" vehicles behind ^= "push forward" vehicles in front

  imin++;
  for (int i=vehIndex; i>=imin; i--){
    veh[i]=veh[i-1]; 
  }

  // decrement indices of controlled vehicles in front, if any
  // martin sep07

  if(externalCtrlExists){
    for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++){
      if(i_vehControlled[iCtrl]<vehIndex){
	i_vehControlled[iCtrl]++;
      }
    }
  }

      //cyclic buffer: nothing changes for past states

}

//##############################################
void RoadSection::get_externalControl()
//##############################################
{

  char in_fname[199];
  sprintf(in_fname,"%s.leadvehs",projectName);
  ifstream  infile (in_fname, ios::in);

  if( !infile){
    n_vehControlled=0; 
    externalCtrlExists=false;
  }

  else{
    externalCtrlExists=true;
    InOut inout;
    inout.get_array(in_fname, n_vehControlled, i_vehControlled);
    if (n_vehControlled> NRMPMAX){
      cerr<<"RoadSection::get_externalControl: Error: "
	  <<" number of controlled veh > NRMPMAX\n";
      exit(-1);
    }

    for (int iCtrl=0; iCtrl<n_vehControlled; iCtrl++){
      externalControl[iCtrl] = new ExternalControl(projectName, iCtrl+1, i_vehControlled[iCtrl]);
      cout<<"RoadSection::get_externalControl: vehicle number "
 	  <<i_vehControlled[iCtrl]<<" Controlled!!\n";
    }
  }
}



//#########################################################
void RoadSection::get_regionControl()  // martin mai08
//#########################################################
{

  char in_fname[199];
  sprintf(in_fname,"%s.ctrlRegions",projectName);
  ifstream  infile (in_fname, ios::in);

  if( !infile){
    n_regionsControlled=-1;  // -1 because index starts from 0
    regionCtrlExists=false;
  }

  else{
    regionCtrlExists=true;
    InOut inout;
    
    int indextab[NRMPMAX];
    double tmintab[NRMPMAX];
    double tmaxtab[NRMPMAX];
    double xmintab[NRMPMAX];
    double xmaxtab[NRMPMAX];
    inout.get_col(in_fname, 1, n_regionsControlled,	 indextab);
    inout.get_col(in_fname, 2, n_regionsControlled, tmintab);
    inout.get_col(in_fname, 3, n_regionsControlled, tmaxtab);
    inout.get_col(in_fname, 4, n_regionsControlled, xmintab);
    inout.get_col(in_fname, 5, n_regionsControlled, xmaxtab);

    //n_regionsControlled++; //!!!martin apr08: Korrektur der Alten Indizierung 0 ... n in general.cpp. 

    //cout <<"n_regionsControlled="<<n_regionsControlled<<endl; exit(0)
    if (n_regionsControlled>= NRMPMAX){
      cerr<<"RoadSection::get_regionControl: Error: "
	  <<" number of controlled regions > NRMPMAX\n";
      exit(-1);
    }

    for (int iCtrl=0; iCtrl<n_regionsControlled; iCtrl++){

      regionControl[iCtrl] = RegionControl(projectName, *proj, indextab[iCtrl],
		      xmintab[iCtrl],xmaxtab[iCtrl],
		      tmintab[iCtrl],tmaxtab[iCtrl]);
      
      initVehIndex[iCtrl]=-1;
      // relevant only for special case local control triggeret by xmin,tmin

      if(true){
	cout<<"RoadSection::get_regionControl: region index= "
	    <<indextab[iCtrl]<<endl;
      }
    }
  }
}



//############################################################
// end of RoadSection 
//############################################################
