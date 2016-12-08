#ifndef PROJ_PARAMS_H
#define PROJ_PARAMS_H

/// Projekt-Parameter wie Anfangs-, Endzeit, xmin, xmax, num. Parameter

class ProjectParams{ 

 public:

  ProjectParams();
  ProjectParams(const char* projectName);
  //~ProjectParams();

  char* get_projectName(){return projectName;}
  
  double distributionSpan_v0_T(){ return distr_v0_T;}

  bool with_travelTime_output(){ return( !traveltime_output==0 ); }
  //bool with_comfort_output(){ return( !comfort_output==0 ); }

  //{0=macro3D(dat), 1=microTraj(micdat), 2=both, 3=none}
  bool with_mac_output(){ return( choice_output==0 || choice_output==2 ); } 
  bool with_mic_output(){ return( choice_output==1 || choice_output==2 ); }
  bool with_trajectory_los(){ return( !trajectory_los==0 ); }

  //int get_choice_method(){ return choice_method; }
  int get_choice_BCup(){ return choice_BCup; }
  int get_choice_BCdown(){ return choice_BCdown; }
  int get_choice_init(){ return choice_init; }
  double get_rho_init(){ return rho_init; }
  double get_xrelCenter(){ return xrelCenter; };
  double get_ampl_init(){ return ampl_init; };


  double get_tstart(){ return tstart; }
  double get_tend(){ return tend; }
  
  double get_xmin(){ return xmin; }
  double get_xmax(){ return xmax; }

  double get_dt(){ return dt; }

  double get_dtout_FCD(){ return dtout_FCD; }
  double get_dt_tseries(){ return dt_tseries; }
  double get_dxout_3d(){ return dxout_3d; }
  double get_dtout_3d(){ return dtout_3d; }
  int get_dnout_3d(){ return dnout_3d; }

  int get_choice_output(){ return choice_output; }

 private:


  char projectName[199];

  /// Art der Simulation (noch nicht genutzt).
    //  int simType;
  ///0=IDM, 7=VW)
  //int choice_model;
  ///0=Euler, 1=Runge-Kutta
  //int choice_method; //oct06
  ///{0=Dirichlet, 1=Neumann, 2=zero, 3=period, 4=flowControl}
  int choice_BCup;
  ///{0=Dirichlet,1=Neumann,2=free,3=period,4=block}
  int choice_BCdown;
  ///2=macroKerner, 4=macroRho, 7=macroV, 5=micro
  int choice_init;
  ///Simulationsanfang (s)
  double tstart;
  ///Simulationsende (s)
  double tend;
  ///upstream-Ende der Simulation (m, ddg, Richtung in pos. x)
  double xmin; 
  ///downstream-Ende der Simulation
  double xmax; 

  ///Numerischer Zeitschritt (s)
  double dt;  


  //Output options:
  ///zeitliche Schrittweite (s) fuer FloatingCars
  double dtout_FCD;
  ///zeitliche Schrittweiten (s) fuer Fuel,TravelTime,Comfort,mac2D_output
  double dt_tseries; 
  ///Gitterweite (m) fuer die 3d-Ausgabe (micdat)
  double dxout_3d; 
  ///zeitliche Schrittweite (s) fuer 3d-Ausgabe (micdat)
  double dtout_3d;
  ///n'th vehicle fuer 3d-Ausgabe (micdat)
  int dnout_3d;

  //boolean flags:
  int traveltime_output;
  //int comfort_output;
  int choice_output;
  int trajectory_los;

  //weitere optionen:
  
  int random_seed; 
  double distr_v0_T; //oct06

  //

  double rho_init;
  double xrelCenter;
  double ampl_init;
  int debug;
  

};

#endif // PROJ_PARAMS_H
