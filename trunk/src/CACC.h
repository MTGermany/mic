#ifndef CACC_H
#define CACC_H
#include <math.h>

#include "MicroModel.h"

class CyclicBuffer;

/// Martin 2015-02-04 Cooperative Adaptive Cruise Control for COOL2 project

/// Von der Basisklasse "MicroModel" sind fuer die Temporegler-Anwendung
/// nur die Fahrzeuglaenge "lveh" relevant.


class CACC: public MicroModel
{

 public:
  
  CACC(){;}
  CACC(const char projectName[], double dt);

  //virtual ~CACC(){;}


  //#######################################################
  /// main acc method
  //#######################################################

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T, 
	     const CyclicBuffer* const cyclicBuf);

  
 private:

  //#######################################################
  /// main local CACC-acceleration function
  //#######################################################

    /// <ul>
    /// <li> Input (Kategorie 1 der "Anforderungen"): <ul>
    /// <li> Geschwindigkeit v, 
    /// <li> Nettoabstand s, 
    /// <li>Annaeherungsrate an Vorderfahrzeug dv, 
    /// <li>Beschl. des Vorderfahrzeugs a_lead,
    /// <li>Multiplikatoren alpha_T und alpha_v0 fuer 
    /// Folgezeit und Wunschgeschw.
    /// <li> Inputdaten des Leader-Fahrzeugs v_lead, dv_lead, a_lead2
    /// </ul>
    /// <li> Output (Kategorie 2 der "Anforderungen")
    /// <ul><li> Beschlunigung des VLA-FAhrzeugs in m/s^2
    /// <li> Ggf. textueller Test-Output
    /// </ul>
    /// <li> Benoetigte Modellaprameter aus der Klasse CACC:
    /// v0,T,delta,a,b,s0,s1,cool, jerk, b_crit, n_multi, tau_acc,tau_brake

  double accCACC(int it, int iveh, double v, double s, double dv, double a_lead, 
		  double v_lead, double s_lead, double dv_lead, double a_lead2,
		  double alpha_T, double alpha_v0);

  //#######################################################
  /// helper functions 
  //#######################################################

   // "desired" acceleration before jerk control, response etc
  double accDesired(double v, double s, double dv, 
                double a_lead, double alpha_T, double alpha_v0);

   // helper function for implementing vehicle response
  double applyVehicleResponse(double accBefore);

   // helper function for implementing jerk control
  double applyJerkControl(double accBefore);

   // helper functions for constructor
  void get_modelparams(const char* fname);
  void calc_eq();


  //#######################################################
  /// time step (imported global variable)
  //#######################################################
  double dt;


  //#######################################################
  // parameters
  //#######################################################


  ///IDM parameters <br>

  /// Wunschgeschwindigkeit (m/s)
  double v0;  
  /// Folgezeit (s)
  double T;
  ///Minimaler Abstand (m)
  double s0;
  /// Abstandskomponente proportional sqrt(v) (einheitenlos, haeufig =0)
  double s1;
  /// Beschleunigungsexponent (einheitenlos)
  double delta;    // old: choice_A => now: CACC acceleration exponent
  /// Max. Beschleunigung (m/s^2)
  double a;
  /// Komfortable Verzoegerung (m/s^2)
  double b;     
  /// Maximale Verzoegerung (m/s^2)
  double bmax;

 ///  ACC Parameters  <br>

  /// coolness factor: =1 fuer VLA  (0 fuer IDM,  <1 fuer hybrid)
  double cool;
  /// Maximaler Jerk in Normalsituationen (m/s^3)
  double jerk;   
  /// Kritische Verzoegerung (b<b_crit<bmax) (m/s^2) 
  double b_crit; 

  /// CACC and response parameters

  /// number of considered leaders (1 or 2)
  int n_multi;

  /// anticipation time [s]
  double tau_antic;

  /// response time acceleration [s]
  double tau_accel;

  /// response time deceleration [s]
  double tau_brake;

  //#######################################################
  /// State and history variables
  //#######################################################

  /// Zeit seit letzter Diskontinuitaet in den Input-Daten (s)
  double dt_since_new_leadveh; 

  /// old gap (necessary for new object detection)
  double s_old;

  /// speed last time step
  double v_old;

  /// leader's speed last time step
  double v_lead_old;

  /// approaching rate last time step
  double dv_old;

  /// leader's approaching rate last time step (CACC)
  double dv_lead_old;

  /// final acceleration in last time step (needed for jerk control and response)
  double a_old; 

  /// n-step PI control to simulate response
  static const int NSTEP=10;  
  double accPI[NSTEP];

  /// final leader acceleration in last time step (necessary?)
  //double a_lead_old; 



  //############################################################
  /// inline helper functions for mixing CAH and IIDM in accCACC function
  //############################################################

  double maxsmooth(double x1, double x2, double dx){
    return 0.5*(x1+x2) + sqrt(0.25*(x1-x2)*(x1-x2) + dx*dx);
  };

  double minsmooth(double x1, double x2, double dx){
    return 0.5*(x1+x2) - sqrt(0.25*(x1-x2)*(x1-x2) + dx*dx);
  };


};

#endif // CACC_H

