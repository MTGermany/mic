#ifndef VW_H
#define VW_H
#include <math.h>

#include "MicroModel.h"

class CyclicBuffer;

/// Implementiert den Beschleunigungsalgorithmus fuer VLA-Temporegler

/// Von der Basisklasse "MicroModel" sind fuer die Temporegler-Anwendung
/// nur die Fahrzeuglaenge "lveh" relevant.
/// Alle fuer die VLA-Anwendung relevanten Variablen und methoden sind 
/// in der HTML-Doku enthalten, der Rest nicht. 


class VW: public MicroModel
{

 public:
  
  VW(){;}
  VW(const char projectName[], double dt);

  //virtual ~VW(){;}

  /// Allg.  VLA-Beschleunigungsroutine (ruft die fuer den Temporegler relevante Routine accVLA auf)

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T, 
	     const CyclicBuffer* const cyclicBuf);

  
 private:

  /// VLA-Beschleunigungsfunktion fuer den Temporegler
  
    /// <ul>
    /// <li> Input (Kategorie 1 der "Anforderungen"): <ul>
    /// <li> Geschwindigkeit v, 
    /// <li> Nettoabstand s, 
    /// <li>Annaeherungsrate an Vorderfahrzeug dv, 
    /// <li>Beschl. des Vorderfahrzeugs a_lead,
    /// <li>Multiplikatoren alpha_T und alpha_v0 fuer 
    /// Folgezeit und Wunschgeschw.
    /// <li>Ob Test-Output ausgegeben werden soll
    /// </ul>
    /// <li> Output (Kategorie 2 der "Anforderungen")
    /// <ul><li> Beschlunigung des VLA-FAhrzeugs in m/s^2
    /// <li> Ggf. textueller Test-Output
    /// </ul>
    /// <li> Benoetigte Modellaprameter aus der Klasse VW:
    /// v0,T,delta,a,b,s0,s1,b_crit,bmax,jerkmax, coolnesss


   double accVLA(int it, int iveh, double v, double s, double dv, 
                double a_lead, double alpha_T, double alpha_v0,bool test_output);
   double accACC_IIDM(int it, int iveh, double v, double s, double dv, 
                double a_lead, double alpha_T, double alpha_v0);
   double accACC_IDMplus(int it, int iveh, double v, double s, double dv,
                double a_lead, double alpha_T, double alpha_v0);
   double accACC_IDM(int it, int iveh, double v, double s, double dv,
                double a_lead, double alpha_T, double alpha_v0);

   // helper function 
   double applyJerkControl(double accBefore);

  ///IDM parameter <br>

  /// Wunschgeschwindigkeit (m/s)
  double v0;  
  /// Folgezeit (s)
  double T;
  ///Minimaler Abstand (m)
  double s0;
  /// Abstandskomponente proportional sqrt(v) (einheitenlos, haeufig =0)
  double s1;
  /// Beschleunigungsexponent (einheitenlos)
  double delta;    // old: choice_A => now: VW acceleration exponent
  /// Max. Beschleunigung (m/s^2)
  double a;
  /// Komfortable Verzoegerung (m/s^2)
  double b;     
  /// Maximale Verzoegerung (m/s^2)
  double bmax;

 ///  Zusaetzl. VLA-Parameter (Kat. 3 und 4 der Anforderungen) <br>

  /// coolness factor: =1 fuer VLA  (0 fuer IDM,  <1 fuer hybrid)
  double cool;
  /// Maximaler Jerk in Normalsituationen (m/s^3)
  double jerk;   
  /// Kritische Verzoegerung (b<b_crit<bmax) (m/s^2) 
  double b_crit; 

  /// optional anticipation next-nearest leader (choice_variant=2)
  /// or choice base model: (hoice_variant=10: IDMplus, 11: IIDM)

  int choice_variant;

  /// Zustandsvariablen (Kat. 5 der Anforderungen)

  /// Zeit seit letzter Diskontinuitaet in den Input-Daten (s)
  double dt_since_new_leadveh; 

  /// Numerischer Zeitschritt (s)
  double dt;


  // Historische Groessen


  /// Beschl. im letzten Zeitschritt (m/s^2)
  double a_old; // necessary only for old accVLA method 

  /// Beschl. des Vorderfahrzeugs im letzten Zeitschritt (m/s^2) //neu!
  double a_lead_old; // necessary only for old accVLA method 

  /// Geschw. im letzten Zeitschritt (m/s)
  double v_old;

  /// Annaeherungsrate im letzten Zeitschritt (m/s)
  double dv_old;

  /// Abstand im letzten Zeitschritt (m)
  double s_old; // necessary for new object detection



  /// benoetigt fuer accVLA

  double maxsmooth(double x1, double x2, double dx)
  {
    return 0.5*(x1+x2) + sqrt(0.25*(x1-x2)*(x1-x2) + dx*dx);
  };

  double minsmooth(double x1, double x2, double dx)
  {
    return 0.5*(x1+x2) - sqrt(0.25*(x1-x2)*(x1-x2) + dx*dx);
  };

  void get_modelparams(const char* fname);
  void calc_eq();

};

#endif // VW_H

