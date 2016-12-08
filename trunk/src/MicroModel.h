#ifndef MICROMODEL_H
#define MICROMODEL_H

#include "constants.h"
class CyclicBuffer;
class Vehicle;


// abstract base class representing a longitudinal vehicle model

class MicroModel  
{
 public:
  
  virtual ~MicroModel(){;}

  int get_modelNumber(){return modelNumber;} //!! new Test
  //arne (21.11.03): T_react only implemented in HDM
  //but variable defined in each model...
  double get_T_react(){return(T_react);}
	bool with_T_react(){return(T_react>SMALL_VAL);}
  
  bool with_distributed_T_react(){ return( T_react_span>0.001 );}

  double get_rhomax(){return rhomax;}
  double get_length(){return lveh;}
  void set_length(double newLen){lveh=newLen;}
  /// Gibt max. statischen Fluss des jeweiligen Modells aus
  double get_Qmax(){return Qmax;}
  double get_rhoQmax(){return rhoQmax;}
  /// errechnet Fundamentaldiagramm des jeweiligen Modells
  virtual double get_veq(double rho);
  void writeFunddia(const char funddia_name[]);

  virtual double acc(int it, int iveh, int imin, int imax,  
		     double alpha_v0, double alpha_T, 
		     const CyclicBuffer* const cyclicBuf)=0;

  virtual void calc_eq()=0;

  virtual void setSpeedlimit(double v0){speedlimit=v0;} //<martin mai08>


  //  static const int NRHO=50; //siehe constants.h
  double veqtab[NRHO+1];        // table of equilibrium velocity(rho)

  void initializeMicroModelVariables();

 protected:

  // Fahrzeuglaenge 
  //arne: muessen alle initalisiert werden!!!

  double lveh; 
  double rhomax;  // max. density = 1/lveh
  double Qmax;    // static capacity
  double rhoQmax; // equil. density at maximum flow
  double speedlimit; // martin mai08
  int modelNumber;  //!! Test!!

  double T_react; 
  double T_react_span;

  void calc_rhoQmax();
  virtual void get_modelparams(const char* fname)=0;
  double intp(const double tab[], int n, double x, double xmin, double xmax);
};

#endif // MICROMODEL_H

