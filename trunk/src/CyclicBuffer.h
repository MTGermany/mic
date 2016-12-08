#ifndef CYCLICBUFFER_H
#define CYCLICBUFFER_H

#include "constants.h"

class Vehicle; //forward declaration of class


//http://www.parashift.com/c++-faq-lite/const-correctness.html#faq-18.1
//What's the difference between "const Fred* p", "Fred* const p" and "const Fred* const p"?
//You have to read pointer declarations right-to-left. const Fred* p
//means "p points to a Fred that is const" - that is, the Fred
//object can't be changed via p.  
//Fred* const p means "p is a const
//pointer to a Fred" - that is, you can change the Fred object
//via p, but you can't change the pointer p itself.
//  const Fred* const p means "p is a const pointer to a const Fred" - that is, you
//can't change the pointer p itself, nor can you change the Fred
//object via p.


class CyclicBuffer
{ 
 public:

  //constructor for system without reaction times
  CyclicBuffer(const Vehicle* const veh, int imin, int imax);
  //constructor with fixed reaction times
  CyclicBuffer(double max_fix_T_react, double dt, 
	       const Vehicle* const veh, int imin, int imax);
  //constructor with distributed reaction times
  CyclicBuffer(double dt, const Vehicle* const veh, int imin, int imax);

  //deconstructor
  virtual ~CyclicBuffer();

  void initializeBuffer(const Vehicle* const veh, int imin, int imax);
 
  void updateBuffer(int it, const Vehicle* const veh, int imin, int imax);
  void updatePresence(int it, const Vehicle* const veh, int imin, int imax);

  //pointer needed for some methods (needed for ramp ...)
  double* get_xp(){return xVeh;}
  double* get_vp(){return vVeh;}
  double* get_ap(){return aVeh;}
  //double* get_aDesp(){return aVehDes;} //not yet implemented (2015-02)
  double* get_lp(){return lVeh;}
  double* get_sp(){return sVeh;}

  //access methods for current state without delay (inline)
  double get_x(const int& iveh) const { return( (iveh==iveh_with_x_error) ? xVeh[iveh]+err_x : xVeh[iveh] );}
  double get_v(const int& iveh) const { return( (iveh==iveh_with_v_error) ? vVeh[iveh]+err_v : vVeh[iveh] );}
  double get_a(const int& iveh) const { return aVeh[iveh];};
  //double get_aDes(const int& iveh) const { return aVehDes[iveh];};//not yet implemented (2015-02)
  double get_l(const int& iveh) const { return lVeh[iveh];};
  double get_s(const int& iveh) const { 
     if(false){cerr<<"CyclicBuffer:  iveh="<<iveh
		   <<" sVeh[iveh]="<<sVeh[iveh]<<endl;
     }
     return sVeh[iveh];
  }
 
  //access methods for past states
  //also possible: HDM with T'==0 (-->IDMM)
  double get_x(const int& iveh, const int& it, const double&  T_react) const; 
  double get_v(const int& iveh, const int& it, const double&  T_react) const; 
  double get_a(const int& iveh, const int& it, const double&  T_react) const;
  double get_l(const int& iveh, const int& it, const double&  T_react) const;
  double get_s(const int& iveh, const int& it, const double&  T_react) const;
 
  void set_v(const int& iveh, const double&  vel){ vVeh[iveh]=vel; }; //external control 

  //fluctuations in position and relative velocity
  void setError_x(const int& iveh, const double& err){ iveh_with_x_error=iveh; err_x=err; };
  void setError_v(const int& iveh, const double& err){ iveh_with_v_error=iveh; err_v=err; };

 private: 

  static const double MAX_T_REACT_S; //init. not allowed in header file

  double dt;
  int nt_past; //max number of columns for cyclic buffer

  bool someHaveReactionTime;
  double maxT_react; //max reaction time (limit for memory allocation)
  bool with_fixT_react;
  //  double fixT_react; //feature of old mic version ...

  int iveh_with_x_error; //vehicle index
  int iveh_with_v_error;
  double err_x;
  double err_v;

  //arrays for current state ("gegenwart")

  double* xVeh;    // positions
  double* vVeh;    // velocities
  double* aVeh;    // accelerations
  //double* aVehDes;    // desired accelerations //not yet implemented (2015-02)
  double* sVeh;    // net distances (for ramp)
  double* lVeh;    // vehicle lengths

  //arrays for fixed past state
  double* xOld;    // positions
  double* vOld;    // velocities
  double* aOld;    // accelerations
  double* lOld;    // vehicle lengths

  //matrices in C++ with fixed number of rows and flexible number of columns
  //cyclic buffer of past states

  //trennung von buffer und present state wg. diverser eingriffe in die gegenwart 
  //(fluct, merge, etc.)

  double** xBuffer;
  double** vBuffer;
  double** aBuffer;
  double** lBuffer;

  void init_buffer_matrix();
  void initialization(); 

  //should not be public ... included in update!!!
  double calcOldState(int iveh, int it, double T_react,  double* const* const past) const;
  void calcOldState(int imin, int imax, int it, double T_react, 
		    double* const* const past, double *old) const;

};

#endif // CYCLICBUFFER_H
