#ifndef FLUCT_H
#define FLUCT_H



/**
Three fluctuation sources:
<ul>
<li> delta-correlated acceleration fluctuations:
 dv/dt=a_det + xi, <xi>=0, <<xi(t)*xi(t')=Q_acc*delta(t-t')
<li> fluctuating correlated estimate errors of the relative velocity dv
     dv(est)=dv + s/ttc*xi_dv(t) with
     xi_dv(t)=(sigma=1, tau_dv) Wiener process
     1/ttc=inverse time-to collision as measure for std deviation
<li> fluctuating correlated estimate errors of distance
     s(est)=s*exp(varcoeff_s*xi_s(t))
     xi_s(t)=(sigma=1, tau_s) Wiener process
     varcoeff_s =  variation coefficient of distance error
</ul>
*/

#include "constants.h"

//!!! file <projName>.fluct1 belongs to first model in .heterog and so forth

class Fluct{

 public:

  Fluct();
  Fluct(const char* projectName, int itype, double dt);
  Fluct(double dt, double Q_acc, double varcoeff_s, double tau_s,
	double invttc_dv, double tau_dv);
  
  void update(double s, double v, double dv);

  bool isActive(){ return active; };

  double get_distance_error();

  double get_dv_error();

  double get_acc_error(){  return xi_a; };


  
 private:

  static const double SMALL_VAL_FLUCT; //init. in cpp file
  static const double sqrt12;
  void initialize();
  
  double dt;

  double Q_acc;      //fluct. strength (m^2/s^3) of dv/dt=a_det+xi(t) 
  double Q_dv;       //fluct. strength (1/s) of a Wiener (1,tau_dv) process
  double Q_s;        //fluct. strength (1/s) of a  Wiener (1,tau_s) process
  double varcoeff_s; // variation coeff (rel stdev) of error for gap estimate
  double tau_s;	     // relaxation time (s) of Wiener process for d(gap)/gap
  // double stddev_dv;  // standard deviation (m/s) of error of dv estimate
  double invttc_dv;  // std dev. of dv in units of 1/timeToColl (1/s)
  double tau_dv;     // relaxation time (s) of Wiener process for dv error


  
  bool active;


  double s;         // true NN distance; updated with update(...) method
  double v;         // true velocity; updated with update(...) method
  double dv;       //  true appr. rate; updated with update(s,dv) method

  double xi_s;      // (sigma=1, tau_s) Wiener process
  double xi_dv;     // (sigma=1, tau_dv) Wiener process
  double xi_a;      // delta-correlated random process, var=Q_acc/dt

  double beta_s;  // exp(-tau_s/dt) etc.
  double beta_dv;
};


#endif // FLUCT_H

