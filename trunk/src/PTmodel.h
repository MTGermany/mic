#ifndef PTmodel_H
#define PTmodel_H



#include "MicroModel.h"
#include "CyclicBuffer.h"
#include "RandomUtils.h"

class PTmodel: public MicroModel
{

 public:
  // static gibt DOS! linkkt nicht! (MT apr14)
  int counter_errZstarNeg; // error zstar<0 (pCrash>=0.5) 
  int counter_errAccLtMin; // error acc<-bmax
  int counter_errUcurvPos; // error U(a*)=min instead of max
  int counter_errVarNeg; // error estimated variance <0 (related to errUcurvPos)
  int counter_errAoutOfRange; // this should not happen because should be catched

  PTmodel(){;} //default constructor needed for PTmodel in Heterogen

  PTmodel(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~PTmodel(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double dv,
		   double alpha_v0, double alpha_T);
  
  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: PTmodel.setSpeedlimit not yet implemented!"<<endl;
  }


 protected:

  // PTmodel parameters

  double v0; // Desired velocity (m/s)
  double s0; //min. bumper-to-bumper distance (m)  
  double tauOVM;  //velocity relaxation time (s) of the freeflow part (or use a!);

  double wm;          // weighting factor of negative PT generalized utilities
  double a0;           //crossover acceleration constant=>decreasing sensitivity
  double gamma;   // sensitivity exponent: PT generalized
    		// utility propto +/- acc^gamma if |acc/a0| >>1
  double wc;            // weighting of crashes per unit PT utility
  double taumax;     // max. time horizon for evaluating crash risk
  double alpha;       // velocity uncertainty variation coeff. (for determ. crash risk)

  double beta;          // logit uncertainty parameter (1/beta=acc uncertainty/a0)
  double tauCorr;    // Correlation time of acceleration/estimation errors
  double bmax;          //max. PT deceleration 
  //2	s0;            //min. bumper-to-bumper distance (m)     
  

  
  // PTmodel state variables (elapsed times, old values,
  // acceleration relaxations etc)

  double dt;          // from proj.dt
  // double dtNewTarget; // time elapsed since new target vehicle

 private:

  static constexpr double gaussnorm=0.39894228; // 1/sqrt(2*PI);
  double delta; // 0.5*(1-gamma)
  double dw;  // 1-wm

  double uPTatab[NTABMAX];   // tabulated d(U_PT)/da
  double uPTaatab[NTABMAX]; // tabulated d^2(U_PT)/da^2
  double get_uPTa(double a);
  double get_uPTaa(double a);
  
  void get_modelparams(const char* fname);
  void calc_eq();
  double gaussdens(double z){return gaussnorm*exp(-0.5*z*z);}


  // Wiener process with stationary variance=1, mean=0, correlation time=tau
  // => ~/trafficSim/sources/stochProcesses/

  double wiener;  // stochastic variable for the Wiener process
  double wienerUpdate(double xold, double tau, double dt){
    double rand= myRand()-0.5; // rand sim G(-1/2,1/2)
    return xold*exp(-dt/tau) + sqrt(24*dt/tau)*rand;
  }
  
  void testModelEquations(const char fname[]);


};

#endif // PTmodel_H
