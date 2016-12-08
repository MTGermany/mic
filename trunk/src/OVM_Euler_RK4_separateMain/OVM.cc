//----------------------------------------------------------------------
/* Optimal Velocity Model Simulator
 Author :Hiroshi Watanabe
 URL: http://www.phys.cs.is.nagoya-u.ac.jp/~watanabe/
 $Id: ov.cc 248 2005-12-12 17:22:03Z kaityo $
 This source file: http://traffic.phys.cs.is.nagoya-u.ac.jp/~mstf/sample/ov/src.html#L61

References:
[1] M. Bando, K. Hasebe, A. Nakayama, A. Shibata, and Y. Sugiyama, 
   Jpn. J. Ind. Appl. Math. 11, 203 (1994).
[2] M. Bando, K. Hasebe, A. Nakayama, A. Shibata, and Y. Sugiyama,
   Phys. Rev. E 51, 1035 (1995).
[3] T. S. Komatsu, S. Sasa, Phys. Rev. E 52 5574 (1995).
*/
//----------------------------------------------------------------------

#include <iostream>
#include <math.h>
#include <stdlib.h>

using namespace std;

// Constant Variables

const int N = 10;         // Number of vehicles
const double dt = 0.05;   // Time Interval
const double L = 20;      // System Size
const double c = 2.0;

const int TIME = 200;     // Total Time to simulate
const int INTERVAL = 10;  // Time interval to observe

const double density = L/(double)N;

// Grobal Variables
double a = 1.0;      // Sensitivity
double s_time = 0;   // Simulation Time

//----------------------------------------------------------------------
/**
 * Random Number Generator
 */
inline double
myrand(void){
  return (double)rand()/(double)RAND_MAX;
}
//----------------------------------------------------------------------
/**
 * OV function
 */
inline double
V(double dx){
  return tanh(dx - c)+tanh(c);
}
//----------------------------------------------------------------------
/**
 * Calculate Differential of x and v
 * Input: x,v
 * Output: x' = fx, v' = fv
 */
void
calcf(double x[], double v[],
      double fx[], double fv[]){
  for(int i=0;i<N;i++){
    double dx;
    if(i != N-1){
      dx = x[i+1] - x[i];
    }else{
      dx = x[0] - x[N-1];
    }
    if(dx <-L*0.5) dx += L;

    if(dx <0){
      cerr << "Invalid Time Step" << endl;
      exit(1);
    }
    // OV Function
    fv[i] = a*(V(dx)-v[i]);
  }
  for(int i=0;i<N;i++){
    fx[i] = v[i];
  }
}
//----------------------------------------------------------------------
/**
 * Integration Scheme
 * Euler Method (1st Order)
 */
void
integrate_Euler(double x[], double v[]){
  static double fv[N];
  static double fx[N];
  calcf(x,v,fx,fv);
  for(int i=0;i<N;i++){
    x[i] += fx[i]*dt;
    v[i] += fv[i]*dt;
  }
}
//----------------------------------------------------------------------
/**
 * Integration Scheme
 * Runge-Kutta (4th Order)
 */
void
integrate_RungeKutta(double x[], double v[]){
  static double kx1[N],kv1[N];
  static double kx2[N],kv2[N];
  static double kx3[N],kv3[N];
  static double kx4[N],kv4[N];
  static double tx[N],tv[N];

  calcf(x,v,kx1,kv1);

  for(int i=0;i<N;i++){
    tx[i] = x[i] +  kx1[i]*dt*0.5;
    tv[i] = v[i] +  kv1[i]*dt*0.5;
  }
  
  calcf(tx,tv,kx2,kv2);

  for(int i=0;i<N;i++){
    tx[i] = x[i] +  kx2[i]*dt*0.5;
    tv[i] = v[i] +  kv2[i]*dt*0.5;
  }

  calcf(tx,tv,kx3,kv3);

  for(int i=0;i<N;i++){
    tx[i] = x[i] +  kx3[i]*dt;
    tv[i] = v[i] +  kv3[i]*dt;
  }

  calcf(tx,tv,kx4,kv4);

  for(int i=0;i<N;i++){
    x[i] += (kx1[i]+2.0*kx2[i]+2.0*kx3[i]+kx4[i])/6.0*dt;
    v[i] += (kv1[i]+2.0*kv2[i]+2.0*kv3[i]+kv4[i])/6.0*dt;
  }

}
//----------------------------------------------------------------------
/**
 * Integration
 */
void
integrate(double x[], double v[]){
  
  integrate_RungeKutta(x,v);

  //Treat Periodic Boundary Condition
  for(int i=0;i<N;i++){
    if(x[i] > L) x[i] -= L;
  }
  s_time += dt;
}
//----------------------------------------------------------------------
/**
 * Initialization
 * Putting small perturbation to the uniform flow
 * with amplitude eps.
 */
void
init(double x[], double v[]){

  const double eps = 0.1;
  double dx = L/(double)N;
  double iv = V(dx);
  for(int i=0;i<N;i++){
    x[i] = L/(double)N*(double)i + eps*myrand();
    v[i] = iv;
  }
  cout << "# dt = " << dt << endl;
  cout << "# density = " << density << endl;
  cout << "# a = " << a << endl;
}
//----------------------------------------------------------------------
/**
 * Main Function
 */
int
main(void){
  static double x[N],v[N];
  init(x,v);
  for(int i=0;s_time<TIME;i++){
    integrate(x,v);
    if(i%INTERVAL){
      for(int j=0;j<N;j++){
        cout << x[j] << " " << s_time << endl;
      }
    }
  }
}
//----------------------------------------------------------------------
// End of ov.cc
//----------------------------------------------------------------------
