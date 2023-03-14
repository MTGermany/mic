#ifndef CACC_CC
#define CACC_CC

// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;


//own
#include "CACC.h"
#include "RandomUtils.h"
#include "constants.h"
#include "general.h"
#include "InOut.h"  /// fuer File-Input/Output; fuer VLA nicht relevant
#include "CyclicBuffer.h"



// aktuelles CACC.cc von mic_temporegler
//###########################################################
//! Achtung: "kommerzieller" Temporegler mit CACC-spezifischen
// Features (wie 7 T-Stufen; s=0 => kein Signal, Integer-Input)
// bei ~/projekte/invent/Temporegler/src/CACC.cc
//###########################################################


CACC::CACC(const char fname[], double dt)
{
  initializeMicroModelVariables();  //arne -> init variables from parent MicroModel
  modelNumber=16;
  speedlimit=100000; // martin apr08: initially no speed limit 

  cout << "\nin CACC file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  rhomax = 1./lveh;
  this->dt=dt;

  dt_since_new_leadveh=0;
  a_old=0;

  int nStep=10;
  if(nStep>NSTEP){
    cerr<<"CACC: Error: number nStep of PI-steps greater than NSTEP"<<endl;
    exit(-1);
  }
  for (int iStep=0; iStep<nStep; iStep++){accPI[iStep]=0;}
  
  v_old=0;
  v_lead_old=0;
  dv_old=0;
  dv_lead_old=0;
  s_old=0;
  calc_eq();
  cout <<"End CACC file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.5/lveh)="<<get_veq(0.5/lveh)<<endl;

}



//################################################################
void CACC::get_modelparams(const char fname[]){
//################################################################

  FILE *fp;
  fp=fopen(fname,"r");
  if(fp==NULL)
  {
    cout<<"fopen: Error while opening the file " <<fname<<endl;
    exit(-1);
  }

  InOut inout;

  inout.getvar(fp,&lveh);

  //IDM 

  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&s0);
  inout.getvar(fp,&s1);
  inout.getvar(fp,&delta);   
  inout.getvar(fp,&a);
  inout.getvar(fp,&b);
  inout.getvar(fp,&bmax);

 // ACC 

  inout.getvar(fp,&cool);
  inout.getvar(fp,&jerk);
  inout.getvar(fp,&b_crit);

 // CACC and response

  inout.getvar(fp,&n_multi);
  inout.getvar(fp,&tau_antic);
  inout.getvar(fp,&tau_accel);
  inout.getvar(fp,&tau_brake);

  fclose(fp);

  rhomax = 1./lveh;
  n_multi=max(min(n_multi,2), 1);
  cout <<"CACC.get_modelparams: n_multi="<<n_multi<<endl;
  
}



//################################################################
void CACC::calc_eq()
//################################################################

    // Calculates equilibrium speed array based on original IDM
    // uses numeric iteration procedure and

  // output:
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 

{

  if(false){cout << "\nin CACC.calc_eq()"<<endl;}

   // Find equilibrium velocities veqtab[ir] with simple relaxation
    // method: Just model for homogeneous traffic solved for 
    // the velocity v_it of one arbitrary vehicle
    //  (no brain, but stable and simple method...)

    double v_it=v0;           // variable of the relaxation equation
    int    itmax      = 100;  // number of iteration steps in each relaxation
    double dtmax      = 2;    // iteration time step (in s) changes from
    double dtmin      = 0.01; // dtmin (rho=rhomax) to dtmax (rho=0)

    // start with rho=0

    veqtab[0]         = v0;

    for(int ir=1; ir<=NRHO; ir++)
    {
      double rho = rhomax*ir/NRHO;
      double s   = 1./rho - 1./rhomax;

      // start iteration with equilibrium velocity for the previous density
      v_it             = veqtab[ir-1];
      if(false) cout << endl;

      for (int it=1; it<=itmax; it++){
        double dtloc = dtmax*v_it/v0 + dtmin; // it. step in [dtmin,dtmax]
        double sstar   = s0 + T * v_it + s1*sqrt((v_it+0.000001)/v0);

	// acceleration for various variants

        double acc = (s>s0) 
	  ? a * (1-pow(v_it/v0, 4) - sstar*sstar/(s*s) )
          :0;

	// actual relaxation 

        v_it            += dtloc*acc;
        if((v_it<0)||(s<s0)) v_it=0;

        if(false){ 
          cout << "rho="<< rho<< " s="<<s<<" sstar="<<sstar
               <<" v0="<<v0<<" acc="<<acc<<" v_it="<<v_it<<endl;
	}
      }
      veqtab[ir] = v_it;

    }

    calc_rhoQmax();  // Qmax, rhoQmax 
} // CACC::calc_eq()


//################################################################
// acc takes care of flow-conserving bottlenecks, calculates
// input variables before calculating accVLA
//################################################################
double CACC::acc(int it, int iveh, int imin, int imax,
	       double alpha_v0, double alpha_T,
	       const CyclicBuffer* const cyclicBuf)
{

  //#############################################################
  // Get local input quantities of acceleration models
  // (possible space dependencies modelled by alpha_v0, alpha_T)
  //#############################################################


  double s  = cyclicBuf->get_s(iveh);
  double v  = cyclicBuf->get_v(iveh);         // Eig. Geschwindigkeit
  double dv = v - cyclicBuf->get_v(iveh-1);   // Annaeherungsrate zum Bezugsobjekt
  //double a_lead = cyclicBuf->get_a(iveh-1); //! geht nicht! (nur Vergleich) 

  bool newObject=((fabs(s_old-dv*dt-s)>10) || (fabs((v-v_old)/dt>20)));
  if(newObject){cout <<"CACC: t="<<(it*dt)<<" iveh="<<iveh<<" s-s_old="<<(s-s_old)<<" v-v_old="<<(v-v_old)<<": New Object Detected!"<<endl;}
  double a_lead = !newObject ? (v-v_old-dv+dv_old)/dt : 0;//!! MT 2015-02-02 
  double s_lead  = cyclicBuf->get_s(iveh-1);
  double v_lead  = cyclicBuf->get_v(iveh-1);  
  double dv_lead = (iveh>=2) ? v_lead - cyclicBuf->get_v(iveh-2): 0; 
  double a_lead2 = (v_lead-v_lead_old-dv_lead+dv_lead_old)/dt;


  // introduce anticipation at top level, here:

  double s_antic=s-tau_antic*dv;
  //double s_antic=s;
  double v_antic=v+a_old*tau_antic;
  double v_lead_antic=v_lead+tau_antic*a_lead;
  double dv_antic=v_antic-v_lead_antic;
  //double dv_antic=dv;

  // (v_old,dv_old,a_old etc = local variables for each veh are updated at the end)
  
  if(false){
  //if(iveh==8){
    cout <<"\nCACC.acc: t="<<(it*dt)<<" iveh="<<iveh;
    cout <<" s="<<s<<" v="<<v<<" dv="<<dv<<" a_lead="<<a_lead<<endl
	 <<"        s_lead="<<s_lead<<" v_lead="<<v_lead<<" dv_lead="<<dv_lead
	 <<" a_lead2="<<a_lead2<<endl
	 <<"        v_old="<<v_old<<" a_old="<<a_old<<" dv_old="<<dv_old<<endl;
  }

  if(false){cout<<" it="<<it<<" iveh="<<iveh
	       <<" x[iveh]="<<cyclicBuf->get_x(iveh)
	       <<" x[iveh-1]="<<cyclicBuf->get_x(iveh-1)
	       <<" s="<<s<<" v="<<v<<" dv="<<dv<<endl;}
  if(isnan(s)) exit(-1);


  //################################################################
  // Central call to local accCACC function
  // for other underlying IDM variants (IDM, IDMplus instead of IIDM)
  // see VW.cpp  
  //################################################################

  double accResult=accCACC(it,iveh,v_antic,s_antic,dv_antic,a_lead, v_lead, s_lead, dv_lead,
			   a_lead2, alpha_T, alpha_v0);


  //################################################################
  // self-sufficient update of dynamic variables for use in next time step
  //################################################################

  dt_since_new_leadveh +=dt;

  s_old      =s;
  v_old      =v;
  v_lead_old =v_lead;
  dv_old     =dv;
  dv_lead_old=dv_lead;
  a_old      =accResult;

  return accResult;

}  // end CACC.acc


// helper function for IIDM acceleration before jerk, response etc
double CACC::accDesired(double v, double s, double dv, double a_lead,
		   double alpha_T, double alpha_v0){
  double Tloc  = alpha_T*T; 
  double v0loc=max(1e-10,min(alpha_v0*v0, speedlimit)); 
  double sstar  = s0 + max(Tloc*v + s1*sqrt((v+0.00001)/v0loc)+ 0.5*v*dv/sqrt(a*b),0.);

  double z=sstar/max(s,0.01);
  double accEmpty=(v<=v0loc) ? a*(1- pow((v/v0loc),delta))
    : -b*(1- pow((v0loc/v),a*delta/b));
  double accPos=accEmpty*(1.- pow(z, min(2*a/accEmpty, 100.))  );
  double accInt=a*(1-z*z);

  double accIIDM=(v<v0loc) 
    ?  (z<1) ? accPos : accInt 
    :  (z<1) ? accEmpty : accInt+ accEmpty;

  //CAH

  double a_lead_restr=min(a_lead,a);
  double dvp=max(dv, 0.0);
  double v_lead = v-dvp;
  double denomCAH        =  v_lead*v_lead - 2 * s * a_lead_restr;

  double accCAH   = ( (v_lead*dvp  < - 2 * s * a_lead_restr) &&(fabs(denomCAH)>1e-8))
    ? v*v*a_lead_restr/denomCAH
    : a_lead_restr - 0.5*dvp*dvp/max(s, 0.0001);

  // ACC with IIDM => desired acceleration

  double accACC_IIDM=(accIIDM>accCAH)
    ? accIIDM
    : (1-cool)*accIIDM + cool*( accCAH+b*tanh((accIIDM-accCAH)/b));

  return accACC_IIDM;
} //accDesired


//#######################################################
/** MT 2015-02 accCACC implements coopertatve ACC: as accACC_IIDM (or accACC_IDMplus) 
    but with response times, jerk control (JC), temporal anticipation, 
    multi-anticipation via desired acceleration a_leadDes of the leader 
    acceleration before response,JC)
*/
//#######################################################

double CACC::accCACC(int it, int iveh, double v, double s, double dv, double a_lead, 
		   double v_lead, double s_lead, double dv_lead, double a_lead2, 
		   double alpha_T, double alpha_v0){

  // (1) temporal anticipation at higher level by passing v_antic etc


  // (2) multi-anticipation:
  //  estimation of raw "desired" acceleration accLeadDes of leader 
  // (before response and jerk control) 
  // and own desired acceleration using accLeadDes

  double accLeadDes=accDesired(v_lead,s_lead,dv_lead,a_lead2,alpha_T, alpha_v0);
  double accDes=(n_multi==2) 
    ? accDesired(v,s,dv, accLeadDes, alpha_T, alpha_v0)
    : accDesired(v,s,dv, a_lead,     alpha_T, alpha_v0);

  //accDes=accDesired(v,s,dv, a_lead,     alpha_T, alpha_v0); //!! check no multi

  if(false){
    //if( (iveh==3) && (fabs(accDes)>0.5)){
    cout <<"accCACC: t="<<(it*dt)<<" iveh="<<iveh
	 <<" s="<<s<<" v="<<v<<" dv="<<dv<<endl
	 <<" a_lead="<<a_lead <<" accLeadDes="<<accLeadDes<<endl
	 <<" accDesNoAntic="
	 <<accDesired(v,s,dv, a_lead,    alpha_T, alpha_v0)
	 <<" accDesAntic="
	 <<accDesired(v,s,dv, accLeadDes, alpha_T, alpha_v0)
	 <<endl;

  }


  // (3) model for mechanical response of vehicle

  double accResponse=applyVehicleResponse(accDes); //!!!

  // jerk control

  double accFinal=(jerk>50) 
    ? accResponse
    : applyJerkControl(accResponse);

  //cout <<"CACC:accCACC: accFinal="<<accFinal<<endl;
  return max(accFinal, -bmax);
 
}// accCACC (MT 2015-02)


//#############################################################
// helper method for modelling vehicle response by 3-step PI control path
// needs state variables a_old and dt
//#############################################################

double CACC::applyVehicleResponse(double accDes){
  //double tau_relax=(accDes-a_old>0) ? tau_accel : tau_brake; 
  double tau_relax=(accDes>0) ? tau_accel : tau_brake; 
  if(tau_relax<0.0001){return accDes;}

  int nStep=3;
  double r= 1 - exp(-nStep*dt/tau_relax); 
  accPI[0]=accDes;
  for (int iStep=1; iStep<nStep; iStep++){
    accPI[iStep]=r*accPI[iStep-1]+(1-r)*accPI[iStep];
  }
  return r*accPI[nStep-1]+(1-r)*a_old;
}


//#############################################################
// helper method; "jerk" comes from input: max jerk in normal cond.
// needs same state variable a_old as applyJerkControl and dt
//#############################################################

double CACC::applyJerkControl(double accBefore){
  double abs_jerkBefore=fabs(accBefore- a_old)/dt;
  double sign_jerkBefore=(accBefore- a_old>=0) ? 1 : -1;
  // accmin avoids overbraking after strong brakes
  // since riskfactor/max jerk also increased
  // if *old* acceleration is sufficiently negative
  double accmin=min(accBefore, a_old);
  double riskFactor=2./(1. + tanh((accmin+b_crit)/b));
  double maxJerk= jerk*riskFactor;  
  //double maxJerk= (sign_jerkBefore<0)  // only jerk<0 controlled: avoid overbraking
  // ? jerk*riskFactor : abs_jerkBefore; 
  double abs_jerkAfter=min(abs_jerkBefore, maxJerk);
  double accAfter=a_old+abs_jerkAfter*sign_jerkBefore*dt;
  //cout <<"CACC::applyJerkControl: jerk="<<jerk<<" accBefore="<<accBefore<<" accAfter="<<accAfter<<endl;
  return accAfter;
}
    


#endif // CACC_CC
