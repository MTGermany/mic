#ifndef VW_CC
#define VW_CC

// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;


//own
#include "VW.h"
#include "RandomUtils.h"
#include "constants.h"
#include "general.h"
#include "InOut.h"  /// fuer File-Input/Output; fuer VLA nicht relevant
#include "CyclicBuffer.h"



// aktuelles VW.cc von mic_temporegler
//###########################################################
//! Achtung: "kommerzieller" Temporegler mit VW-spezifischen
// Features (wie 7 T-Stufen; s=0 => kein Signal, Integer-Input)
// bei ~/projekte/invent/Temporegler/src/VW.cc
//###########################################################


VW::VW(const char fname[], double dt)
{
  initializeMicroModelVariables();  //arne -> init variables from MicroModel
  modelNumber=1; //arne 10-5-2005
  speedlimit=100000; // martin apr08: initially no speed limit 

  cout << "\nin VW file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname);
  rhomax = 1./lveh;
  this->dt=dt;
  //this->p_proj=&proj;

  dt_since_new_leadveh=0;
  a_old=0;
  a_lead_old=0;
  v_old=0;
  dv_old=0;
  s_old=100;
  calc_eq();
  cout <<"End VW file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

}



//################################################################
void VW::get_modelparams(const char fname[]){
//################################################################

  FILE *fp;
  fp=fopen(fname,"r");
  if(fp==NULL)
  {
    cout<<"fopen: Error while opening the file " <<fname<<endl;
    exit(-1);
  }

  InOut inout;

  // quick hack to parse additional n_multi parameter if it exists (2015-02)
  // (does not work robustly since "\begin{verbatim}" not excluded)

  //double proformaParams[20];
  //int n_param;
  //inout.get_col(fname,1,n_param,proformaParams);
  //bool withAccDes=(n_param==14);

  // end quick hack


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

 // VW 
  inout.getvar(fp,&cool);
  inout.getvar(fp,&jerk);
  inout.getvar(fp,&b_crit);

  // optional anticipation next-nearest leader (n_multi=2)
  // (get_var returns 0 if line not exists)

  inout.getvar(fp,&n_multi);
  if(n_multi==0){n_multi=1;}
  else{n_multi=2;}


  cout <<" n_multi="<<n_multi<<endl;

  fclose(fp);

  rhomax = 1./lveh;
  
}



//################################################################
void VW::calc_eq()
//################################################################

    // Calculates equilibrium velocity  with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // !! calculates THE WHOLE FIELD veq
    // NO resignation effect respected up to now
  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin VW.calc_eq()"<<endl;}

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
} // VW::calc_eq()


//################################################################
// acc takes care of flow-conserving bottlenecks, calculates
// input variables before calculating accVLA
//################################################################
double VW::acc(int it, int iveh, int imin, int imax,
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
  double a_lead = (v-v_old-dv+dv_old)/dt;//!! MT 2015-02-02 

  // (v_old,dv_old,a_old etc = local variables for each veh are updated at the end)
  
  //if(false){
  if(iveh==8){
    cout <<"\nVW.acc: t="<<(it*dt)<<" iveh="<<iveh;
    cout <<" s="<<s<<" v="<<v<<" dv="<<dv<<" a_lead="<<a_lead<<endl
	 <<"        v_old="<<v_old<<" a_old="<<a_old<<" dv_old="<<dv_old<<endl;
  }

  if(false){cout<<" it="<<it<<" iveh="<<iveh
	       <<" x[iveh]="<<cyclicBuf->get_x(iveh)
	       <<" x[iveh-1]="<<cyclicBuf->get_x(iveh-1)
	       <<" s="<<s<<" v="<<v<<" dv="<<dv<<endl;}
  if(isnan(s)) exit(-1);


  //################################################################
  // !!! Choice of one of the acceleration models !!!
  // accVLA: Most general case with separate jerk control, input errors etc
  // (the other 3 methods have common jerk control block applyJerkControl(acc))
  //        (if special effects turned off, nearly as accACC_IIDM)
  //       !! with strong sudden braking -4 m/s^2 and cool=1
  //       !! accVLA much better than the three simple versions below 
  //       !! Jerk control not reason but possible intelligent sstarmin (seffmin_s0 etc)
  //         (accVLA: integrated 3 below: separate method)(jan14)
  // accACC_IIDM: New ACC formulation with IIDM (preferred version)
  // accACC_IDMplus: New ACC formulation with IDM-Plus model (also OK)
  // accACC_IDM: New ACC formulation with IDM+v>v0 treatment (similar to RoySoc paper)
  //################################################################

  //double accResult=accVLA(it,iveh,v,s,dv,a_lead,alpha_T, alpha_v0, false);
  //double accResult=accACC_IIDM(it,iveh,v,s,dv,a_lead,alpha_T, alpha_v0);
  double accResult=accACC_IDMplus(it,iveh,v,s,dv,a_lead,alpha_T, alpha_v0); 
  //double accResult=accACC_IDM(it,iveh,v,s,dv,a_lead,alpha_T, alpha_v0); 


  //################################################################
  // self-sufficient update of dynamic variables for use in next time step
  //################################################################

  v_old=v;
  dv_old=dv;
  a_old=accResult;

  return accResult;

}  // end VW.acc



//#######################################################
/** MT jun11 accACC_IIDM just implements the final ACC model with 
IIDM basis and without confusing special cases (but jerk control is possible)
*/
//#######################################################

double VW::accACC_IIDM(int it, int iveh, double v, double s, double dv, double a_lead,
		     double alpha_T, double alpha_v0){

  // IIDM

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
    
  // ACC with IIDM

  double accACC_IIDM=(accIIDM>accCAH)
    ? accIIDM
    : (1-cool)*accIIDM + cool*( accCAH+b*tanh((accIIDM-accCAH)/b));

  if(false){
    cout <<"s="<<s<<" v="<<v<<" dv="<<dv<<" a_lead="<<a_lead
	 <<" accIIDM="<<accIIDM<<" accACC_IIDM="<<accACC_IIDM <<endl;
  }

  double accFinal=(jerk>50) 
    ? accACC_IIDM
    : applyJerkControl(accACC_IIDM);

  //cout <<"VW:accACC_IIDM: accFinal="<<accFinal<<endl;
  return accFinal;
 
}



//#######################################################
/** MT nov13 accACC_IDMplus just implements the final ACC model with 
 IDMplus basis and without confusing special cases (but jerk control is possible)
*/
//#######################################################

double VW::accACC_IDMplus(int it, int iveh, double v, double s, double dv, double a_lead,
		     double alpha_T, double alpha_v0){

  // IDMplus

  double Tloc  = alpha_T*T; 
  double v0loc=max(1e-10,min(alpha_v0*v0, speedlimit)); 
  double sstar  = s0 + max(Tloc*v + s1*sqrt((v+0.00001)/v0loc)+ 0.5*v*dv/sqrt(a*b),0.);

  double z=sstar/max(s,0.01);
  double accEmpty=(v<=v0loc) ? a*(1- pow((v/v0loc),delta))
    : -b*(1- pow((v0loc/v),a*delta/b));
  double accInt=a*(1-z*z);

  double accIDMplus=min(accEmpty, accInt);

  //CAH

  double a_lead_restr=min(a_lead,a);
  double dvp=max(dv, 0.0);
  double v_lead = v-dvp;
  double denomCAH        =  v_lead*v_lead - 2 * s * a_lead_restr;

  double accCAH   = ( (v_lead*dvp  < - 2 * s * a_lead_restr) &&(fabs(denomCAH)>1e-8))
    ? v*v*a_lead_restr/denomCAH
    : a_lead_restr - 0.5*dvp*dvp/max(s, 0.0001);
    
  // ACC with IDMplus

  double accACC_IDMplus=(accIDMplus>accCAH)
    ? accIDMplus
    : (1-cool)*accIDMplus + cool*( accCAH+b*tanh((accIDMplus-accCAH)/b));


  double accFinal=(jerk>50) 
    ? accACC_IDMplus
    : applyJerkControl(accACC_IDMplus);

  if(false){
    cout <<"s="<<s<<" v="<<v<<" dv="<<dv<<" a_lead="<<a_lead
	 <<" accIDMplus="<<accIDMplus
	 <<" accACC_IDMplus="<<accACC_IDMplus
	 <<" accFinal="<<accFinal
	 <<endl;
  }


  return accFinal;
 
}

//#######################################################
/** MT jun11 accACC_IDM just implements the final ACC model with original IDM
basis plus v>v0loc treatment but without confusing other special cases
(but jerk control is possible)
differences to RoySoc paper:
 (i)  v>v0loc treatment 
 (ii) sstar restricted to s0 from below (omission error in RoySoc)
 (iii) dvp=max(dv,0) used instead of dv in CAH case distinction
       (minor error in RoySoc)
*/
//#######################################################

double VW::accACC_IDM(int it, int iveh, double v, double s, double dv, double a_lead,
		     double alpha_T, double alpha_v0){

  // IDM with v>v0 treatment

  double Tloc  = alpha_T*T; 
  double v0loc=max(1e-10, min(alpha_v0*v0, speedlimit)); 
  double deltaloc=(v<v0loc) ? delta : 1; 
  double sstar  = s0 + max(Tloc*v + s1*sqrt((v+0.00001)/v0loc)+ 0.5*v*dv/sqrt(a*b),0.);

  double accIDM=a*( 1.- pow((v/v0loc),deltaloc) - SQR(sstar/s));

  //CAH

  double a_lead_restr=min(a_lead,a);
  double dvp=max(dv, 0.0);
  double v_lead = v-dvp;
  double denomCAH        =  v_lead*v_lead - 2 * s * a_lead_restr;

  double accCAH   = ( (v_lead*dvp  < - 2 * s * a_lead_restr) &&(denomCAH!=0))
    ? v*v*a_lead_restr/denomCAH
    : a_lead_restr - 0.5*dvp*dvp/max(s, 0.0001);
    
  // ACC with IDM

  double accACC_IDM=(accIDM>accCAH)
    ? accIDM
    : (1-cool)*accIDM + cool*( accCAH+b*tanh((accIDM-accCAH)/b));

  double accFinal=(jerk>50) 
    ? accACC_IDM
    : applyJerkControl(accACC_IDM);

  return accFinal;
}


//#######################################################
/** implements the most general case with sensor errors etc:
"Reverse engineering" and physical simulation of processes in real ACC
different acceleration functions can be chosen, e.g., IDMnew
 **/
//#######################################################


double VW::accVLA(int it, int iveh, double v, double s, double dv, double a_lead,
		  double alpha_T, double alpha_v0, bool test_output)
{

  // IDM: effectives Tmin/T selbst wenn Vorderfz s. schnell entfernt
  double sstarmin_vT = 0.1; // 0.1 ... 0.3  bisher 0.2
  
  // CAH und IDM:  Kleinster Wert von (s-s0)/s0  (seff=s-s0 im Nenner)
  // "Nicht-glattes" Verhalten falls seffmin_s0 > 0.5
  double seffmin_s0  =0.2;  // 0.1 ... 0.5 bisher 0.1
  
  // Test Ereignis "Neues Bezugsobjekt" 
  // (fuer manche Versionen der Reverse Engineering dieser Fkt.)
  double sdiffcrit   = 10;  // 5 ... 10 (>v0*dt)
  
  //#####################################################
  // Ggf. Anpassung der eingestellten Folgezeit T und 
  // Wunschgeschwindigkeit v0 um Faktoren alpha_T, alpha_v0
  //#####################################################

 
  double Tloc  = alpha_T*T; 
  double v0loc=max(1e-8,min(alpha_v0*v0, speedlimit));   // martin mai08
  
  double aloc = a; //parameter
  
  //#############################################################
  // Beruecksichtigung realer Sensor-Kenngroessen und -Fehler 
  // falls ConsiderSensorErrors=true (part of accVLA: most general version)
  //#############################################################

  bool ConsiderSensorErrors=false; //!! (Arne, 6-5-05)
  
 
  if(ConsiderSensorErrors){
    
    bool isRange = false; //!!
    double smax  = 150;  //1500,150
  
    bool isDrift    = false; //!!
    double s_drift  = 1;     // 1
    double dv_drift =-0.2; // -0.2
  
    bool isFluct       = false; //!!
    double sigma_s_rel = 0.02; //0, 0.02 sigma_s=sigma_s_rel*s 
    double sigma_dv    = 0.1;    // 0, 0.1(m/s)
    double sigma_v     = 0.1;    // 0, 0.1(m/s)
 
    // equal-distributed random variables with mu=0, sigma=1
    // random numbers in [-0.5 ... 0.5] have variance=1/12
    
    double sqrt12=sqrt(12.);
    
    double r1 = sqrt12*myRand(); // ((double) (rand()/((double) RAND_MAX+1.)) - 0.5);
    double r2 = sqrt12*myRand();//((double) (rand()/((double) RAND_MAX+1.)) - 0.5);
    double r3 = sqrt12*myRand();//((double) (rand()/((double) RAND_MAX+1.)) - 0.5);
    
    if(isRange){s=(s<smax) ? s : 1./SMALL_VAL;}
    
    if(isFluct){
      s     *= (1.+r1*sigma_s_rel);
      dv    += r2*sigma_dv;
      v     += r3*sigma_v;
    }
    
    if(isDrift){
      s      += s_drift;
      dv     +=dv_drift;
    }
    
    if (false){
      //if (iveh==2){
      cout<<"ConsiderSensorErrors=true: "
	  <<" r1*sigma_s_rel="<<(r1*sigma_s_rel)
	  <<" r2*sigma_dv="<<(r2*sigma_dv)
	  <<endl;
    }
  }
  
  
  if(test_output){
    cout <<" VW.acc: Input Groessen nach Aufbringen Flukt.:"
	 <<" s="<<s<<" v="<<v<<" dv="<<dv<<" a_lead="<<a_lead<<endl;
  }
  
  
  //#############################################################
  // Test auf Wechsel des Bezugsobjekts (setzt dt_since_new_leadveh)
  // (VOR) Filter!
  //#############################################################
  
  double s_diff= s_old-dv*dt - s;
  if(s_diff<0){s_diff=-s_diff;}
  if(s_diff>sdiffcrit){
    dt_since_new_leadveh=0.;
    if(test_output){
      cout << "VW.acc: Neues Bezugsobjekt erkannt!"<<endl;
    }
  }
  
  if(false){
    if(s_diff>sdiffcrit){
      cout<<" VW.acc: Discontinuitiy of s implies new lead vehicle!\n"
	  <<" s_old="<<s_old<<" dv="<<dv<<" s="<<s
	  <<" |s_diff|="<<s_diff<<endl;
    }
  }
  
  
  //#############################################################
  // !!Konsistenzcheck der Eingangsgroessen
  // (+ kontinuierliches Anpassen von v0 bei ext. Aenderungen)
  // + Tiefpass-Filterung von s,v,dv
  // zum Daempfen der Fluktuationen (Tiefpass von a nach dessen 
  // Berechnung weiter unten (part of accVLA: most general version)
  //#############################################################
  
  // EMA !! evtl. alle EMA's zusammen zu Jerk control; ereignisgest.
  // EMA auf tau=0 setzten, wenn Gefahr! Dann zurueck und nochmal ohne
  // Daempfung => brauche zwei "alte" Werte!
  
  s=max(s,0.0);
  v=max(v,0.0);
  dv=min(dv, v);
  
  double tau_s     =0.2;
  double tau_v     =0.5;
  double tau_dv    =0.2;
  double tau_alead =0.5;

  
  tau_s=tau_v=tau_dv=tau_alead=0;
  
  
  double beta_s     = (tau_s <0.1*dt)     ? 1 : 1. - exp(-dt/tau_s);
  double beta_v     = (tau_v <0.1*dt)     ? 1 : 1. - exp(-dt/tau_v);
  double beta_dv    = (tau_dv<0.1*dt)     ? 1 : 1. - exp(-dt/tau_dv);
  
  if(dt_since_new_leadveh>dt){
    s      = beta_s*s          + (1.-beta_s)*s_old;
    v      = beta_v*v          + (1.-beta_v)*v_old;
    dv     = beta_dv*dv        + (1.-beta_dv)*dv_old;
  }
  
  //#####################################################
  // Ermittlung von a_lead durch Differenzieren 
  // (NACH Filtern der Zaehler-Groessen, sonst wg. kl. Nenner kritisch!)
  // Hinweis: Streiche diese Passage, wenn direkter Input 
  //#####################################################
  
  a_lead = (v-v_old-dv+dv_old)/dt;//!!
  if (it<10){cout <<"VW.acc: it="<<it<<" step=0 a_lead="<<a_lead<<endl;}
  a_lead=min(max(a_lead, -bmax), bmax);
  
  double beta_alead = (tau_alead <0.1*dt) ? 1 : 1. - exp(-dt/tau_alead);
  a_lead = beta_alead*a_lead + (1.-beta_alead)*a_lead_old;
  if (it<10){cout <<"VW.acc: it="<<it<<" step=1 a_lead="<<a_lead<<endl;}
   if(it<2){a_lead=0; a_lead_old=0;}
  
  if(test_output){
    cout <<" VW.acc: Input Groessen nach EMA-Filterung:"
	 <<" s="<<s<<" v="<<v<<" dv="<<dv<<" a_lead="<<a_lead<<endl;
  }
  
  //########################################################
  // IDM Beschleunigung (part of accVLA: most general version)
  //########################################################
  
  double v0_sstar=20;   // lediglich Normierung, dass s1 Einheit m
  double sstar  = s0 + Tloc*v + s1*sqrt((v+0.01)/v0_sstar)+ (0.5*v*dv)/sqrt(aloc*b);
  
    double sstarmin=seffmin_s0*s0+sstarmin_vT*v*Tloc; //!! neu
  // bisher:
  //double sstarmin=           s0+sstarmin_vT*v*Tloc;
  
  if(sstar<sstarmin) {sstar=sstarmin; }
  
  double tau_relax=20.;  // physiolog. Relaxationszeit, falls v>v0
  double accFree= (v<v0loc) 
    ? aloc * (1.- pow((v/v0loc),delta))
    : (v0loc-v)/tau_relax; 
  
  //  Unterdruecke accFree, falls dv^2/2s Bremsverzoeg. >=b impliziert
  // (Das FD bleibt unveraendert)
  double b_kin=0.5*max(dv,0.0)*dv/s;
  accFree*=max(1-b_kin/b,0.0);
  
  double accIDM = accFree - aloc *sstar*sstar/(s*s);
  
  if(test_output){
    cout <<" VW.accVLA:IDM part: sstar="<<sstar<<" accIDM="<<accIDM<<endl;
  }
  
  //########################################################
  // CAH Bremsbeschleunigung (constant-acceleration heuristics) 
  //########################################################
  
  
  // Unterdruecke CAH- "Mitzieheffekte", falls Vordermann staerker
  // als gewuenscht beschleunigt (part of accVLA: most general version)

  double aleadmax=max(0.0, 1.1*accIDM); 
  double a_lead_eff=min(aleadmax, a_lead); // !! bisher: hart
  
  // CAH
  
  double dvp=max(dv, 0.0);
  double v_lead = v-dvp;
  double seff=max(s-s0, seffmin_s0*s0); 
  bool   smin_at_stop = (v_lead*dvp  < - 2 * seff*a_lead_eff);
  double denom        =  v_lead*v_lead - 2 * seff*a_lead_eff;
  
  double accCAH    = ( smin_at_stop &&(denom!=0))
    ? v*v*a_lead_eff/denom
    : a_lead_eff - 0.5*dvp*dvp/seff;



  
  //######################################################
  // Mischen von IDM, CAH ("Beste aller Welten abzueglich b")
  // (part of accVLA: most general version)
  //######################################################
  
  // coolness=0: acc1=acc2=accIDM; coolness=1 (VLA):  acc1=accCAH
  double acc1     = cool*accCAH + (1.-cool)*accIDM;
  
  // folgende 3 Zeilen erlauben schnelle (IDM-) Beschl. am Stauende
  double db1     = 0.2*aloc;
  double shiftb1 = 0.0*aloc;  // shift to negative values of accIDM
  //double shiftb1 = 0.2*aloc;  // shift to negative values of accIDM
  double delta_b = b*0.5*(tanh((-accIDM-shiftb1)/db1)+1);
  //delta_b  = b;  //!! Deaktivieren der "delta" Manipulation
  
  double acc2 = - delta_b + maxsmooth(accIDM, acc1, delta_b);//!!possibly overridden


  //######################################################
  // Neue vereinfachte Formulierung fuer acc2=acc ohne Jerk (mt nov09)
  // in ~/tex/papers/ACC_RoyalSoc_2009
  //######################################################

  double denomNew        =  v_lead*v_lead - 2 * s * a_lead;

  double accCAHNew    = ( (v_lead*dvp  < - 2 * s * a_lead_eff) &&(denomNew!=0))
    ? v*v*a_lead/denomNew
    : min(a,a_lead) - 0.5*dvp*dvp/max(s, 0.0001);

  double acc2New=(accIDM>accCAHNew)
    ? accIDM
    : (1-cool)*accIDM + cool*( accCAHNew+b*tanh((accIDM-accCAHNew)/b));

  // (mt nov09) activate if new formulation with classical IDM wanted
  //  ( deactivate formulation with IDMnew below)

  acc2=acc2New; // possibly overridden!!

  //######################################################
  //  (mt nov09)
  // Verallgemeinerung der  vereinfachten Formulierung auf exaktes Einhalten
  // von s0+vT im Gleichgewicht fuer v<=v0fuer acc2=acc ohne Jerk
  // plots in ~/tex/papers/ACC_RoyalSoc_2009/analytics/
  // Erklaerung hier in README_ACCmodel
  //######################################################

  double z=sstar/max(s,0.01);
  double accEmpty=(v<=v0loc) ? a*(1- pow((v/v0loc),delta))
    : -b*(1- pow((v0loc/(v+1e-10)),a*delta/b));
  double accPos=accEmpty*(1.- pow(z, min(2*a/accEmpty, 100.))  );
  double accInt=a*(1-z*z);

  double accIDMnew=(v<v0loc) 
    ?  (z<1) ? accPos : accInt 
    :  (z<1) ? accEmpty : accInt+ accEmpty;

    
  double acc2New2=(accIDMnew>accCAHNew)
    ? accIDMnew
    : (1-cool)*accIDMnew + cool*( accCAHNew+b*tanh((accIDMnew-accCAHNew)/b));

  // (mt nov09) activate if new formulation WITH IDMnew wanted

  acc2=acc2New2; //!!
  //acc2=accIDMnew; //!!


  
  //######################################################
  // Jerk control: (part of accVLA: most general version)
  // (other acc versions have common "applyJerkControl(..) method)
  //######################################################
  
  
  // (1) Teste, ob Jerk-control aktiv
  
  
  //bool isJerkControl=(dt_since_new_leadveh<2*taumax_a);
  bool isJerkControl=true; //!! (arne 6-5-05)
  
  // (2) Fuehre Jerk-control durch
  
  double acc3=acc2;  // Vorbelegung, falls Jerk-control nicht aktiv
  
  /*
  if(isJerkControl){
    // nachste Zeile verhindert Moeglichkeit, 
    // dass Vollbremsung ggf. nicht schnell genug beendet wird
    double acc_min= min(acc2, a_old);  
    
    double taumax_a_newTargetCorr=min(taumax_a, 0.5*dt_since_new_leadveh); 
    
    // reduce or eliminate jerk control in case of danger
    double tau_a  = taumax_a_newTargetCorr*0.5*(1. + tanh((acc_min+b_crit)/b) );
    
    // EMA of a (analog to s,v,dv at the beginning)
    // EMA !! evtl. alle EMA's zusammen zu Jerk control; ereignisgest.
    // EMA auf tau=0 setzten, wenn Gefahr!
    
    double beta   = 1. - exp(-dt/tau_a);
    acc3 = beta*acc2 + (1.-beta)*a_old;
    //cout <<"in jerkControl: taumax_a="<<taumax_a<<" dt_since_new_leadveh="<<dt_since_new_leadveh <<" tau_a="<<tau_a<<endl;

    //cout << "new lead object: tau_a="<<tau_a <<endl; 
  }
  */

  // new Method (jan14)
  // "jerk" comes from input: max jerk in normal cond.
  if(isJerkControl){

    double abs_jerkBefore=fabs(acc2- a_old)/dt;
    double sign_jerkBefore=(acc2- a_old>=0) ? 1 : -1;
    // accmin avoids overbraking after strong brakes
    // since riskfactor/max jerk also increased
    // if *old* acceleration is sufficiently negative
    double accmin=min(acc2, a_old);
    double riskFactor=2./(1. + tanh((accmin+b_crit)/b));
    double maxJerk= jerk*riskFactor;  
    //double maxJerk= (sign_jerkBefore<0)  // only jerk<0 controlled: avoid overbraking
    // ? jerk*riskFactor : abs_jerkBefore; 
    double abs_jerkAfter=min(abs_jerkBefore, maxJerk);
    acc3 = a_old+abs_jerkAfter*sign_jerkBefore*dt;

  }
    
  //#############################################################
  // Test-Output (VW::accVLA)
  //#############################################################
  
  if(test_output){
    cout <<" VW.accVLA: VLA part:"
	 <<" v_old="<<v_old
	 <<" a_old="<<a_old
	 <<" derived effective a_lead_eff="<<a_lead_eff
      // <<" alpha_v0loc="<<alpha_v0loc
      // <<" alpha_T="<<alpha_T<<"):"
	 <<endl
	 <<" accCAH="<<accCAH
	 <<" acc before jerk ctrl="<<acc2
      //  <<" smin_at_stop="<<smin_at_stop
      // <<" b_kinStop="<<(v*v*a_lead_eff/denom)
      //  <<" b_kinDrive="<<(0.5*dv*dv/seff - a_lead_eff)
      //  <<" delta_b="<<delta_b
      //  <<"   accIDM="<<accIDM
	 <<" result="<<max(acc3,-bmax)
	 <<endl
	 <<"--------------------------"
	 <<endl;
  }
  
  //#############################################################
  // update der internen History-Zustandsvariablen und return 
  //#############################################################
  
  double acc4=max(acc3,-bmax);
  
  a_old      = acc4;
  a_lead_old  = a_lead;
  s_old      = s;
  v_old      = v;
  dv_old     = dv;
  dt_since_new_leadveh +=dt;
  return(acc4);
}  // VW::accVLA


  // new Method (jan14) !!
  // "jerk" comes from input: max jerk in normal cond.
double VW::applyJerkControl(double accBefore){
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
  a_old=accAfter; // update old acceleration (data variable of class VW)
  //cout <<"VW::applyJerkControl: jerk="<<jerk<<" accBefore="<<accBefore<<" accAfter="<<accAfter<<endl;
  return accAfter;
}
    


#endif // VW_CC
