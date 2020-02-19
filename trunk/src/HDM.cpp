#ifndef HDM_CC
#define HDM_CC

// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;

//own
#include "HDM.h"
#include "general.h"
#include "InOut.h"
#include "RandomUtils.h"
#include "CyclicBuffer.h"

/////////////////////////////////////////////////////
//arne: HDM (former Human) is not as generic as VDT
//since applying any base model is not that easy!
//VDT is the only generic meta model which works 
//with some underlying base models in the way 
//that one can specify a basemodel-number!!!
/////////////////////////////////////////////////////


// #############################################################
// Constructors
// #############################################################


HDM::HDM(const char fnameIDM[], const char fname[], double dt)
{
  T_react_span=0;
  initializeMicroModelVariables();  //arne -> init variables from MicroModel

  // !! initialize state history variable and define dt 
  // BEFORE  get_IDMmodelparams (latter needs dt)

  modelNumber=2;
  speedlimit=100000; // martin apr08: initially no speed limit 

  this->dt=dt;
  dt_since_new_leadveh=0;
  a_old=0;
  s_old=1000;

  // initialize resignation-related dyn. variables

  alpha_adyn=1;      // initialization (leave to 1!)
  alpha_v0dyn=1;     
  alpha_Tdyn=1; 

  n_temp_antic=1;


  cout << "\nin HDM file Cstr: fnameIDM= "<<fnameIDM <<endl;
  get_IDMmodelparams(fnameIDM); // no calc_eq() here

  cout << "\nin HDM file Cstr: fname= "<<fname<<endl;
  get_modelparams(fname); // no calc_eq() here

  calc_eq(); 

  cout <<"End HDM file Cstr: Test: rhomax="<<rhomax
       <<" get_veq(0.02)="<<get_veq(0.02)<<endl;

  //  exit(0);
}

//################################################################
// initialize random reaction time
// called from Heterogen
// main switch to activate distributed reaction time
// is defined in proj-file 
//################################################################
void HDM::init_T_react()
{ 
  double newTreact = uniform(T_react-T_react_span,T_react+T_react_span);
  //double newTreact = gaussian(T_react, T_react_span,2); //cut-off default is 3*sigma
  //cout<<"HDM.init_Treact is activated in proj: T_react="<<T_react
  //      <<" T_react_variation="<<T_react_variation
  //      <<" --> ind. T_react="<<newTreact<<endl;
  T_react=newTreact;
}

//################################################################
void HDM::get_IDMmodelparams(const char fname[]){
//################################################################

  FILE *fp;
  fp=fopen(fname,"r");
  filecheck(fp,fname);
  InOut inout;

  inout.getvar(fp,&lveh);

  inout.getvar(fp,&v0);
  inout.getvar(fp,&T);
  inout.getvar(fp,&s0);
  inout.getvar(fp,&s1);
  inout.getvar(fp,&delta);   
  inout.getvar(fp,&a);
  inout.getvar(fp,&b);
  inout.getvar(fp,&bmax);

  fclose(fp);

  rhomax = 1./lveh;

  // KEIN calc_eq hier!!    

}  // end get_IDMmodelparams


//################################################################
void HDM::get_modelparams(const char fname[])
//################################################################
{
  FILE *fp;
  fp=fopen(fname,"r");
  filecheck(fp,fname);
  InOut inout;
	
	//oct06:
	int number_params = inout.getNumberOfDataLines(fname);
	if(number_params != 11){
		cerr<<"\nError!!!\n"<<endl;
		cerr<<"HDM input for \""<<fname<<"\" requires 11 parameters (upgrade version Oct06) but provided are only "<<number_params<<endl;
		cerr<<"\nplease run : micUpgrade project"<<endl<<endl;
		exit(-1);
	}
	
	
  inout.getvar(fp,&T_react);
  inout.getvar(fp,&T_react_span);
  inout.getvar(fp,&smax);
  inout.getvar(fp,&n_antimax);
  inout.getvar(fp,&n_temp_antic);
  inout.getvar(fp,&alpha_a_resmin);
  inout.getvar(fp,&alpha_v0_resmin);
  inout.getvar(fp,&alpha_T_resmax);
  inout.getvar(fp,&tau_res);
  inout.getvar(fp,&v1rel);
  inout.getvar(fp,&v2rel);  
  fclose(fp);

  beta = 1.-exp(-dt/tau_res);


  cout<<"T_react_span="<<T_react_span<<endl;

  if(n_antimax<=0)
    {
      cout<<"Cstr. of HDM: n_antimax = "<<n_antimax<<" not possible."
	  <<" At least one interaction partner needed, so we set n_antimax = 1"
	  <<endl;
      n_antimax=1;
    }

  if(n_temp_antic < 0)
    {
      cout<<"Cstr. of HDM: restrict  n_temp_antic= "<<n_temp_antic<<" to non-negative values."<<endl;
      n_temp_antic=0;
    }

  //#################################################################

  // correct s0 and T to obtain same FD as IDM:
  // if n_antimax->infty and s_stretch=1
  // then se(v)=PI/sqrt(6)*(s0+vT)*(...)^{-1/2}
  // => multiply s0,s1 and T (but not v0) by a factor of
  // 1/(sum_{i=1}^{n_antimax} (S_NN/S_NNN)^2
  // cf. HDM paper with identifier gamma

  //test: increase next nearest neigbor interaction
  //s_stretch=0.5;  //!! incr. NNN interaction: s=s_NN+s_stretch*sum_{j=2}^ ...
  
  s_stretch=1.;     // normal NNN interaction: s=sum_{j=1}^jmax s_{i-j, i-j+1}

  double gamma=0;

  for (int i=1; i<=n_antimax; i++){
    gamma += 1./SQR(i*s_stretch);
  }

  gamma=sqrt(gamma); 

  cout<<"HDM: correction factor for (T,s0,s1) due to anticipation: gamma="<<gamma<<endl;
  cout<<"before normalization: T="<<T<<", s0="<<s0<<", s1="<<s1<<endl;
	
  T  /= gamma;
  s0 /= gamma;
  s1 /= gamma;
	
  cout<<"after normalization: T="<<T<<", s0="<<s0<<", s1="<<s1<<endl;
	

  //#################################################################

  //first most downstream vehicles do not have enough vehicles ahead
  //-----> REnormalise (T, s0, s1) adaptively
  //notice that this is a REnormalization applied to only a few vehicles below!


  //array for gamma correction factors
  //index 0 means no vehicle ahead, same for index 1 --> full REnormalisation
  //index 2 means anticipation of 2 vehicles.....
  if(n_antimax>=N_ANTICIPATION_MAX){
    cerr<<"HDM: renormFactorAntic["<<N_ANTICIPATION_MAX
	  		  <<"] array not sufficient for n_antimax="<<n_antimax
				<<"! exit!"<<endl;
    exit(-1);
  }

  for(int i=0;i<N_ANTICIPATION_MAX;i++) renormFactorAntic[i]=1;
  //renormFactorAntic    = new double[n_antimax+1];

  renormFactorAntic[0] = gamma;
	
	//das ist nur die korrektur der renormalisierung !!
	for(int j=1;j<n_antimax;j++){
		double sum=0;
		for (int i=1; i<=j; i++){sum += 1./SQR(i*s_stretch);}
		renormFactorAntic[j] = gamma/sqrt(sum); 
  }
	
	
  if(true){
    cout<<"HDM: gamma = "<<gamma<<", 1/gamma="<<(1/gamma)<<endl;
    for(int j=0;j<n_antimax;j++){
	    cout<<"Inverse for Renormatisation: renormFactorAntic["<<j<<"] = "
	        <<renormFactorAntic[j]<<endl;
	  }
		//exit(-1);
  }
  
  //#################################################################

  // KEIN calc_eq hier!!    

} // HDM::get_modelparams


//################################################################
void HDM::calc_eq()
//################################################################

    // Calculates equilibrium velocity  with finite s0
    // and free-acc exponent delta
    // uses numeric iteration procedure and
    // calculates THE WHOLE FIELD veq

  // output:   Arhomax, Atab[], vfactab[] (for calc_rhs! vfactab only hdcorr) 
  // veqtab[]                             (only needed for output),
  // Qmax, rhoQmax                        (only needed for BC), 
  // rho_vtab, rho_freetab, rho_congtab   (only needed for BC)

{

  if(false){cout << "\nin HDM.calc_eq()"<<endl;}

   // Find equilibrium velocities veqtab[ir] with simple relaxation
    // method: Just model for homogeneous traffic solved for 
    // the velocity v_it of one arbitrary vehicle
    //  (no brain, but stable and simple method...)

    double v_it=v0;           // variable of the relaxation equation
    int    itmax      = 100;  // number of iteration steps in each relaxation
    double dtmax      = 2;    // iteration time step (in s) changes from
    double dtmin      = 0.01; // dtmin (rho=rhomax) to dtmax (rho=0)

    // start with rho=0

    veqtab[0]    = v0;
    v0loc        = v0; // used in accTwoVehInt
		//renorm for identical fundDia as IDM 
    Tloc         = T*renormFactorAntic[1];  // used in accTwoVehInt
		s0           *= renormFactorAntic[1];
		s1           *= renormFactorAntic[1];
    aloc         = a;  // used in accTwoVehInt 

    double fact  = 1./(v2rel-v1rel);

    for(int ir=1; ir<=NRHO; ir++)
    {
      double rho = rhomax*ir/NRHO;
      double s   = max(1./rho - 1./rhomax, 0.01);

      // start iteration with equilibrium velocity for the previous density
      v_it             = veqtab[ir-1];
      double sigma=max(min(fact*(v_it/v0-v1rel), 1.), 0.); 
      double alpha_v0_res =alpha_v0_resmin+sigma*(1-alpha_v0_resmin);
      double alpha_T_res  =alpha_T_resmax +sigma*(1-alpha_T_resmax);
      double alpha_a_res  =alpha_a_resmin +sigma*(1-alpha_a_resmin);
      v0loc      *= alpha_v0_res;
      Tloc       *= alpha_T_res; //Use renorm for only one interaction partner !! (oct06)
      aloc       *= alpha_a_res;
      if(false)cout <<" rho="<<rho<<" v_it="<<v_it
		    <<" fact="<<fact<<" v1rel="<<v1rel<<" sigma="<<sigma<<endl;
      if(false) cout << endl;

      //!! Achtung brauche equil. fuer BC !
      for (int it=1; it<=itmax; it++){
	      double acc   = accTwoVehInt(s,v_it,0., 0.,T_react)+accFree(v_it);
	      double dtloc = dtmax*v_it/v0 + dtmin; // it. step in [dtmin,dtmax]

	      // actual relaxation 
	  
     	  v_it            += dtloc*acc;
	      if((v_it<0)||(s<s0)) v_it=0;
	  
     	  if(false){ 
	        cout << "rho="<< rho<< " s="<<s
		           <<" v0="<<v0<<" acc="<<acc<<" v_it="<<v_it<<endl;
	      }
	   }
      veqtab[ir] = v_it;
    }

    Tloc    = T/renormFactorAntic[1];  // used in accTwoVehInt
		s0     /= renormFactorAntic[1];
		s1     /= renormFactorAntic[1];
    calc_rhoQmax();  // Qmax, rhoQmax 


} // HDM::calc_eq()





//################################################################
// acceleration due to isolated 2-veh interaction
// !!! local values aloc,v0loc, Tloc are used !!
//################################################################

double HDM::accFree(double v)
{
  return (max(-10., aloc * ( 1.- pow((v/v0loc),delta))));
}

double HDM::accTwoVehInt(double s, double v, double dv, double a, double T_react_loc)
{
  //arne: T_react also local (the last vehicles in array are excluded!)

  // !!! local values aloc,v0loc, Tloc are used !!
  // a is own (old) vehicle acceleration

  //cout<<"HDM::accTwoVehInt input s="<<s<<", v="<<v<<", dv="<<dv<<", a="<<a<<endl;
  //  cout<<"HDM::accTwoVehInt parameter aloc="<<aloc<<", Tloc="<<Tloc<<", v0loc="<<v0loc<<endl;

  //arne: feb 06: general anticipation with n_temp_antic number, default is 1

  double diff_s_antic= -dv*n_temp_antic*T_react_loc; // s Anticipation a la Davis
  //double diff_s_antic=0;           // no s anticipation
  
  double diff_v_antic= a*n_temp_antic*T_react_loc; // v Anticipation a la Treiber
  //double diff_v_antic=0;           // no v anticipation
  
  //double diff_dv_antic=a*T_react_loc; // dv Anticipation
  //double diff_dv_antic=0;           // no dv anticipation (better results!)
  
  double s_antic  = max(s+diff_s_antic, 0.5*s0);
  double v_antic  = max(v+diff_v_antic, 0.);
  double dv_antic = dv;
  //double dv_antic = dv+diff_dv_antic;
  
  double sstarmin_vT = 0.2; // min. sstar in units of v*T
  
  // IDM acceleration

  double sstar  = s0 + Tloc*v_antic + s1*sqrt((v_antic+0.0001)/v0loc)
                     + 0.5*v_antic*dv_antic /sqrt(aloc*b);
  
  if(sstar<s0+sstarmin_vT*v_antic*Tloc)
    {
      sstar=s0+sstarmin_vT*v_antic*Tloc;
    }

    if(v0loc<=0){
			cerr<<" HDM.accTwoVehInt: >Warning: v0="<<v0<<" v0loc="<<v0loc
	    			<<" <=0  "<<endl;
    }

    if(false){
	   cout <<"HDM,accTwoVehInt:\n"
	     <<" v0loc="<<v0loc
	     <<" aloc="<<aloc<<" sstar="<<sstar
			 <<" sActual ="<<s
	     <<" v_antic="<<v_antic
	     <<" s1*sqrt((v_antic+0.0001)/(v0loc+0.1))="
	     <<(s1*sqrt((v_antic+0.0001)/v0loc))
	     <<endl;
      }
    
    return (-aloc*SQR(sstar/s_antic));

} // accTwoVehInt



//################################################################
// HDM-driver model; reaction time implemented by passing
// OLD dynamic fields, calculated in RoadSection

//double HDM::acc(int it, int iveh, int imin, int imax,
//		  double xvehOld[], double length[], double vvehOld[], double avehOld[], 
//		  double alpha_v0, double alpha_T,
//		  double err_s, double err_dv)


double HDM::acc(int it, int iveh, int imin, int imax,
		double alpha_v0, double alpha_T,
		const CyclicBuffer* const cyclicBuf)
{
	
	//if(iveh<5) cout<<"iveh="<<iveh<<",  imin="<<imin<<", imax="<<imax<<endl;
	
  //hack: das ist auch so in alter version, aber direkt in RoadSection!!!
  //jedes veh besorgt sich selbstaendig die vergangenheit
  //problem bei upstream inflow mit reaktionszeit, deshalb hier ausgeschlosssen
  //in FloatingCars werden diese daten auskommentiert!

  //achtung, bei fluktuationen gibt es auch noch in Vehicle eine verwendung 
  //von T_React !!!
	
  double T_react_loc = (iveh>=imax-NVEH_DELAY_EXCLUDED && cyclicBuf->get_x(iveh,it,0)< DX_DELAY_EXCLUDED_M) ? 0 : T_react;


  //test for uncorrelated reaction time
  //double T_react_loc = (iveh>=imax-NVEH_DELAY_EXCLUDED) ? 0 : uniform(T_react-0.3,T_react+0.3);

  //cout<<"HDM acc....\n";

  //double sOld = xvehOld[iveh-1] -length[iveh-1]-xvehOld[iveh];
  //double vOld = vvehOld[iveh];
  //double aOld = avehOld[iveh]; 

  //input variables are time delayed !!!!

  double sOld = cyclicBuf->get_s(iveh,it,T_react_loc); //net distance to leader
  double vOld = cyclicBuf->get_v(iveh,it,T_react_loc);
  double aOld = cyclicBuf->get_a(iveh,it,T_react_loc);

  if(false && iveh==2)
    {
      cout<<"HDM.acc  it="<<it<<" t="<<it*dt<<", iveh="<<iveh
	  <<", T_react="<<T_react_loc
	  <<", sActual="<<cyclicBuf->get_s(iveh)<<", sOld="<<sOld
	  <<", vActual="<<cyclicBuf->get_v(iveh)<<", vOld="<<vOld
	  <<", aActual="<<cyclicBuf->get_a(iveh)<<", aOld="<<aOld<<endl;
    }

  //##################################################
  // space dependencies modelled by alpha_v0, alpha_T
  //##################################################
  
  Tloc  = alpha_T*T; 
  v0loc=min(alpha_v0*v0, speedlimit);   // martin mai08
    aloc  = a;  

  //cout<<"Tloc="<<Tloc<<", alpha_T="<<alpha_T<<", T="<<T<<endl;

  //##################################################
  // v-history dependencies by resignation effect
  // influence (multiplicatively, i.e., independently)
  // same parameters Tloc,v0loc,aloc
  //##################################################
  
  double fact  = 1./(v2rel-v1rel);
  double sigma = max(min(fact*(vOld/v0loc-v1rel), 1.), 0.); 
  double alpha_a_res  = alpha_a_resmin +sigma*(1-alpha_a_resmin);
  double alpha_v0_res = alpha_v0_resmin+sigma*(1-alpha_v0_resmin);
  double alpha_T_res  = alpha_T_resmax +sigma*(1-alpha_T_resmax);
  
  alpha_v0dyn += beta*(alpha_v0_res-alpha_v0dyn);
  v0loc       *= alpha_v0dyn;
  
  alpha_Tdyn += beta*(alpha_T_res-alpha_Tdyn);
  Tloc       *= alpha_Tdyn;

  alpha_adyn  += beta*(alpha_a_res-alpha_adyn);
  aloc        *= alpha_adyn;
  
  if(false){
      cout<<"HDM.acc: it="<<it<<" iveh="<<iveh
	  <<" sigma="<<sigma
	  <<" v0="<<v0
	  <<" alpha_v0="<<alpha_v0
	  <<" alpha_v0_res="<<alpha_v0_res
	  <<" alpha_v0dyn="<<alpha_v0dyn
	  <<endl;
    }
  
  
  //#############################################################
  // Nonlocal interaction: summing up resulting 2-veh accelerations
  //#############################################################
  
  
  //Sep06: int n_anti_actual = (n_antimax<(iveh-imin)) ? n_antimax : (iveh-imin);
	//bug oct 06 (arne): first vehicle (imin=0) is not updated but serves as interaction
	//partner. So, the re-renormalization affects fully the vehicles iveh=1,2 (only one interaction partner)
	
	int n_anti_actual = (n_antimax<(iveh-(imin+1))) ? n_antimax : max((iveh-(imin+1)),1);

  //renormalization compensates for limited number of leaders of most downstream vehicles 

	bool withRenorm=true;
			
  if( n_anti_actual<n_antimax && withRenorm ){
    Tloc *= renormFactorAntic[n_anti_actual];
    s0   *= renormFactorAntic[n_anti_actual];
    s1   *= renormFactorAntic[n_anti_actual];  
  }

  double acc  = accFree(vOld);
  //if(iveh<5) cout<<"iveh="<<iveh<<" n_anti_actual = "<<n_anti_actual<<", Tloc="<<Tloc<<", alpha_T="<<alpha_T<<", T="<<T<<endl;
	
	//interaction partner 1...n_anti_max
  for (int j=1; (j<=n_anti_actual); j++){
    
		if(j>1){
	    sOld += s_stretch*(cyclicBuf->get_s(iveh-j+1,it,T_react_loc));
	  }
      
    double dvOld = vOld-cyclicBuf->get_v(iveh-j,it,T_react_loc);
         
	  //if(iveh<5)cout<<"accTwoVehInt T="<<T<<", Tloc="<<Tloc<<", s0="<<s0<<", s1="<<s1<<endl;
		
    if(sOld<smax){
	    acc += accTwoVehInt(sOld, vOld, dvOld, aOld, T_react_loc);
			//if(iveh<5)cout<<"it="<<it<<" accTwoVehInt= "<<accTwoVehInt(sOld,vOld,dvOld,aOld, T_react_loc)<<endl;
	  }
    else{
	   //cout <<"HDM.acc: iveh = "<<iveh<<" sOld="<<sOld<<" maxDistExceeded=true!"<<endl;
    }
  }
  
	//if(iveh<5)cout<<" acc = "<<acc<<endl;
	
  if( n_anti_actual<n_antimax && withRenorm ){
    Tloc /= renormFactorAntic[n_anti_actual];
    s0   /= renormFactorAntic[n_anti_actual];
    s1   /= renormFactorAntic[n_anti_actual];  
  }
  
  //######################## check for errors ####################

  if(!((acc>-100000)&&(acc<10))){ 
      cout <<"HDM: Warning: acc="<<acc<<" too large"<<endl;
      cout<<"it="<<it<<" iveh="<<iveh<<" n_anti_actual = "<<n_anti_actual<<endl;
			cout<<"n_anti_actual="<<n_anti_actual<<", renormFactorAntic[n_anti_actual]="<<renormFactorAntic[n_anti_actual]<<endl;
      for (int j=0; j<n_anti_actual; j++){
				double sOld      = cyclicBuf->get_s(iveh-j,it,T_react_loc);
				double vOld      = cyclicBuf->get_v(iveh,it,T_react_loc);
				double dvOld     = vOld - cyclicBuf->get_v(iveh-j-1,it,T_react_loc);
				double aOld      = cyclicBuf->get_a(iveh,it,T_react_loc);
				double a_leadOld = cyclicBuf->get_a(iveh-j-1,it,T_react_loc);
				cout<<"  j="<<j<<" sOld="<<sOld<<" xOld="<<cyclicBuf->get_x(iveh,it,T_react_loc)
	      			<<" v0="<<v0<<" vOld=vvehOld[iveh]="<<cyclicBuf->get_v(iveh,it,T_react_loc)
						<<" dvOld="<<dvOld
						<<" aOld="<<aOld
						<<" a_leadOld="<<a_leadOld
						<<"\n  reconstructing accTwoVehInt:\n";
				cout <<" accTwoVehInt="<<accTwoVehInt(sOld,vOld,dvOld,aOld,T_react_loc)<<endl;
			}
      	cout <<endl;
    }

  //if((v0loc<0.01)&& (vOld>0.05)){cout <<"HDM: v0loc="<<v0loc<<" v="<<vOld<<endl;}

  //######################## </check> ####################
  //if(iveh==1)cout<<"it="<<it<<" acc="<<max(acc, -bmax)<<endl;
  return(max(acc, -bmax));
}

#endif // HDM_CC
