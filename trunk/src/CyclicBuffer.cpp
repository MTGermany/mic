// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
#include <fstream>
using namespace std;

#include "CyclicBuffer.h"
#include "Vehicle.h"

//#define __OUTPUT__
#ifdef __OUTPUT__
   #define OUTPUT(code) code
#else
   #define OUTPUT(code) /* - */
#endif

//Initalisierung:
const double CyclicBuffer::MAX_T_REACT_S = 5;

// #############################################################
// Constructor 1: system without reaction times
// #############################################################

CyclicBuffer::CyclicBuffer(const Vehicle* const veh, int imin, int imax)
{
  initialization();
  someHaveReactionTime=false;
  nt_past=0; 

  cout<<"********************************************************"<<endl;
  cout<<"CyclicBuffer constructor (1) without delay !!!"<<endl;
  cout<<"********************************************************"<<endl;
  
}



// #############################################################
// Constructor 2: system with one constant/fixed reaction time
// contraint allows a precalculation of common past state ==> faster!
// #############################################################

CyclicBuffer::CyclicBuffer(double max_fix_T_react, double dt,
			   const Vehicle* const veh, int imin, int imax)
{ 
  initialization();
  someHaveReactionTime=true;
  with_fixT_react=true;
 
  //max size of cyclic buffer:
  this->dt      = dt;
  this->maxT_react = max_fix_T_react;
  

  //gegenwart ist auch im buffer!!! deshalb brauchen wir noch eine spalte mehr.
  this->nt_past = static_cast<int>(this->maxT_react/dt)+2;  
  
  cout<<"********************************************************"<<endl;
  cout<<"CyclicBuffer constructor (2) with fixed reaction time!\n";
  cout<<"dt="<<dt<<", maxT_react="<<this->maxT_react<<", nt_past = "<<nt_past<<endl;
  cout<<"with_fixT_react = "<<((with_fixT_react)? "true":"false")<<endl;
  cout<<"NVEHMAX= "<<NVEHMAX<<endl;
  cout<<"********************************************************"<<endl;

  init_buffer_matrix();
  initializeBuffer(veh, imin, imax);

  xOld = new double[NVEHMAX];    
  vOld = new double[NVEHMAX];
  aOld = new double[NVEHMAX];
  lOld = new double[NVEHMAX];

  cout<<"exit constructor"<<endl;
}



// #############################################################
// Constructor 3: system with distributed reaction times
// past state is calculated on individual basis ==> slower!
// #############################################################

CyclicBuffer::CyclicBuffer(double dt, const Vehicle* const veh, int imin, int imax)
{ 
  initialization();
  someHaveReactionTime=true;

  //max size of cyclic buffer:
  this->dt      = dt;
  this->maxT_react = MAX_T_REACT_S;

  this->nt_past = static_cast<int>(this->maxT_react/dt)+2;  
  //gegenwart ist auch im buffer!!! deshalb brauchen wir noch eine spalte mehr.


  cout<<"********************************************************"<<endl;
  cout<<"CyclicBuffer constructor (3): individual (distributed) reaction times\n";
  cout<<"dt="<<dt<<", maxT_react="<<this->maxT_react<<", nt_past = "<<nt_past<<endl;
  cout<<"with_fixT_react = "<<((with_fixT_react)? "true":"false")<<endl;
  cout<<"NVEHMAX= "<<NVEHMAX<<endl;
  cout<<"********************************************************"<<endl;

  init_buffer_matrix();
  initializeBuffer(veh, imin, imax);

  cout<<"exit constructor"<<endl;
}


void CyclicBuffer::initialization()
{
  err_x=0;
  err_v=0; 
  iveh_with_x_error=0;
  iveh_with_v_error=0; 

  xBuffer=0; //pointer init.
  vBuffer=0;
  aBuffer=0;
  lBuffer=0;
  
  xOld=0; //pointer init.
  vOld=0;
  aOld=0;
  lOld=0;

 
  xVeh = new double[NVEHMAX];
  vVeh = new double[NVEHMAX]; 
  aVeh = new double[NVEHMAX]; 
  //aVehDes = new double[NVEHMAX]; //not yet implemented (2015-02)
  sVeh = new double[NVEHMAX];
  lVeh = new double[NVEHMAX];

}


void CyclicBuffer::init_buffer_matrix()
{
  //rows/zeilen: NVEHMAX (first index)
  //cols/spalten: nt_past (second index) x[iveh][it_past]

  xBuffer = new double*[NVEHMAX];
  vBuffer = new double*[NVEHMAX];
  aBuffer = new double*[NVEHMAX];
  lBuffer = new double*[NVEHMAX]; 
 

  for(int i=0; i<NVEHMAX; i++)
     {
       xBuffer[i] = new double[nt_past]; 
       vBuffer[i] = new double[nt_past];
       aBuffer[i] = new double[nt_past];
       lBuffer[i] = new double[nt_past];
     } 


}


// #############################################################
// Deconstructor
// #############################################################

CyclicBuffer::~CyclicBuffer()
{
  cout<<"deconstructor CyclicBuffer ...\n";

  //delete matrix

  for(int i=0; i<NVEHMAX; i++)
    {
      if(xBuffer && xBuffer[i]) delete[] xBuffer[i];
      if(xBuffer && vBuffer[i]) delete[] vBuffer[i];
      if(aBuffer && aBuffer[i]) delete[] aBuffer[i]; 
      if(lBuffer && lBuffer[i]) delete[] lBuffer[i];
    }  

  if(xBuffer) delete[] xBuffer;
  if(vBuffer) delete[] vBuffer;
  if(aBuffer) delete[] aBuffer;
  if(lBuffer) delete[] lBuffer;

  //delete one dimensional arrays:

  if(xVeh) delete[] xVeh;
  if(vVeh) delete[] vVeh;
  if(aVeh) delete[] aVeh;
  //if(aVehDes) delete[] aVehDes;//not yet implemented (2015-02)
  if(lVeh) delete[] lVeh;
  if(sVeh) delete[] sVeh;

  if(xOld) delete[] xOld;
  if(vOld) delete[] vOld;
  if(aOld) delete[] aOld;
  if(lOld) delete[] lOld;

}


// #############################################################
// Initialization of buffer
// #############################################################

void CyclicBuffer::initializeBuffer(const Vehicle* const veh, int imin, int imax)
{ 
  if(someHaveReactionTime)
    {
      for(int iveh=imin; iveh<=imax; iveh++)
	{
	  OUTPUT(cout<<"initializeBuffer: iveh="<<iveh<<", imax="<<imax<<" length = "<<lVeh[iveh]<<endl;);
	  
	  for(int it_past=0; it_past<nt_past; it_past++)
	    {
	      xBuffer[iveh][it_past] = veh[iveh].getPos();
	      vBuffer[iveh][it_past] = veh[iveh].getVel();
	      aBuffer[iveh][it_past] = veh[iveh].getAcc(); 
	      lBuffer[iveh][it_past] = veh[iveh].getLength();    

	      OUTPUT(cout<<"initializeBuffer:"
		     <<"\txBuffer["<<iveh<<"]["<<it_past<<"]="<<xBuffer[iveh][it_past]
		     <<"\tvBuffer["<<iveh<<"]["<<it_past<<"]="<<vBuffer[iveh][it_past]
		     <<"\taBuffer["<<iveh<<"]["<<it_past<<"]="<<aBuffer[iveh][it_past]
		     <<"\tlBuffer["<<iveh<<"]["<<it_past<<"]="<<lBuffer[iveh][it_past]<<endl;);
	    }
	}
    }
}

// #############################################################
// Buffer update: 
//
// in erster version: calc past state for given overall reaction time !!!
//
// #############################################################

void CyclicBuffer::updatePresence(int it, const Vehicle* const veh, int imin, int imax)
{

  OUTPUT(cout
	 <<"\nupdatePresence: it="<<it<<", imin="<<imin<<", imax="<<imax
	 <<" xVeh[imin]="<<veh[imin].getPos()<<",  vVeh[imin]="<<veh[imin].getVel()<<endl;)

  //current state: merge/diverge (shifts everything)
  for(int iveh=imin; iveh<=imax; iveh++)
    {
      xVeh[iveh]   = veh[iveh].getPos();
      vVeh[iveh]   = veh[iveh].getVel(); 
      aVeh[iveh]   = veh[iveh].getAcc();
      //aVehDes[iveh]   = veh[iveh].getAccDes();//not yet implemented (2015-02)
      lVeh[iveh]   = veh[iveh].getLength();

      OUTPUT(cout<<"updatePresence: present state "
	         <<"\txVeh["<<iveh<<"]="<<xVeh[iveh]
		 <<"\tvVeh["<<iveh<<"]="<<vVeh[iveh]
		 <<"\taVeh["<<iveh<<"]="<<aVeh[iveh]
	     //<<"\taVehDes["<<iveh<<"]="<<aVehDes[iveh]//not yet implemented (2015-02)
		 <<"\tlVeh["<<iveh<<"]="<<lVeh[iveh]<<endl;);
    }
  
  for(int iveh=imin+1; iveh<=imax; iveh++)
    {
      sVeh[iveh]=veh[iveh-1].getBackPos()-veh[iveh].getPos(); 
    }
  
}

// #############################################################
//
// #############################################################

void CyclicBuffer::updateBuffer(int it, const Vehicle* const veh, int imin, int imax)
{
  if(someHaveReactionTime) 
    {
      OUTPUT(cout
	   <<"\nupdateBuffer: it="<<it<<", imin="<<imin<<", imax="<<imax
	   <<" xVeh[imin]="<<veh[imin].getPos()<<",  vVeh[imin]="<<veh[imin].getVel()<<endl;)
      
      int iCycle=it%nt_past;
      for (int iveh=imin; iveh<=imax; iveh++)
	{
	  xBuffer[iveh][iCycle] = veh[iveh].getPos();
	  vBuffer[iveh][iCycle] = veh[iveh].getVel();
	  aBuffer[iveh][iCycle] = veh[iveh].getAcc(); 
	  lBuffer[iveh][iCycle] = veh[iveh].getLength();    
	  
	  OUTPUT(cout<<"updateBuffer: it="<<it<<", iCycle="<<iCycle
		 <<"\txBuffer["<<iveh<<"]["<<iCycle<<"]="<<xBuffer[iveh][iCycle]
		 <<"\tvBuffer["<<iveh<<"]["<<iCycle<<"]="<<vBuffer[iveh][iCycle]
		 <<"\taBuffer["<<iveh<<"]["<<iCycle<<"]="<<aBuffer[iveh][iCycle]
		 <<"\tlBuffer["<<iveh<<"]["<<iCycle<<"]="<<lBuffer[iveh][iCycle]<<endl;);
	}
      
      // precalculate ONE past state for all vehicles !!! 
      // allows for better performance .... 
      if(with_fixT_react)
	{
	  calcOldState(imin, imax, it, maxT_react, xBuffer, xOld); 
	  calcOldState(imin, imax, it, maxT_react, vBuffer, vOld); 
	  calcOldState(imin, imax, it, maxT_react, aBuffer, aOld); 
	  calcOldState(imin, imax, it, maxT_react, lBuffer, lOld); 
	}
    }
}



// #############################################################
// public: past state access (x and v with errors !)
// (1) x and v with error 
// (2) a and l 
// (3) distance s resulting from x and l
// #############################################################

double CyclicBuffer::get_x(const int& iveh, const int& it, const double& T_react) const
{
  if(T_react<=SMALL_VAL) return get_x(iveh); //nullvergleich nicht zuverlaessig!!!
  
  double x = (with_fixT_react) ? xOld[iveh] : calcOldState(iveh,it,T_react,xBuffer); 
  return( (iveh==iveh_with_x_error) ? x+err_x : x );
}

double CyclicBuffer::get_v(const int& iveh, const int& it, const double& T_react) const
{   
  if(T_react<=SMALL_VAL) return(get_v(iveh));
    
  double v =  (with_fixT_react) ? vOld[iveh] : calcOldState(iveh,it,T_react,vBuffer); 
  return( (iveh==iveh_with_v_error) ? v+err_v : v );
}

// #############################################################


double CyclicBuffer::get_a(const int& iveh, const int& it, const double& T_react) const
{
  if(T_react<=SMALL_VAL) return(get_a(iveh));

  return( (with_fixT_react) ? aOld[iveh] : calcOldState(iveh,it,T_react,aBuffer) );
}



double CyclicBuffer::get_l(const int& iveh, const int& it, const double& T_react) const
{ 
  if(T_react<=SMALL_VAL) return( get_l(iveh) );

  return( (with_fixT_react) ? lOld[iveh] :  calcOldState(iveh,it,T_react,lBuffer) );
}

// #############################################################

double CyclicBuffer::get_s(const int& iveh, const int& it, const double& T_react) const
{
  if(T_react<=SMALL_VAL) return get_s(iveh);

  double s= (with_fixT_react) ? 
    xOld[iveh-1]-lOld[iveh-1]-xOld[iveh] : 
    get_x(iveh-1,it,T_react) - get_l(iveh-1,it,T_react) - get_x(iveh,it,T_react);

  return( s );
}


// #############################################################
// private: calculation of individual past 
// 
// ausgehend von iCycle als Bezugspunkt fuer die gegenwart, wird 
// zurueckgegangen in die vergangenheit
// interpolation fuer inkommensurable reaktionszeiten
//
// der buffer ist zyklisch zeitlich in schritte von dt geordnet.
// #############################################################


double CyclicBuffer::calcOldState(int iveh, int it, double T_react, double* const* const past) const
{
  int iCycle=it%nt_past; //reference for "present" without delay
  int iPastRight = iCycle-static_cast<int>(T_react/dt);
  if(iPastRight<0) iPastRight += nt_past;
  int iPastLeft  = (iPastRight==0) ? nt_past-1 : iPastRight-1;
  double fracPastLeft = T_react/dt - static_cast<int>(T_react/dt);
  double pastState= fracPastLeft*past[iveh][iPastLeft] + (1-fracPastLeft)*past[iveh][iPastRight];

  OUTPUT(cout<<"calcOldState::it="<<it<<", iveh="<<iveh<<", iCycle="<<iCycle
	 <<", iPastRight="<<iPastRight<<", iPastLeft="<<iPastLeft
	 <<", fracPastLeft="<<fracPastLeft<<", pastState="<<pastState<<endl;)

  return pastState;
}

void CyclicBuffer::calcOldState(int imin, int imax, int it, double T_react, 
				double* const* const past, double *old) const
{
  int iCycle=it%nt_past; //reference for "present" without delay
  int iPastRight = iCycle-static_cast<int>(T_react/dt);
  if(iPastRight<0) iPastRight += nt_past;
  int iPastLeft  = (iPastRight==0) ? nt_past-1 : iPastRight-1;
  double fracPastLeft = T_react/dt - static_cast<int>(T_react/dt);

  for (int iveh=imin; iveh<=imax; iveh++)
    {
      old[iveh] = fracPastLeft*past[iveh][iPastLeft] + (1-fracPastLeft)*past[iveh][iPastRight];
      
      OUTPUT(cout<<"calcOldState::it="<<it<<", iveh="<<iveh<<", iCycle="<<iCycle
	     <<", iPastRight="<<iPastRight<<", iPastLeft="<<iPastLeft
	     <<", fracPastLeft="<<fracPastLeft<<", pastState="<<pastState<<endl;)
    }
}



