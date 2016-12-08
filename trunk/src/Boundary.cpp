// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
using namespace std;


//own
#include "Boundary.h"
#include "RandomUtils.h"
#include "general.h"
#include "InOut.h"
#include "constants.h"

#include "Vehicle.h"

Boundary::Boundary(){}

Boundary::Boundary(const char* projectName, bool isUpstream, ProjectParams* proj)
{
  sprintf(this->projectName,"%s",projectName);
  this->proj  = proj;
  this->dt = proj->get_dt();
  this->xmax = proj->get_xmax(); 
  //this->fluct = Fluct(fluct);//new Fluct(projectName, proj->get_dt);
  this->choice_BC  = (isUpstream) ? proj->get_choice_BCup() : proj->get_choice_BCdown();
  this->isUpstream = isUpstream;

  nWait=0;
  nInsertedBC=0;
  isCtrl = (isUpstream&&(choice_BC==4));
  ctrl_nWait=0;
  ctrl_Qcap=0;
  char in_fname[199];
  InOut inout;

  if((choice_BC==0)||isCtrl){
     if(isUpstream){sprintf(in_fname,"%s.BCup",projectName);}
     else{sprintf(in_fname,"%s.BCdown",projectName);}
     cout << "Reading BC ...; if segm. fault; NBC too small!!!"<<endl;
     inout.get_array(in_fname, n_times, times, QBC, vBC);

     if(n_times>NBC){
       cerr<<" Boundary: Error: input file "<<in_fname
	   <<" has more than "<<(NBC)<<" data lines;"
	   <<" increase NBC in constants.h or decrease data"<<endl;
       exit(-1);
     }

     //!!! Hack, um .BCl, .BCr inds Format der .BCup, .BCdown Files 
     // zu bringen => neue Files .BCupNew, .BCdownNew; danach umkop!
     bool convertOldBCfiles=false;
     if(convertOldBCfiles){
       char out_fname[199];
       if(isUpstream){sprintf(out_fname,"%s.BCupNew",projectName);}
       else{sprintf(out_fname,"%s.BCdownNew",projectName);}
       for (int it=0; it<n_times; it++){
	 double rho_invkm=QBC[it];
	 QBC[it]=vBC[it];
	 vBC[it]=QBC[it]/rho_invkm;
       }
       inout.write_array(out_fname, n_times, times, QBC, vBC, 
          "t(s)",   "Q(veh/h)",  "v(km/h)");
     }

     // transform to SI
     for (int i=0; i<n_times; i++){
       vBC[i]/=3.6;
       QBC[i]/=3600.;
     }

  }
  else{
    n_times=0;
    times[0]=QBC[0]=vBC[0]=0;
  }

  if(isCtrl){ // flow-controlled IC
     cout<<"Boundary: Cstr: Controlled inflow!, get Qcap ..."<<endl;
     sprintf(in_fname,"%s.Qcap",projectName);
     double QcapDummyArray[1];
     int nArr;
     inout.get_array(in_fname, nArr, QcapDummyArray);
     ctrl_Qcap=QcapDummyArray[0]/3600.;
  }

  if(false){
    cout << "Boundary Cstr: \n";
    for (int i=0; i<n_times; i++){
      cout<<"times["<<times[i]<<"] : "<<"vBC["<<i<<"]="<<vBC[i]<<" QBC["<<i<<"]="<<QBC[i]<<endl;
    }
    //exit(0);
  }

} 

double Boundary::getFlow(double time){
  return( intpextp(times, QBC, n_times, time));
} 
double Boundary::getVel(double time){
  return( intpextp(times, vBC, n_times, time));
} 

void Boundary::set_choiceBC(int new_choiceBC){
  choice_BC = new_choiceBC;
  if(false) cout<<"set choiceBC (5==red traffic Light, 2==Neumann/green Light) = "<<choice_BC<<endl;
} 



 // if up:choice_BC    = {0=Dirichlet,1=Neumann,2=zero,3=period,4=flowControl}
//#####################################################################
// functions max, min als inline in general.h

void Boundary::updateUpstream(Vehicle veh[], int &imin, int &imax,
			      Heterogen* het, double time){
  
  if((choice_BC==0)||isCtrl ){
    
    if(isCtrl){ctrl_update(time);}
    double QBC=(isCtrl) ? ctrl_flow : getFlow(time);
    double vBC=getVel(time);
    nWait += QBC*dt;      
    if(nWait>=1.){ // new vehicle wants to enter 

      // type of new vehicle
       double randomNumber = myRand(); // ~ G(0,1)
       int type            = het->get_type(randomNumber);
       int modelNumber   = het->get_modelNumber(type);
       //cout <<"type="<<type<<" modelNumber="<<modelNumber<<endl;
      double x_lastveh    = veh[imax].getPos();
      double v_lastveh    = veh[imax].getVel();
      double a_lastveh    = veh[imax].getAcc(); //<MT nov06>

      vBC = min(vBC, 1.5*v_lastveh);
      double len_lastveh  = veh[imax].getLength();
      double sFreeMin     = 0.8/(veh[imax].get_rhoQmax()); //!! 0.8/..
      if (modelNumber==11){// Kerner-book p. 407
        double tau=1;
        sFreeMin =v_lastveh*tau;
      }
      if (modelNumber==13){// TianJung ... Treiber paper ASGM (2011)
        sFreeMin =v_lastveh; // eigentlich v0, aber vehicle_getV0() gibt's nicht
      }
      double sFree        = x_lastveh-len_lastveh;

      //arne 18-5-2005
      //achtung, hier wirken sich rundungsfehler aus:
      //bool boundaryIsFree = ( roundDouble(sFree,2) > sFreeMin );

      bool boundaryIsFree = ( sFree > sFreeMin );
  
      if(false){
	cout<<"\nBoundary.updateUpstream:"
	    <<" New vehicle wants to enter! time="<<time<<"\n"
	  //<<" isCtrl="<<isCtrl
	  //<<" imax="<<imax
	  //<<" imin="<<imin
	    <<" x_lastveh="<<x_lastveh
	    <<" v_lastveh="<<v_lastveh
	    <<" len_lastveh="<<len_lastveh
	    <<" QBC="<<QBC
	    <<" vBC="<<vBC
	    <<" nWait="<<nWait
	    <<endl
	    <<" sFreeMin="<<sFreeMin
	    <<" sFree="<<sFree
	    <<" boundaryIsFree="<<boundaryIsFree
	    <<endl;
	//if(time>10){exit(-1);}
      }

    // case of free boundary

      if(boundaryIsFree) {
        imax   ++;   // imax "&" arguments of this funct!
        nInsertedBC ++;  // cannot take imax because of possible ramps
	nWait--;

        if(imax>=NVEHMAX){
	  cerr<<"Boundary::updateUpstream: More than NVEHMAX vehicles"
	      <<" not possible!\n";
	  exit(-1);
	}


         // type neededalready above!
        double x            = vBC*nWait/max(QBC, 0.001);
	if(x>x_lastveh-sFreeMin-len_lastveh){
	  x=x_lastveh-sFreeMin-len_lastveh;
	}
	double rhoenter    = 1. / (x_lastveh - x) ;
	MicroModel *pmodel = het->new_pmodel(type);
        double vmaxeq      = pmodel->get_veq(0.5*rhoenter); //0.5*...
        // <MT nov06>
        double bmax=4;  // max. kinematic deceleration at boundary //!! 5
        double beff=max(0.1, bmax+a_lastveh);
        double vmaxkin   = v_lastveh + sqrt(2*sFree*beff);
	// </MT>

	double v           = min(min(vBC, vmaxeq), vmaxkin);

	veh[imax] = Vehicle(x,v,proj, het->get_fluct(type), pmodel, 
			    het->get_modelNumber(type), het->get_setNumber(type));

        if(false) {
          cout <<"BCup, boundaryIsFree: new veh:"
	   <<" imax = "<<imax
	    // <<" type = "<<type
           <<" x_enter="<<static_cast<int>(x)
           <<" v_enter_kmh="<<static_cast<int>(3.6*v)
           <<" x_lastveh ="<<static_cast<int>(x_lastveh)
	   <<" v_lastveh_kmh ="<<static_cast<int>(3.6*v_lastveh)
	   <<" sFree="<<sFree
	   <<" sFreeMin="<<sFreeMin
	    // <<" rho="<<static_cast<int>(1000*rhoenter)
           <<" nWait="<<nWait
           <<endl;
	}
	//if(imax>35){exit(0);} //!!
      }

     // vehicle wants to enter but upstream boundary is congested
      
      else{  
        // variant 1: Do nothing except of update ctrl_inflow
	// !!!
        if(isCtrl){ctrl_nWait++;}

        if(false){ 
          cout <<"Boundary.updateUpstream: Error: boundary not free!!!!!"
	       <<" time="<<time<<" imax = "<<imax
               <<"\n QBC_invh="<<static_cast<int>(3600*QBC) 
               <<" vBC_kmh="<< static_cast<int>(3.6*vBC)
	       <<"\n x_lastveh ="<<static_cast<int>(veh[imax].getPos())
               <<" v_lastveh ="<<static_cast<int>(veh[imax].getVel())
	       <<" sFree="<<sFree
	       <<" sFreeMin="<<sFreeMin
               <<"\n nWait="<<nWait
               <<" ctrl_nWait="<<ctrl_nWait
               <<endl;

	  //exit(-1);
	}

      }
      
      
    }       // end new vehicle wants to enter 
  }         // end choice_BCup==0 or isCtrl

  else if(choice_BC==1){//Neumann
    double x_last     = veh[imax].getPos();
    double v_last     = veh[imax].getVel();
    double x_lastlast = veh[imax-1].getPos();

    if(x_lastlast<=2.*x_last){
      imax        ++;
      nInsertedBC ++;
      if(imax>=NVEHMAX) error("More than NVEHMAX vehicles not possible!");

      double randomNumber= myRand();//(double) (rand()/((double) RAND_MAX+1.0));
      int type           = het->get_type(randomNumber);
      //bool hasReactTime  = (type==2);
      double x       = 2.*x_last - x_lastlast;
      veh[imax] = Vehicle(x,v_last,proj, het->get_fluct(type),
                          het->new_pmodel(type),
                          het->get_modelNumber(type), 
                          het->get_setNumber(type));
      if(false) cout <<"left_boundary: new imax = "<<imax
           <<" x="<<x<<" v="<<v_last<<endl;
    }
  }

  else if(choice_BC==2){//No entering vehicles
    // no action
  }

  else if(choice_BC==3){//periodic BC
    // no action, since downstream BC triggers action for periodic BC
  }
  else if(choice_BC==4){//Flow-control
    // action at if-branch (choice_BC==0)
  }

  else{ error ("choice_BCup>4 not implemeted!");}

}           // end updateUpstream




//#####################################################################

 // if down:choice_BC    = {0=Dirichlet,1=Neumann,2=zero,3=period,4=block BC,5=full braking}

void Boundary::updateDownstream(Vehicle veh[], int& imin, int& imax,double time){

  if(choice_BC==0){//Dirichlet
    double vBC=getVel(time);  // no QBC necessary


    if( veh[imin].getPos() > xmax) {
      imin++;
      if(false){
      cout <<"Boundary::updateDownstream: veh[imin-1].getPos()="
	   <<veh[imin-1].getPos()
	   <<" imin="<<imin<<" increased!"<<endl;
      }
    } 
    veh[imin].setVel(vBC);
    //veh[imin+1].setVel(vBC);  //?? noetig? side effects mit v-control!
  }

  else if(choice_BC==1){//Neumann

    // just increase counting index 
    // => vehicles with i<imin no longer considered.

    if( veh[imin].getPos() > xmax) {
      imin++;
    } 
  }

  else if(choice_BC==2){//No leaving vehicles

    // following not really effective: first veh. not accelerated in 
    // RoadSection.update;
    // just leave imin=0 and set distance to large value

      //accelerate only first vehicle because of no predicessor
    //      veh[imin].setAcc(2.0);

    //cout<<"kritierum veh[imin].getPos()>xmax = "
    //<<veh[imin].getPos()<<"\t"<<xmax<<endl;

    //Martin dec 04: Auskommentierung!!
    //if(veh[imin].getPos()>=xmax){
	//cout<<"incrementiere imin"<<endl;
	//imin++;
    //} 
      // veh[imin].setAcc(2.0);
    
  }

  else if(choice_BC==3){//Periodic BC; see also RoadSection.update!!
    
    if(veh[imin+1].getPos() > xmax) {
      veh[imax+1]=Vehicle(veh[imin+1]);
      veh[imax+1].setPos(veh[imin+1].getPos()-(xmax-(proj->get_xmin())));
      imin++;
      imax++;
    }
    veh[imin].setAcc(veh[imax].getAcc());
    veh[imin].setVel(veh[imax].getVel());
    veh[imin].setPos(veh[imax].getPos()+(xmax-(proj->get_xmin())));

  }
  else if(choice_BC==4){//Blocking BC: First vehicle stopped at xmax
                       
    if (veh[imin].getPos() > xmax-SMALL_VAL){veh[imin].setVel(0);}
  }

  else if(choice_BC==5){ //full braking of leading vehicle! 
    //because i don't know how to get to bmax in IDM
    //i set bmax to 6 m/s^2!
    //problem: weiss nicht wie ich dauerhaft das angesteuerte FZ zum bremsen bringe!
    //if(veh[imin].getPos() > xmax-SMALL_VAL){
      // cout<<"choice_BC==5 --> full braking of veh imin ="<<imin<<"\n";
      //veh[imin].setAcc(-6.0);
  }


  else{ error ("choice_BCup>5 not implemented!");}
           

} // end updateDownstream



void Boundary::ctrl_update(double time){

  double Qin=getFlow(time);
  if (Qin>=ctrl_Qcap){
    ctrl_flow=ctrl_Qcap;
    ctrl_nWait += (Qin-ctrl_flow)*dt;
  }
  else{
    if(ctrl_nWait>0){
      ctrl_flow = min(ctrl_Qcap, Qin + ctrl_nWait/dt);
      ctrl_nWait+=(Qin-ctrl_flow)*dt;
    } else{ctrl_flow = Qin;}
  }
  //cout << "update: inflow="<<inflow<<endl;
}

void Boundary::ctrl_writeLog(double time){

  static bool First=true;
  char   out_fname[199];

  sprintf(out_fname,"%s.ctrl_log",projectName);
  FILE  *outfile;                  
  outfile     = First ? fopen(out_fname,"w") : fopen(out_fname,"a");
  if (First){
    First=FALSE;
    fprintf(outfile,"Time(min)\tQin(1/h)\tcontr_inflow(1/h)\tn_veh\n");
  }
  if(true){
    fprintf(outfile,"%f\t%f\t%f\t%i\n", time/60, 
       (3600*getFlow(time)), 3600*ctrl_flow, static_cast<int>(ctrl_nWait));
  }
  fclose (outfile);
}

