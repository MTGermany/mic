#ifndef KernerCA_H
#define KernerCA_H


#include "MicroModel.h"
#include "CyclicBuffer.h"

class KernerCA: public MicroModel
{

 public:
  
  KernerCA(){;} //default constructor needed for KernerCA[N] in Heterogen

  KernerCA(const char projectName[], double dt);
  
  //shallow copy of object used in Heterogen.cpp
  //no dynamic memory allocation in MicroModels! 
  //use the default copy-constructor, deconstr and =operator
  //virtual ~KernerCA(){;} //do not overwrite default deconstr.

  double acc(int it, int iveh, int imin, int imax,
	     double alpha_v0, double alpha_T,
	     const CyclicBuffer* const cyclicBuf);

  double accSimple(double s, double v, double dv,
		   double alpha_v0, double alpha_T); 
  void setSpeedlimit(double v0){//<martin mai08>
    cout <<"Warning: KernerCA.setSpeedlimit not yet implemented!"<<endl;
  }


 protected:

  // KernerCA parameters

  double v0;
  double k; //Multiplikator fuer sync-Abstand D=lveh+k*v*tau
  double pb0; //"Troedelwahrsch." for standing vehicles
  double pb1; //  "Troedelwahrsch." for moving vehicles
  double pa1; //"Beschl.=Anti-Troedelwahrsch." falls v<vp
  double pa2; // "Beschl.=Anti-Troedelwahrsch." falls v>=vp
  double vp; // Geschw., ab der weniger "anti-getroedelt" wird (etwa v(ST))
  double lambda; //merging criterium: OK, falls Nettoluecken nach
                    // merge > 1/2*lambda*v
  int nagelSchreck; // if=0, KCA selected (default); if=1, original NSM selected

  double dt;          // from proj.dt

 private:

  void get_modelparams(const char* fname);
  void calc_eq();
};

#endif // KernerCA_H
