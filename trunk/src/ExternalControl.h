#ifndef EXTERNALCONTROL_H
#define EXTERNALCONTROL_H

class ExternalControl{ 

 public:

  ExternalControl();
  ExternalControl(char* projectName, int number, int vehIndex);
  double getAcc(double time_s); 
  double getVel(double time_s); 
  double getGapBack(double time_s); 
  double getGapBackJump(double time_s); 
  double getVelBack(double time_s); 
  bool newTargetDetected(double time_s);
  int get_vehIndex(){return vehIndex;} 
  bool ctrlByVel(){return (useVel||useVelBosch);}
  bool ctrlByAcc(){return useAcc;}

 private:
  static const int NDATAMAX=10001;
  int nCtrlFiles;   // number of external control files (should be exactly 1)
  bool useVel;      // prescribe v in generic format (.vlead%i)
  bool useVelBosch; // prescribe v in Bosch format (.vleadBosch%i)
  bool useAcc;      // prescribing acceleration (.brake%i)
  bool useJump;     // prescribing one-off jumps and speed changes

  int vehIndex;   // vehicle index from .leadvehs in RoadSection; 
                  // only informative function here
  int    n_times;
  double times[NDATAMAX];
  double dtData;
  double tLastTargetEntered;
  double acc[NDATAMAX];
  double vel[NDATAMAX];
  double gapBack[NDATAMAX];
  double velBack[NDATAMAX];
  double s_jump[NDATAMAX];
  double v_jump[NDATAMAX];
  double v0_jump[NDATAMAX];
};

#endif // EXTERNALCONTROL_H
