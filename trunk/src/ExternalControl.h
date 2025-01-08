#ifndef EXTERNALCONTROL_H
#define EXTERNALCONTROL_H

// for comments of the 4 control types, see the .cpp file

class ExternalControl{ 

 public:

  ExternalControl();
  ExternalControl(char* projectName, int number, int vehIndex);
  double getAcc(double time_s); 
  double getVel(double time_s); 
  double getAlphaV0(double time_s); 
  double getVelJump();
  double getAlphaV0Jump();
  double getGapBack(double time_s); 
  double getGapBackJump(double time_s); 
  double getVelBack(double time_s); 
  bool newTargetDetected(double time_s);
  bool checkNewJump(double time_s);
  int get_vehIndex(){return vehIndex;} 
  bool ctrlByVel(){return (useVel||useVelBosch);}
  bool ctrlByAlphaV0(){return useAlphaV0;}
  bool ctrlByAcc(){return useAcc;}
  bool ctrlByJump(){return useJump;}

 private:
  static const int NDATAMAX=10001;
  int nCtrlFiles;   // number of external control files (should be exactly 1)
  bool useVel;      // prescribe v in generic format (.vlead%i)
  bool useAlphaV0;       // prescribe alpha_v0 in generic format (.v0lead%i)
  bool useVelBosch; // prescribe v in Bosch format (.vleadBosch%i)
  bool useAcc;      // prescribing acceleration (.brake%i)
  bool useJump;     // prescribing one-off jumps and speed changes
  int vehIndex;   // vehicle index from .leadvehs in RoadSection; 
                  // only informative function here
  int    n_times;
  double times[NDATAMAX];
  double dtData;
  double tLastTargetEntered; // for jumps
  double acc[NDATAMAX];
  double vel[NDATAMAX];
  double alpha_v0[NDATAMAX];
  double gapBack[NDATAMAX]; // only if useVelBosch
  double velBack[NDATAMAX]; // ""
  int i_jump;              // index of the latest jump
  bool newJump;            // useJump: if a new jump happens in this timestep 
  double s_jump[NDATAMAX]; // gapBack for "jump" control if useJump
  double v_jump[NDATAMAX]; // discrete speed jumps of ctrl veh if useJump
  double alpha_v0_jump[NDATAMAX]; // change desired speed (=^ new veh) if useJump
};

#endif // EXTERNALCONTROL_H
