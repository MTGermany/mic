#ifndef DETECTOR_H
#define DETECTOR_H

#include "ProjectParams.h"
#include "Vehicle.h"

class Detector{
 public:
  Detector();
	virtual ~Detector();
  Detector(int x_m, int avgInterval_sec, 
           ProjectParams* proj, const char* projectName);

  void writeAggregatedData(int it);
  void writeSingleVehData(int it, double v_single, 
			  double l_single,
			  double Tbrutto,  double v_prev, 
                          double l_prev, int type, int setNumber, double sreal);

  void update(int it, Vehicle veh[], int imin, int imax);
  int getVehCount(){return total_sum_veh;}

	
	
 private:
  //ProjectParams* proj;
  double tStart;

  char projectName[199];
  char aggregate_fname[199];
  char single_fname[199];

  double time_lastPassage;
  int    iveh_lastPassage;
  double v_single;
  double minTimeHeadway;  // minimum time headway between two vehicles (s)
  double dt;
  int    avgInterval;
  int    pos;
  int    sum_veh;
  int    total_sum_veh;
  double sum_v;
  double sum_invv;
  double sum_invq; // MT jan11
	
	FILE* FH_single;
	FILE* FH_det;
	
	
	
};

#endif // DETECTOR_H
