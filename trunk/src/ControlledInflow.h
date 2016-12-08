#ifndef CTRL_H
#define CTRL_H


class ControlledInflow
{
 public: 
  ControlledInflow();
  void update(double Qin, double dt);
  double getinflow();
  void setQcap(double Qcap_invh);
  void incrWaitVeh();
  void write_in_log(char projectName[], int it, double dt);

 private:
  double Qcap,Qin;    //(1/s)
  double n_wait;
  double inflow;
};


#endif
