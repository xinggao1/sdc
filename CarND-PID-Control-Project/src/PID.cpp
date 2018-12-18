#include <cstdio>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  pre_cte = 0;
  i_error = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  d_error = cte - pre_cte;
  pre_cte = cte;
  i_error += cte;
}

double PID::TotalError() {
  double ret =  Kp * p_error + Ki * i_error + Kd * d_error;
  return  ret;

}

