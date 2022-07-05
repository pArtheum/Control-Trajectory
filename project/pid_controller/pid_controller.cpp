/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
               double output_lim_mini) {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  min_lim = output_lim_mini;
  max_lim = output_lim_maxi;
  delta_time = 0.0;
}

void PID::UpdateError(double cte) {
  if (delta_time > 0) {
    d_error = (cte - p_error) / delta_time;
  } else {
    d_error = 0.0;
  }
  d_error = delta_time > 0 ? (cte - p_error) / delta_time : 0.0;
  p_error = cte;
  i_error += cte * delta_time;
}

double PID::TotalError() {
  double control;
  control = (Kp * p_error) + (Kd * d_error) + (Ki * i_error);
  if (control < min_lim) {
    control = min_lim;
  }
  if (control > max_lim) {
    control = max_lim;
  }
  return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  delta_time = new_delta_time;
  return delta_time;
}