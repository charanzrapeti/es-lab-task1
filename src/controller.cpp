/**
 * @file controller.cpp
 * @brief Implementation file for the PIDController class.
 *
 * This file contains the implementation of the PIDController class,
 * which implements a PID (Proportional-Integral-Derivative) controller
 * for controlling the inverted pendulum system.
 *
 * @author [Your Name]
 * @date [Date]
 */

#include "controller.h"
#include <iostream>

PIDController::PIDController() { update_params(kp, ki, kd); }

void PIDController::setClamp(double max_, double min_) {
  max = max_;
  min = min_;
}


double PIDController::output(double error) {
  e2 = e1;
  e1 = e0;
  e0 = error;

  integral += e0;

  u0 = kp * e0 + kd * (e0 - e1) + ki * integral;

  // Debug output
  std::cout << "Output calculation\n";
  std::cout << "U0 is " << u0 << "\n";


  // Clamp after hack
  if (u0 > max) u0 = max;
  else if (u0 <= min) u0 = 1;

  u2 = u1;
  u1 = u0;

  return u0;
}





void PIDController::update_params(double kp_, double ki_, double kd_) {
  kp = kp_;
  ki = ki_;
  kd = kd_;
  std::cout << "Received params " << kp_ << ki_ << kd_ <<  "updated params " << kp << ki << kd << std::endl;
}

void PIDController::reset() {
  e0 = e1 = e2 = 0;
  u0 = u1 = u2 = 0;
  integral = 0;
}

