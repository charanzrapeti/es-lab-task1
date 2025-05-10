/**
 * @file simulator.cpp
 * @brief Implementation file for the Simulator class.
 *
 * This file contains the implementation of the Simulator class, which simulates
 * the behavior of an inverted pendulum system. It includes the definition of
 * member functions for running the simulation, updating simulation parameters,
 * and resetting the simulator to its initial state.
 *
 */

#include "simulator.h"
#include "controller.h"
#include <chrono>
#include <cmath>
#include <experimental/random>
#include <iostream>
#include <mutex>
#include <thread>

void Simulator::run_simulator() {

  if (!g_start) {
    std::unique_lock<std::mutex> lock(g_start_mutex);
    g_start_cv.wait(lock);
  }
  int j = 0; // temp index
  int delay_index = 0;
  while (T < m_params.simulation_time) {
    if (g_pause) {
      std::unique_lock<std::mutex> lock(g_pause_mutex);
      g_pause_cv.wait(lock);
    }
    {
      // parameters cannot be updated in the middle of time step
      std::lock_guard<std::mutex> lock(g_start_mutex);

      // Convert delay (microseconds) to number of steps
      int delay_steps = static_cast<int>(m_params.delay / (m_params.delta_t * 1e6));

      // Apply jitter
      int jitter_steps = std::experimental::randint(-m_params.jitter, m_params.jitter) / static_cast<int>(m_params.delta_t * 1e6);

      // Compute total delay index
      delay_index = (i - delay_steps - jitter_steps) % buffer_size;

      // Wrap-around (circular buffer) if negative
      if (delay_index < 0)
          delay_index += buffer_size;

      // Ensure it's within bounds
      delay_index = delay_index % buffer_size;


      error = m_params.ref_angle - theta.at(delay_index);
      F = m_controller->output(-error);

      // new values for theata based on state of last time step
      j = (i + 1) % buffer_size;
      theta_dot[1] = theta_dot[0] + m_params.delta_t * theta_dot_dot[0];
      theta.at(j) = theta.at(i) + m_params.delta_t * theta_dot[0];

      if (std::abs(theta[j]) > M_PI) {
        theta.at(j) =
            theta.at(j) - (theta.at(j) / std::abs(theta.at(j))) * 2 * M_PI;
      }

      x_dot[1] = x_dot[0] + m_params.delta_t * x_dot_dot[0];
      x[1] = x[0] + m_params.delta_t * x_dot[0];

      A = c_ml * std::cos(theta.at(j));
      b = c_ml * std::cos(theta.at(j));
      C = -c_ml * std::pow(theta_dot[1], 2) * std::sin(theta.at(j)) - F;
      c = c_ml * -1 * m_params.g * std::sin(theta[j]);

      x_dot_dot[1] = (A * c - a * C) / (a * B - A * b);

      theta_dot_dot[1] = -(c + b * x_dot_dot[1]) / a;

      theta_dot[0] = theta_dot[1];
      theta_dot_dot[0] = theta_dot_dot[1];

      // new values for x based on state of last time step
      x_dot[0] = x_dot[1];
      x[0] = x[1];
      x_dot_dot[0] = x_dot_dot[1];

      T += m_params.delta_t;
      i = j;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
}
void Simulator::reset_simulator() {
  T = 0;
  F = 0;
  theta.fill(0);            // angle of pendulum with respect to vertical
  theta.at(0) = M_PI_4 / 8; // starting angle
  theta_dot = {0, 0};
  theta_dot_dot = {0, 0};

  i = 0;
  error = 0;

  x = {0, 0}; // position of cart
  x_dot = {0, 0};
  x_dot_dot = {0, 0};
  m_controller->reset();
}
void Simulator::update_params(double ref, int delay, int jitter) {
  std::lock_guard<std::mutex> lock(m_params_mutex);
  m_params.ref_angle = ref;
  m_params.delay = delay;
  m_params.jitter = jitter;
}

