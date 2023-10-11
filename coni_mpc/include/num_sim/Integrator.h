/*    coni_mpc
 *    CoNi-MPC: Cooperative Non-inertial Frame Based Model Predictive Control
 *    Copyright (C) 2023 Baozhe Zhang, 
 *    Fast Lab, Huzhou Institute of Zhejiang University
 *  
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <memory>

#include <boost/numeric/odeint.hpp>

#include <num_sim/System.h>

namespace num_sim
{

template <size_t STATE_DIM, size_t CONTROL_DIM>
class Integrator
{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using State_t = typename System<STATE_DIM, CONTROL_DIM>::State_t;
  using Control_t = typename System<STATE_DIM, CONTROL_DIM>::Control_t;
  using StateArray_t = typename std::vector<State_t>;
  using TimeArray_t = typename std::vector<double>;

  struct Observer
  {
    // logging array
    StateArray_t &state_array_;
    TimeArray_t &time_array_;

    Observer(StateArray_t &state_array, TimeArray_t &time_array) : 
        state_array_(state_array), 
        time_array_(time_array)
    {
    }

    void reset()
    {
      if (state_array_.empty() && time_array_.empty())
        return;
      state_array_.clear();
      time_array_.clear();
    }

    // observer logging function
    void operator()(const State_t &x, double t)
    {
      state_array_.push_back(x);
      time_array_.push_back(t);
    } 
  };

  // constructor
  Integrator(const std::shared_ptr<System<STATE_DIM, CONTROL_DIM>> &system_ptr) : 
      system_ptr_(system_ptr), 
      observer(state_array_, time_array_)
  {
    system_function_ = 
        [this](const State_t &x, State_t &dxdt, double t)
        {
          system_ptr_->updateDynamics(x, t, dxdt);
        };
    rk4_stepper_ptr_ = std::make_shared<boost::numeric::odeint::runge_kutta4<State_t, double, State_t, double, boost::numeric::odeint::vector_space_algebra>>();
  }

  StateArray_t getStateArray() { return state_array_; }
  TimeArray_t getTimeArray() { return time_array_; }


  // update in each iteration
  void integrate_n_steps(State_t &state, 
      double start_time, size_t num_steps, double dt)
  {
    observer.reset();
    boost::numeric::odeint::integrate_n_steps(*rk4_stepper_ptr_, system_function_, state, start_time, dt, num_steps, observer);
  }


 private: 

  // system instance
  std::shared_ptr<System<STATE_DIM, CONTROL_DIM>> system_ptr_;

  // function to odeint to integrate
  std::function<void(const State_t &, State_t &, double)> system_function_;

  // stepper instance
  std::shared_ptr<boost::numeric::odeint::runge_kutta4<
      State_t, double, State_t, double, boost::numeric::odeint::vector_space_algebra>> rk4_stepper_ptr_;

  // logging array
  StateArray_t state_array_;
  TimeArray_t time_array_;
  Observer observer;
};


} // namespace num_sim
