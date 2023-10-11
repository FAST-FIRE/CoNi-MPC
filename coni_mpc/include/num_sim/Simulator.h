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

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include <num_sim/System.h>
#include <num_sim/Integrator.h>

namespace num_sim 
{

// one system, one integrator, and one simulator
template <class SYSTEM>
class NumericalSimulator
{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = SYSTEM::STATE_DIM;
  static const size_t CONTROL_DIM = SYSTEM::CONTROL_DIM;
  
  using State_t = typename SYSTEM::State_t;
  using Control_t = typename SYSTEM::Control_t;
  using StateArray_t = typename SYSTEM::StateArray_t;
  using TimeArray_t = typename SYSTEM::TimeArray_t;

  NumericalSimulator() = default;

  // constructor
  NumericalSimulator(double system_dt, 
      double control_dt, 
      const State_t &x0, 
      std::shared_ptr<SYSTEM> system_ptr, 
      bool verbose = false) : 
      system_dt_(system_dt), 
      control_dt_(control_dt), 
      system_ptr_(system_ptr), 
      x0_(x0), 
      stop_(false), 
      verbose_(verbose), 
      integrator_(system_ptr_)
  {
    if (system_dt_ <= 0 || control_dt_ <= 0)
        throw std::runtime_error("Step sizes must be positive.");
    if (system_dt_ > control_dt_)
        throw std::runtime_error("Simulation step must be smaller than the control step.");
  }

  // copy constructor
  NumericalSimulator(const NumericalSimulator &rhs) = delete; 

  // destructor
  virtual ~NumericalSimulator() { finish(); }
  // Gets called after the integrator step
  virtual void finishSystemIteration(double sim_time) {}
  // During controller update, this method does processing before the state measurement arrives
  virtual void prepareControllerIteration(double sim_time) {}
  // During controller update, this method does processing once the state measurement arrives
  virtual void finishControllerIteration(double sim_time) {}
  // spawns the two threads in a nonblocking way (using RK4 to integrate)
  void simulate(double duration)
  {
      stop_ = false;
      sim_start_time_ = std::chrono::high_resolution_clock::now();
      x_ = x0_;
      system_thread_ = std::thread(&NumericalSimulator::simulateSystem, this, duration);
      control_thread_ = std::thread(&NumericalSimulator::simulateController, this, duration);
  }

  // waits for the simulation threads to finish
  void finish()
  {
      if (system_thread_.joinable())
          system_thread_.join();
      if (control_thread_.joinable())
          control_thread_.join();
    }

  // stops the simulation
  void stop() { stop_ = true; }


 protected: 

  void simulateSystem(double duration)
  {
    const double residue = control_dt_ / system_dt_ - size_t(control_dt_ / system_dt_);
    auto wall_time = sim_start_time_;
    double sim_time = std::chrono::duration<double>(wall_time - sim_start_time_).count();

    while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - sim_start_time_).count() < duration &&
        !stop_) {
      // 
      integrator_.integrate_n_steps(x_, sim_time, size_t(control_dt_ / system_dt_), system_dt_);

      if (residue > 1e-6)
        integrator_.integrate_n_steps(x_, sim_time + size_t(control_dt_ / system_dt_) * system_dt_, 1, residue);
      
      if (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - wall_time).count() >= control_dt_) {
        if (verbose_)
            std::cerr << "Simulation running too slow. Please increase the step size!" << std::endl;
      } else {
        std::this_thread::sleep_until(wall_time + std::chrono::duration<double>(control_dt_));
      }
      
      wall_time += std::chrono::duration_cast<std::chrono::system_clock::duration>(
          std::chrono::duration<double>(control_dt_));
      
      state_mtx_.lock();
      x_ = integrator_.getStateArray().back();
      state_mtx_.unlock();

      sim_time = std::chrono::duration<double>(wall_time - sim_start_time_).count();
      finishSystemIteration(sim_time);
    }
  }
  
  void simulateController(double duration)
  {
    double sim_time;
    while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - sim_start_time_).count() < duration &&
        !stop_) {
      // 
      sim_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - sim_start_time_).count();
      prepareControllerIteration(sim_time);
      sim_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - sim_start_time_).count();
      finishControllerIteration(sim_time);
    }
  }


  double system_dt_;
  double control_dt_;
  std::shared_ptr<SYSTEM> system_ptr_;
  std::chrono::time_point<std::chrono::high_resolution_clock> sim_start_time_;
  State_t x0_;
  State_t x_;
  std::thread system_thread_;
  std::thread control_thread_;
  std::mutex state_mtx_;
  std::mutex control_mtx_;
  std::atomic<bool> stop_;
  bool verbose_;
  Integrator<STATE_DIM, CONTROL_DIM> integrator_;
};



} // namespace num_sim