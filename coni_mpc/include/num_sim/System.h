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

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

const double G = 9.8;

namespace num_sim
{


template <size_t state_dim, size_t control_dim>
class System
{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = state_dim;
  static const size_t CONTROL_DIM = control_dim;

  using State_t = Eigen::Matrix<double, STATE_DIM, 1>;
  using Control_t = Eigen::Matrix<double, CONTROL_DIM, 1>;
  using StateArray_t = typename std::vector<State_t>;
  using TimeArray_t = typename std::vector<double>;

  // default constructor
  System(const std::string &name) :
      name_(name), 
      control_action_(Control_t::Zero())
  {
  }

  // copy constructor
  System(const System &rhs) : 
      name_(rhs.name_), 
      control_action_(rhs.control_action_)
  {
  }

  // copy assignment operator
  System& operator=(const System &rhs)
  {
    if (this != &rhs) {
      name_ = rhs.name_;
      control_action_ = rhs.control_action_;
    }
    return *this;
  }

  virtual ~System() {}

  virtual void updateControlledDynamics(const State_t &state, 
      const double &t, 
      const Control_t &control, 
      State_t &derivative) = 0;
  
  virtual void updateDynamics(const State_t &state, 
      const double &t, 
      State_t &derivative) = 0;

  std::string getName() const { return name_; }
  Control_t getControlAction() const { return control_action_; }
  void setControlAction(const Control_t &control) { control_action_ = control; }

 protected:
  std::string name_;
  Control_t control_action_;
};

// This is a simple model of the quadrotor system
class QuadrotorSystem final : public System<10, 4>
{
 public: 

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = System<10, 4>;

  // constructor
  QuadrotorSystem(const std::string &name) : 
      Base(name)
  {
  }

  // copy constructor
  QuadrotorSystem(const QuadrotorSystem &rhs) : 
      Base(rhs)
  {
  }

  // p, v, q
  void updateControlledDynamics(const State_t &state, 
      const double &t, 
      const Control_t &control, 
      State_t &derivative) override
  {
    control_action_ = control;
    Eigen::Quaterniond q_WB(state(6), state(7), state(8), state(9));
    Eigen::Vector3d temp = q_WB * Eigen::Vector3d(0, 0, control(0));
    Eigen::Matrix4d Lambda;
    Lambda << 0, -control(1), -control(2), -control(3),
        control(1), 0, control(3), -control(2),
        control(2), -control(3), 0, control(1),
        control(3), control(2), -control(1), 0;
    derivative(0) = state(3); // vx
    derivative(1) = state(4); // vy
    derivative(2) = state(5); // vz
    derivative(3) = temp(0); // ax
    derivative(4) = temp(1); // ay
    derivative(5) = -G + temp(2); // az
    Eigen::Vector4d temp_vector = 0.5 * Lambda * Eigen::Vector4d(
        q_WB.w(), q_WB.x(), q_WB.y(), q_WB.z());
    derivative(6) = temp_vector(0);
    derivative(7) = temp_vector(1);
    derivative(8) = temp_vector(2);
    derivative(9) = temp_vector(3);
  }

  void updateDynamics(const State_t &state, 
      const double &t, 
      State_t &derivative) override
  {
    updateControlledDynamics(state, t, control_action_, derivative);
  }
};


} // namespace num_sim