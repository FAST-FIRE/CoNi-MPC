/*
 * @Author: Baozhe ZHANG 
 * @Date: 2023-05-04 11:06:13 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-05-22 12:55:49
 */

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

#ifndef CONI_MPC_MPC_BASE_H
#define CONI_MPC_MPC_BASE_H

#include "acado_mpc/mpc_common.h"
#include "acado_mpc/mpc_params.h"
#include "acado_mpc/mpc_controller.h"

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace coni_mpc
{

class MpcBase
{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcBase(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  virtual ~MpcBase() = default;
  MpcBase(const MpcBase &) = delete;
  MpcBase &operator=(const MpcBase &) = delete;
  MpcBase(MpcBase &&) = delete;
  MpcBase &operator=(MpcBase &&) = delete;


  acado_mpc_common::RelativeTrajectory 
  getReferenceTrajectory() const { return reference_trajectory_; }

  acado_mpc_common::RelativeTrajectory 
  getReferenceWindow() const { return reference_window_; }

  acado_mpc_common::QuadRelativeEstimate 
  getStateEstimate() const { return state_estimate_; }


  void setReferenceTrajectory(const acado_mpc_common::RelativeTrajectory &reference_trajectory) 
  { reference_trajectory_ = reference_trajectory; }
  void setReferenceWindow(const acado_mpc_common::RelativeTrajectory &reference_window) 
  { reference_window_ = reference_window; }
  void setStateEstimate(const acado_mpc_common::QuadRelativeEstimate &state_estimate) 
  { state_estimate_ = state_estimate; }

  virtual acado_mpc_common::ControlCommand run();

  const size_t WINDOW_NUM = acado_mpc::kSamples + 1;
 protected: 
  acado_mpc_common::RelativeTrajectory reference_trajectory_;
  acado_mpc_common::RelativeTrajectory reference_window_;
  acado_mpc_common::QuadRelativeEstimate state_estimate_;
  
  acado_mpc::MpcParams<double> mpc_params_;
  acado_mpc::MpcController<double> mpc_controller_;
};

} // namespace coni_mpc


#endif // CONI_MPC_MPC_BASE_H