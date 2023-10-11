// Created by Baozhe Zhang on Oct 11, 2022

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


// Reference: rpg_mpc (the original LICENSE is listed below)

/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
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

#include "acado_mpc/mpc_common.h"
#include "acado_mpc/mpc_wrapper.h"
#include "acado_mpc/mpc_params.h"

#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


namespace acado_mpc {

enum STATE {
  kPosX = 0,
  kPosY,
  kPosZ,
  kVelX,
  kVelY,
  kVelZ,
  kOriW,
  kOriX,
  kOriY,
  kOriZ,
  kAccImuX,
  kAccImuY,
  kAccImuZ,
  kOmegaNonX,
  kOmegaNonY,
  kOmegaNonZ,
  kBetaNonX,
  kBetaNonY,
  kBetaNonZ
};

enum INPUT {
  kThrust = 0,
  kRateX = 1,
  kRateY = 2,
  kRateZ = 3
};

/**
 * @brief MpcController class
 * 
 * @tparam T scalar type
 */
template<typename T>
class MpcController {
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static_assert(kStateSize == 19,
                "MpcController: Wrong model size. Number of states does not match.");
  static_assert(kInputSize == 4,
                "MpcController: Wrong model size. Number of inputs does not match.");

  MpcController(const ros::NodeHandle& nh,
                const ros::NodeHandle& pnh,
                const std::string& topic = "acado_mpc/trajectory_predicted", 
                const std::string& topic_world = "acado_mpc/trajectory_predicted_world");

  MpcController() : MpcController(ros::NodeHandle(), ros::NodeHandle("~")) {}

  acado_mpc_common::ControlCommand off();

  acado_mpc_common::ControlCommand run(
      const acado_mpc_common::QuadRelativeEstimate& state_estimate,
      const acado_mpc_common::RelativeTrajectory& reference_trajectory,
      const MpcParams<T>& params, 
      const Eigen::Quaterniond& W_q_non = Eigen::Quaterniond::Identity());


private:

  bool setStateEstimate(
      const acado_mpc_common::QuadRelativeEstimate& state_estimate);

  bool setReference(const acado_mpc_common::RelativeTrajectory& reference_trajectory);

  acado_mpc_common::ControlCommand updateControlCommand(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
      ros::Time& time);

  bool publishPrediction(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
      ros::Time& time, 
      const Eigen::Quaterniond& W_q_non);


  void preparationThread();

  bool setNewParams(MpcParams<T>& params);

  // Handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // // Subscribers and publisher.
  ros::Publisher pub_predicted_trajectory_;
  ros::Publisher pub_predicted_trajectory_world_;

  // Parameters
  MpcParams<T> params_;

  // MPC
  MpcWrapper<T> mpc_wrapper_;

  // Preparation Thread
  std::thread preparation_thread_;

  // Variables
  T timing_feedback_, timing_preparation_;
  bool solve_from_scratch_;
  Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> reference_states_;
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_;
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
};


} // namespace acado_mpc
