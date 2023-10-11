// Created by Baozhe on Oct 11, 2022

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


#include "acado_mpc/mpc_controller.h"

#include <ctime>

namespace acado_mpc {

template<typename T>
MpcController<T>::MpcController(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh, 
    const std::string& topic, const std::string& topic_world) :
    nh_(nh),
    pnh_(pnh),
    mpc_wrapper_(MpcWrapper<T>()),
    timing_feedback_(T(1e-3)),
    timing_preparation_(T(1e-3)),
    est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
                                                  0, 0, 0, 
                                                  0, 0, 0, 
                                                  1, 0, 0, 0, 
                                                  0, 0, 9.81, 
                                                  0, 0, 0, 
                                                  0, 0, 0).finished()),
    reference_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
    predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()) {
  pub_predicted_trajectory_ =
      nh_.advertise<nav_msgs::Path>(topic, 1);
  pub_predicted_trajectory_world_ = 
      nh_.advertise<nav_msgs::Path>(topic_world, 1);


  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  setNewParams(params_);

  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
}


template<typename T>
acado_mpc_common::ControlCommand MpcController<T>::off() {
  return acado_mpc_common::ControlCommand();
}

template<typename T>
acado_mpc_common::ControlCommand MpcController<T>::run(
    const acado_mpc_common::QuadRelativeEstimate& state_estimate,
    const acado_mpc_common::RelativeTrajectory& reference_trajectory,
    const MpcParams<T>& params, 
    const Eigen::Quaterniond& W_q_non) {

#define GREEN std::string("\033[1;42m")
#define RST std::string("\033[0m")

  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();
  if (params.changed_) {
    params_ = params;
    setNewParams(params_);
  }

  preparation_thread_.join();

  // Convert everything into Eigen format.
  setStateEstimate(state_estimate);
  setReference(reference_trajectory);

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    ROS_INFO("Solving MPC with hover as initial guess.");
    mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);
  std::cout << GREEN + "ACADO Opt objective value: " << acado_getObjective() << RST << std::endl;


  // solver breaks
  if (std::isnan(acado_getObjective())) {
    ROS_ERROR("MPC fails");
    ROS_ERROR("MPC fails");
    ROS_ERROR("MPC fails");
    acado_mpc_common::ControlCommand hover;
    hover.bodyrates = Eigen::Vector3d::Zero();
    hover.collective_thrust = 9.8;
    preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);
    return hover;
#undef GREEN
#undef RST
  }

  // Publish the predicted trajectory.
  publishPrediction(predicted_states_, predicted_inputs_, call_time, W_q_non);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  // Return the input control command.
  return updateControlCommand(predicted_states_.col(0),
                              predicted_inputs_.col(0),
                              call_time);
}

template<typename T>
bool MpcController<T>::setStateEstimate(
    const acado_mpc_common::QuadRelativeEstimate& state_estimate) {
  est_state_(kPosX) = state_estimate.position.x();
  est_state_(kPosY) = state_estimate.position.y();
  est_state_(kPosZ) = state_estimate.position.z();
  est_state_(kVelX) = state_estimate.velocity.x();
  est_state_(kVelY) = state_estimate.velocity.y();
  est_state_(kVelZ) = state_estimate.velocity.z();
  est_state_(kOriW) = state_estimate.orientation.w();
  est_state_(kOriX) = state_estimate.orientation.x();
  est_state_(kOriY) = state_estimate.orientation.y();
  est_state_(kOriZ) = state_estimate.orientation.z();
  est_state_(kAccImuX) = state_estimate.a_imu.x();
  est_state_(kAccImuY) = state_estimate.a_imu.y();
  est_state_(kAccImuZ) = state_estimate.a_imu.z();
  est_state_(kOmegaNonX) = state_estimate.omega_non.x();
  est_state_(kOmegaNonY) = state_estimate.omega_non.y();
  est_state_(kOmegaNonZ) = state_estimate.omega_non.z();
  est_state_(kBetaNonX) = state_estimate.beta_non.x();
  est_state_(kBetaNonY) = state_estimate.beta_non.y();
  est_state_(kBetaNonZ) = state_estimate.beta_non.z();
  const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
  return quaternion_norm_ok;
}

// TODO: this function also computes the reference inputs?
template<typename T>
bool MpcController<T>::setReference(
    const acado_mpc_common::RelativeTrajectory& reference_trajectory) {
  reference_states_.setZero();
  reference_inputs_.setZero();
  reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << (T)9.8f, (T)0.0f, (T)0.0f, (T)0.0f).finished().replicate(1, kSamples + 1);

  const T dt = mpc_wrapper_.getTimestep();
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  // Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);
  if (reference_trajectory.points.size() == 1) {
    // q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        // reference_trajectory.points.front().heading,
        // Eigen::Matrix<T, 3, 1>::UnitZ()));
    // q_orientation = reference_trajectory.points.front().orientation.template cast<T>() * q_heading;
    q_orientation = reference_trajectory.points.front().orientation.template cast<T>();
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        << reference_trajectory.points.front().position.template cast<T>(),
        reference_trajectory.points.front().velocity.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        reference_trajectory.points.front().a_imu_ref.template cast<T>(),
        reference_trajectory.points.front().omega_non_ref.template cast <T>(),
        reference_trajectory.points.front().beta_non_ref.template cast <T>()
    ).finished().replicate(1, kSamples + 1);

    // acceleration << reference_trajectory.points.front().acceleration.template cast<T>() - gravity;
    // reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
    //     reference_trajectory.points.front().bodyrates.template cast<T>()
    // ).finished().replicate(1, kSamples + 1);
  } else {
    for (int i = 0; i < kSamples + 1; i++) {
      // q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          // iterator->heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
      // q_orientation = q_heading * iterator->orientation.template cast<T>();
      q_orientation = reference_trajectory.points.at(i).orientation.template cast<T>();
      reference_states_.col(i) << reference_trajectory.points.at(i).position.template cast<T>(),
          reference_trajectory.points.at(i).velocity.template cast<T>(),
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          reference_trajectory.points.at(i).a_imu_ref.template cast<T>(),
          reference_trajectory.points.at(i).omega_non_ref.template cast<T>(),
          reference_trajectory.points.at(i).beta_non_ref.template cast<T>();
      // if (reference_states_.col(i).segment(kOriW, 4).dot(
      //     est_state_.segment(kOriW, 4)) < 0.0)
      //   reference_states_.block(kOriW, i, 4, 1) =
      //       -reference_states_.block(kOriW, i, 4, 1);
      // acceleration << iterator->acceleration.template cast<T>() - gravity;
      // reference_inputs_.col(i) << acceleration.norm(),
      //     iterator->bodyrates.template cast<T>();
      quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
    }
  }
  return quaternion_norm_ok;
}

template<typename T>
acado_mpc_common::ControlCommand MpcController<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

  // Bound inputs for sanity.
  input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
                                           std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
  input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
  input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
  input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
                                          std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));

  acado_mpc_common::ControlCommand command;

  command.collective_thrust = input_bounded(INPUT::kThrust);
  command.bodyrates.x() = input_bounded(INPUT::kRateX);
  command.bodyrates.y() = input_bounded(INPUT::kRateY);
  command.bodyrates.z() = input_bounded(INPUT::kRateZ);
  return command;
}

template<typename T>
bool MpcController<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
    ros::Time& time, 
    const Eigen::Quaterniond& W_q_non) {
  nav_msgs::Path path_msg;
  nav_msgs::Path path_world_msg;
  path_msg.header.stamp = time;
  path_world_msg.header.stamp = time;
  path_msg.header.frame_id = acado_mpc_common::RELATIVE_FRAME_ID;
  path_world_msg.header.frame_id = acado_mpc_common::WORLD_FRAME_ID;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped pose_world;
  T dt = mpc_wrapper_.getTimestep();

  for (int i = 0; i < kSamples; i++) {
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = states(kPosX, i);
    pose.pose.position.y = states(kPosY, i);
    pose.pose.position.z = states(kPosZ, i);
    pose.pose.orientation.w = states(kOriW, i);
    pose.pose.orientation.x = states(kOriX, i);
    pose.pose.orientation.y = states(kOriY, i);
    pose.pose.orientation.z = states(kOriZ, i);
    path_msg.poses.push_back(pose);
    Eigen::Vector3d temp_position(states(kPosX, i), 
        states(kPosY, i), states(kPosZ, i));
    Eigen::Quaterniond temp_orientation(states(kOriW, i), 
        states(kOriX, i), states(kOriY, i), states(kOriZ, i));
    temp_position = W_q_non * temp_position;
    temp_orientation = W_q_non * temp_orientation;
    pose_world.header.stamp = time + ros::Duration(i * dt);
    pose_world.header.seq = i;
    pose_world.pose.position.x = temp_position.x();
    pose_world.pose.position.y = temp_position.y();
    pose_world.pose.position.z = temp_position.z();
    pose_world.pose.orientation.w = temp_orientation.w();
    pose_world.pose.orientation.x = temp_orientation.x();
    pose_world.pose.orientation.y = temp_orientation.y();
    pose_world.pose.orientation.z = temp_orientation.z();
    path_world_msg.poses.push_back(pose_world);
  }

  pub_predicted_trajectory_.publish(path_msg);
  pub_predicted_trajectory_world_.publish(path_world_msg);

  return true;
}

template<typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template<typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  mpc_wrapper_.setLimits(
      params.min_thrust_, params.max_thrust_,
      params.max_bodyrate_xy_, params.max_bodyrate_z_, 
      params.max_v_z_);
  // mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template
class MpcController<float>;

template
class MpcController<double>;

} // namespace acado_mpc
