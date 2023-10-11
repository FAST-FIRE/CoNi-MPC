//
// Created by baozhe on 22-10-17.
//

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

#include <acado_mpc/mpc_common.h>

namespace acado_mpc_common
{
// Implementations
QuadRelativeEstimate::QuadRelativeEstimate() :
    position(Eigen::Vector3d::Zero()),
    velocity(Eigen::Vector3d::Zero()),
    orientation(Eigen::Quaterniond::Identity()),
    a_imu(Eigen::Vector3d::Zero()),
    omega_non(Eigen::Vector3d::Zero()),
    beta_non(Eigen::Vector3d::Zero())
{
}

QuadRelativeEstimate::QuadRelativeEstimate(const nav_msgs::Odometry &relative_estimate_msg,
                                           const Eigen::Vector3d &a_imu,
                                           const Eigen::Vector3d &omega_non,
                                           const Eigen::Vector3d &beta_non) :
    position(relative_estimate_msg.pose.pose.position.x,
             relative_estimate_msg.pose.pose.position.y,
             relative_estimate_msg.pose.pose.position.z),
    velocity(relative_estimate_msg.twist.twist.linear.x,
             relative_estimate_msg.twist.twist.linear.y,
             relative_estimate_msg.twist.twist.linear.z),
    orientation(relative_estimate_msg.pose.pose.orientation.w,
                relative_estimate_msg.pose.pose.orientation.x,
                relative_estimate_msg.pose.pose.orientation.y,
                relative_estimate_msg.pose.pose.orientation.z),
    a_imu(a_imu),
    omega_non(omega_non), beta_non(beta_non)
{
}

QuadRelativeEstimate::QuadRelativeEstimate(const Eigen::Vector3d &p,
                                           const Eigen::Vector3d &v,
                                           const Eigen::Quaterniond &q,
                                           const Eigen::Vector3d &a_imu,
                                           const Eigen::Vector3d &omega_non,
                                           const Eigen::Vector3d &beta_non) :
    position(p),
    velocity(v),
    orientation(q),
    a_imu(a_imu),
    omega_non(omega_non), beta_non(beta_non)
{
}



QuadRelativeEstimate::~QuadRelativeEstimate()
{
  //
}

RelativeTrajectoryPoint::RelativeTrajectoryPoint() :
    position(Eigen::Vector3d::Zero()),
    velocity(Eigen::Vector3d::Zero()),
    orientation(Eigen::Quaterniond::Identity()),
    a_imu_ref(Eigen::Vector3d::Zero()),
    omega_non_ref(Eigen::Vector3d::Zero()),
    beta_non_ref(Eigen::Vector3d::Zero())
{
  //
}

RelativeTrajectoryPoint::RelativeTrajectoryPoint(const Eigen::Vector3d &p,
                                                 const Eigen::Vector3d &v,
                                                 const Eigen::Quaterniond &q,
                                                 const acado_mpc_common::QuadRelativeEstimate &relative_estimate) :
    position(p),
    velocity(v),
    orientation(q),
    a_imu_ref(relative_estimate.a_imu),
    omega_non_ref(relative_estimate.omega_non),
    beta_non_ref(relative_estimate.beta_non)
{
}

RelativeTrajectoryPoint::RelativeTrajectoryPoint(const Eigen::Vector3d &p,
                                                 const Eigen::Vector3d &v,
                                                 const Eigen::Quaterniond &q,
                                                 const Eigen::Vector3d &a_imu,
                                                 const Eigen::Vector3d &omega_non,
                                                 const Eigen::Vector3d &beta_non) :
    position(p),
    velocity(v),
    orientation(q),
    a_imu_ref(a_imu),
    omega_non_ref(omega_non),
    beta_non_ref(beta_non)
{
}

RelativeTrajectoryPoint::~RelativeTrajectoryPoint()
{
}

RelativeTrajectory::RelativeTrajectory() :
    points()
{
  //
}

RelativeTrajectory::RelativeTrajectory(
    const acado_mpc_common::RelativeTrajectoryPoint &point) :
    points()
{
  points.push_back(point);
}

RelativeTrajectory::~RelativeTrajectory()
{
  //
}

nav_msgs::Path RelativeTrajectory::toRosPath() const
{
  nav_msgs::Path path_msg;
  ros::Time t = ros::Time::now();
  path_msg.header.stamp = t;
  path_msg.header.frame_id = "non_inertial_frame";

  geometry_msgs::PoseStamped pose;
  for (const auto &point : points) {
    pose.pose.position.x = point.position.x();
    pose.pose.position.y = point.position.y();
    pose.pose.position.z = point.position.z();
    pose.pose.orientation.w = point.orientation.w();
    pose.pose.orientation.x = point.orientation.x();
    pose.pose.orientation.y = point.orientation.y();
    pose.pose.orientation.z = point.orientation.z();
    path_msg.poses.push_back(pose);
  }
  return path_msg;
}

ControlCommand::ControlCommand() :
    collective_thrust(0.0),
    bodyrates(Eigen::Vector3d::Zero())
{
}

ControlCommand::ControlCommand(const Eigen::Vector4d &u) :
    collective_thrust(u(0)),
    bodyrates(u.tail(3))
{
}

ControlCommand::ControlCommand(double thrust, const Eigen::Vector3d &bodyrates) :
    collective_thrust(thrust),
    bodyrates(bodyrates)
{
}

ControlCommand::~ControlCommand()
{
}

mavros_msgs::AttitudeTarget ControlCommand::toRosAttitudeTarget(double thrust_param) const 
{
  mavros_msgs::AttitudeTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "non_inertial_frame";
  msg.type_mask = 
      mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  msg.thrust = std::min(1.0, collective_thrust * thrust_param / 9.8);
  msg.body_rate.x = bodyrates.x();
  msg.body_rate.y = bodyrates.y();
  msg.body_rate.z = bodyrates.z();
  return msg;
}

} // namespace acado_mpc_common