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

#include "coni_mpc/num_sim_mpc.h"

#include <random>

namespace coni_mpc
{

NumSimMpc::NumSimMpc(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
    MpcBase(nh, pnh), 
    nh_(nh), pnh_(pnh), 
    imu_(0.0, 0.0, 9.8)
{
  car_odom_pub_ = 
      nh_.advertise<nav_msgs::Odometry>(CAR_ODOM_TOPIC, 1);
  quad_odom_pub_ = 
      nh_.advertise<nav_msgs::Odometry>(QUAD_ODOM_TOPIC, 1);
  relative_est_pub_ = 
      nh_.advertise<nav_msgs::Odometry>(RELATIVE_EST_TOPIC, 1);
}

// helper function
inline static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &vec) {
  Eigen::Matrix3d result;
  result << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return result;
}

void NumSimMpc::genRelativeEstimate()
{
  Eigen::Quaterniond W_q_quad(quad_odom_.pose.pose.orientation.w, 
                              quad_odom_.pose.pose.orientation.x, 
                              quad_odom_.pose.pose.orientation.y, 
                              quad_odom_.pose.pose.orientation.z);
  Eigen::Quaterniond W_q_non(car_odom_.pose.pose.orientation.w, 
                            car_odom_.pose.pose.orientation.x, 
                            car_odom_.pose.pose.orientation.y, 
                            car_odom_.pose.pose.orientation.z);
  W_q_quad.normalize();
  W_q_non.normalize();
  Eigen::Vector3d W_position_relative(
      quad_odom_.pose.pose.position.x - 
          car_odom_.pose.pose.position.x, 
      quad_odom_.pose.pose.position.y - 
          car_odom_.pose.pose.position.y, 
      quad_odom_.pose.pose.position.z - 
          car_odom_.pose.pose.position.z);
  Eigen::Vector3d non_position_relative = 
      W_q_non.inverse() * W_position_relative;
  state_estimate_.position = non_position_relative;
  Eigen::Vector3d W_velocity_quad = 
      Eigen::Vector3d(quad_odom_.twist.twist.linear.x, 
                      quad_odom_.twist.twist.linear.y, 
                      quad_odom_.twist.twist.linear.z);
  Eigen::Vector3d W_velocity_non = 
      Eigen::Vector3d(car_odom_.twist.twist.linear.x, 
                      car_odom_.twist.twist.linear.y, 
                      car_odom_.twist.twist.linear.z);
  Eigen::Vector3d w_velocity_relative_linear = 
      W_velocity_quad - W_velocity_non;
  Eigen::Vector3d non_body_rate_non(car_odom_.twist.twist.angular.x, 
                                    car_odom_.twist.twist.angular.y, 
                                    car_odom_.twist.twist.angular.z);
  Eigen::Vector3d non_velocity_relative = 
      -skewSymmetric(non_body_rate_non) * non_position_relative + 
      W_q_non.inverse() * w_velocity_relative_linear;
  state_estimate_.velocity = non_velocity_relative;
  state_estimate_.orientation = W_q_non.inverse() * W_q_quad;
  state_estimate_.a_imu = imu_;
  state_estimate_.omega_non = non_body_rate_non;
  state_estimate_.beta_non = Eigen::Vector3d::Zero();

  relative_est_.header.frame_id = acado_mpc_common::RELATIVE_FRAME_ID;
  relative_est_.pose.pose.position.x = state_estimate_.position.x();
  relative_est_.pose.pose.position.y = state_estimate_.position.y();
  relative_est_.pose.pose.position.z = state_estimate_.position.z();
  relative_est_.twist.twist.linear.x = state_estimate_.velocity.x();
  relative_est_.twist.twist.linear.y = state_estimate_.velocity.y();
  relative_est_.twist.twist.linear.z = state_estimate_.velocity.z();
  relative_est_.pose.pose.orientation.w = state_estimate_.orientation.w();
  relative_est_.pose.pose.orientation.x = state_estimate_.orientation.x();
  relative_est_.pose.pose.orientation.y = state_estimate_.orientation.y();
  relative_est_.pose.pose.orientation.z = state_estimate_.orientation.z();
}

/**
 * @brief Return a random variable with mean=0.0, var=1.0
 * 
 * @return double 
 */
inline static double genRV()
{
  static std::mt19937 generator(std::random_device{}());
  static std::normal_distribution<double> dist(0.0, 1.0);
  return dist(generator);
}

void NumSimMpc::setEstimationNoise(double p_var, 
                                  double v_var, 
                                  double ori_var, 
                                  double imu_non_var, 
                                  double omega_non_var, 
                                  double beta_non_var)
{
  state_estimate_.position += 
      Eigen::Vector3d(p_var * genRV(), 
                      p_var * genRV(), 
                      p_var * genRV());
  state_estimate_.velocity += 
      Eigen::Vector3d(v_var * genRV(), 
                      v_var * genRV(), 
                      v_var * genRV());
  Eigen::AngleAxisd temp(state_estimate_.orientation);
  temp.angle() += ori_var * genRV();
  state_estimate_.orientation = Eigen::Quaterniond(temp);
  state_estimate_.a_imu += 
      Eigen::Vector3d(imu_non_var * genRV(), 
                      imu_non_var * genRV(), 
                      imu_non_var * genRV());
  state_estimate_.beta_non += 
      Eigen::Vector3d(beta_non_var * genRV(), 
                      beta_non_var * genRV(), 
                      beta_non_var * genRV());
}

acado_mpc_common::ControlCommand NumSimMpc::run() 
{
  genRelativeEstimate();
  // p 0.025m; v 0.025m/s; q 2.5/180*pi rad; imu 0.025; omega 2.5/180*pi; beta 0
  setEstimationNoise(0.025*0.025, 0.025*0.025, 
      0.0435*0.0435, 0.025*0.025, 0.0435*0.0435, 0.0);

  car_odom_pub_.publish(car_odom_);
  quad_odom_pub_.publish(quad_odom_);
  relative_est_pub_.publish(relative_est_);

  return mpc_controller_.run(state_estimate_, reference_window_, mpc_params_);
}


}; // namespace coni_mpc