/*
 * @Author: Baozhe ZHANG 
 * @Date: 2023-05-22 10:00:03 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-05-22 12:40:49
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

#ifndef CONI_MPC_NUM_SIM_MPC_H
#define CONI_MPC_NUM_SIM_MPC_H

#include "coni_mpc/mpc_base.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

namespace coni_mpc
{

class NumSimMpc final : public MpcBase 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NumSimMpc(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  virtual ~NumSimMpc() override = default;
  NumSimMpc(const NumSimMpc &) = delete;
  NumSimMpc &operator=(const NumSimMpc &) = delete;
  NumSimMpc(NumSimMpc &&) = delete;
  NumSimMpc &operator=(NumSimMpc &&) = delete;

  void setImu(const Eigen::Vector3d &imu) { imu_ = imu; };
  void setCarOdom(const nav_msgs::Odometry &car_odom) { car_odom_ = car_odom; };
  void setQuadOdom(const nav_msgs::Odometry &quad_odom) { quad_odom_ = quad_odom; };

  virtual acado_mpc_common::ControlCommand run() override;

 private: 
  ros::NodeHandle nh_, pnh_;
  Eigen::Vector3d imu_;
  nav_msgs::Odometry car_odom_;
  nav_msgs::Odometry quad_odom_;
  nav_msgs::Odometry relative_est_;

  const char *CAR_ODOM_TOPIC = "coni_mpc/car_odom";
  const char *QUAD_ODOM_TOPIC = "coni_mpc/quad_odom";
  const char *RELATIVE_EST_TOPIC = "coni_mpc/relative_est";

  // publishers in world frame
  ros::Publisher car_odom_pub_;
  ros::Publisher quad_odom_pub_;
  // relative estimation
  ros::Publisher relative_est_pub_;

  void genRelativeEstimate();
  void setEstimationNoise(double p_var, 
                          double v_var, 
                          double ori_var, 
                          double imu_non_var, 
                          double omega_non_var, 
                          double beta_non_var);



}; // class NumSimMpc

} // namespace coni_mpc



#endif // CONI_MPC_NUM_SIM_MPC_H