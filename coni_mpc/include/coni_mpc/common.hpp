/*
 * Author: Baozhe Zhang
 * Created Date: Sep 13, 2022
 * Usage: This file contains some utility structs and functions.
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

#pragma once

#include <cmath>
#include <random>  // add noise
#include <vector>
#include <iostream>
#include <algorithm>

#include <acado_mpc/mpc_common.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

/////////////////////////////////////////////////////////////
//              Random number generator                    //
/////////////////////////////////////////////////////////////

double get_random_number()
{
  static std::mt19937 generator(std::random_device{}());
  static std::normal_distribution<double> dist(0.0, 1.0);
  return dist(generator);
}

/////////////////////////////////////////////////////////////
//              Common trajectories                        //
/////////////////////////////////////////////////////////////

acado_mpc_common::RelativeTrajectory
GenerateOnePointTrajectory(size_t window_num, double start_z){
  acado_mpc_common::RelativeTrajectory result;
  result.points.clear();
  for (auto i = 0; i < window_num; i++) {
    acado_mpc_common::RelativeTrajectoryPoint point;
    point.position = Eigen::Vector3d(0, 0, start_z);
    point.velocity = Eigen::Vector3d::Zero();
    point.orientation = Eigen::Quaterniond::Identity();
    point.a_imu_ref = Eigen::Vector3d::Zero();
    point.a_imu_ref(2) = 9.81;
    point.omega_non_ref = Eigen::Vector3d::Zero();
    point.beta_non_ref = Eigen::Vector3d::Zero();
    result.points.push_back(point);
  }
  return result;
}


acado_mpc_common::RelativeTrajectory
GenerateOnePointTrajectory(size_t window_num, double start_x, double start_y, double start_z){
  acado_mpc_common::RelativeTrajectory result;
  result.points.clear();
  for (auto i = 0; i < window_num; i++) {
    acado_mpc_common::RelativeTrajectoryPoint point;
    point.position = Eigen::Vector3d(start_x, start_y, start_z);
    point.velocity = Eigen::Vector3d::Zero();
    point.orientation = Eigen::Quaterniond::Identity();
    point.a_imu_ref = Eigen::Vector3d::Zero();
    point.a_imu_ref(2) = 9.81;
    point.omega_non_ref = Eigen::Vector3d::Zero();
    point.beta_non_ref = Eigen::Vector3d::Zero();
    result.points.push_back(point);
  }
  return result;
}

std::vector<acado_mpc_common::RelativeTrajectoryPoint>
GenerateCircleTrajectory(size_t sample_num, double R, double omega, double duration,
                         double start_z)
{
  std::vector<acado_mpc_common::RelativeTrajectoryPoint> result;
  Eigen::Matrix3d temp_rotation_WB;
  Eigen::Quaterniond temp_q_WB;
  const double G = 9.81;
  double t = sqrt(pow(G, 2) + pow(omega * omega * R, 2));
  for (size_t i = 0; i < sample_num; i++) {
    temp_rotation_WB << G / t * cos(omega * (i * duration / sample_num)),
        -sin(omega * (i * duration / sample_num)),
        -omega * omega * R / t * cos(omega * (i * duration / sample_num)),
        G / t * sin(omega * (i * duration / sample_num)),
        cos(omega * (i * duration / sample_num)),
        -omega * omega * R /t * sin(omega * (i * duration / sample_num)),
        omega * omega * R / t,
        0,
        G / t;
    temp_q_WB = Eigen::Quaterniond(temp_rotation_WB);
    temp_q_WB.normalize();
    acado_mpc_common::RelativeTrajectoryPoint point;
    point.position(0) = R * cos(omega * (i * duration / sample_num));
    point.position(1) = R * sin(omega * (i * duration / sample_num));
    point.position(2) = start_z;
    point.orientation = temp_q_WB;
    point.velocity(0) = -omega * R * sin(omega * (i * duration / sample_num));
    point.velocity(1) = omega * R * cos(omega * (i * duration / sample_num));
    point.velocity(2) = 0;
    point.a_imu_ref = Eigen::Vector3d::Zero();
    point.a_imu_ref(2) = 9.81;
    point.omega_non_ref = Eigen::Vector3d::Zero();
    point.beta_non_ref = Eigen::Vector3d::Zero();
    result.push_back(point);
  }
  return result;
}


