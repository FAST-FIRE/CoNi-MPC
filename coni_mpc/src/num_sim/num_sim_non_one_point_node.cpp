/*
 * @Author: Baozhe ZHANG 
 * @Date: 2023-05-22 12:46:41 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-05-22 13:50:20
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

#include "num_sim/Simulator.h"
#include "acado_mpc/mpc_controller.h"
#include "acado_mpc/mpc_common.h"
#include "coni_mpc/common.hpp"
#include "coni_mpc/num_sim_mpc.h"

#include <functional>

#include <ros/ros.h>
#include <gflags/gflags.h>


using namespace num_sim;
using namespace acado_mpc_common;

DEFINE_double(r, 0.0, "R value");
DEFINE_double(v, 0.0, "V value");
DEFINE_double(w, 0.01, "W value");

const double START_Z = 2.0;
const double DURATION = 15.0;


class QuadrotorSimulator : public NumericalSimulator<QuadrotorSystem>
{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base_t = NumericalSimulator<QuadrotorSystem>;
  using CarState_t = Eigen::Matrix<double, 16, 1>; // p, v, q, a, w

  // constructor
  QuadrotorSimulator(double system_dt, 
      double control_dt, 
      const State_t &x0, 
      std::shared_ptr<QuadrotorSystem> system_ptr, 
      std::shared_ptr<coni_mpc::NumSimMpc> control_system_ptr, 
      const std::vector<RelativeTrajectoryPoint> &trajectory, 
      const std::function<RelativeTrajectory(size_t, double, double, size_t, const std::vector<RelativeTrajectoryPoint>&)> &plan_strategy, 
      const std::vector<CarState_t> &car_trajectory, 
      const std::function<std::vector<CarState_t>(size_t, double, double, size_t, const std::vector<CarState_t>&)> &car_plan_strategy, 
      double trajectory_duration, 
      size_t trajectory_sample_num, 
      double car_trajectory_duration, 
      size_t car_trajectory_sample_num,  
      bool verbose = true) : 
      Base_t(system_dt, control_dt, x0, system_ptr, verbose), 
      control_system_ptr_(control_system_ptr), 
      trajectory_(trajectory), 
      plan_strategy_(plan_strategy), 
      car_trajectory_(car_trajectory), 
      car_plan_strategy_(car_plan_strategy), 
      trajectory_duration_(trajectory_duration), 
      trajectory_sample_num_(trajectory_sample_num), 
      car_trajectory_duration_(car_trajectory_duration), 
      car_trajectory_sample_num_(car_trajectory_sample_num)
  {
  }   

  // FIXME: if this virtual destructor is not specified or
  // finish() is not called (i.e., the threads are joined because of the destructor)
  // then the following overriden functions are not properly dispatched
  virtual ~QuadrotorSimulator() { finish(); }


  virtual void finishSystemIteration(double sim_time) override
  {
    // printf("[System Iteration]\t t: %.3f \tx: %.3f \ty: %.3f \tz: %.3f\n \tT: %.3f \tw_x: %.3f, \tw_y: %.3f \tw_z: %.3f\n", 
    //     sim_time, x_(0), x_(1), x_(2), 
    //     system_ptr_->getControlAction()(0), 
    //     system_ptr_->getControlAction()(1), 
    //     system_ptr_->getControlAction()(2), 
    //     system_ptr_->getControlAction()(3));
  }

  virtual void prepareControllerIteration(double sim_time) override
  {
    nav_msgs::Odometry car_odom;
    auto car_point = car_plan_strategy_(
        1, 
        sim_time, 
        car_trajectory_duration_, 
        car_trajectory_sample_num_, 
        car_trajectory_).at(0);
    car_odom.header.frame_id = acado_mpc_common::WORLD_FRAME_ID;
    car_odom.header.stamp = ros::Time::now();
    car_odom.pose.pose.position.x = car_point(0);
    car_odom.pose.pose.position.y = car_point(1);
    car_odom.pose.pose.position.z = car_point(2);
    car_odom.twist.twist.linear.x = car_point(3);
    car_odom.twist.twist.linear.y = car_point(4);
    car_odom.twist.twist.linear.z = car_point(5);
    car_odom.pose.pose.orientation.w = car_point(6);
    car_odom.pose.pose.orientation.x = car_point(7);
    car_odom.pose.pose.orientation.y = car_point(8);
    car_odom.pose.pose.orientation.z = car_point(9);
    control_system_ptr_->setImu(Eigen::Vector3d(
        car_point(10), car_point(11), 9.8));
    car_odom.twist.twist.angular.x = car_point(13);
    car_odom.twist.twist.angular.y = car_point(14);
    car_odom.twist.twist.angular.z = car_point(15);
    // printf("[Controller Iteration]\t t: %.3f \tx: %.3f \ty: %.3f \tz: %.3f\n", 
    //     sim_time, car_point(0), car_point(1), car_point(2)); 

    nav_msgs::Odometry quad_odom;

    state_mtx_.lock();
    State_t x_temp = x_;
    state_mtx_.unlock();

    quad_odom.header.frame_id = acado_mpc_common::WORLD_FRAME_ID;
    quad_odom.header.stamp = ros::Time::now();
    quad_odom.pose.pose.position.x = x_temp(0);
    quad_odom.pose.pose.position.y = x_temp(1);
    quad_odom.pose.pose.position.z = x_temp(2);
    quad_odom.twist.twist.linear.x = x_temp(3);
    quad_odom.twist.twist.linear.y = x_temp(4);
    quad_odom.twist.twist.linear.z = x_temp(5);
    quad_odom.pose.pose.orientation.w = x_temp(6);
    quad_odom.pose.pose.orientation.x = x_temp(7);
    quad_odom.pose.pose.orientation.y = x_temp(8);
    quad_odom.pose.pose.orientation.z = x_temp(9);

    control_system_ptr_->setCarOdom(car_odom);
    control_system_ptr_->setQuadOdom(quad_odom);

  }

  virtual void finishControllerIteration(double sim_time) override
  {
    auto x_ref = plan_strategy_(
        control_system_ptr_->WINDOW_NUM, 
        sim_time, 
        trajectory_duration_, 
        trajectory_sample_num_, 
        trajectory_);
    
    control_system_ptr_->setReferenceWindow(x_ref);

    auto command = control_system_ptr_->run();
    Control_t control;
    control << command.collective_thrust, 
               command.bodyrates.x(), 
               command.bodyrates.y(), 
               command.bodyrates.z();

    system_ptr_->setControlAction(control);
  }
 private: 
  std::shared_ptr<coni_mpc::NumSimMpc> control_system_ptr_;
  // the trajectory in the non-inertial frame
  std::vector<RelativeTrajectoryPoint> trajectory_;
  // the plan strategy of the relative trajectory
  std::function<RelativeTrajectory(size_t, double, double, size_t, const std::vector<RelativeTrajectoryPoint>&)> plan_strategy_;
  // car trajectory
  std::vector<CarState_t> car_trajectory_;
  // the plan strategy for car 
  std::function<std::vector<CarState_t>(size_t, double, double, size_t, const std::vector<CarState_t>&)> car_plan_strategy_;

  double trajectory_duration_;
  size_t trajectory_sample_num_;

  double car_trajectory_duration_;
  size_t car_trajectory_sample_num_;
};

std::vector<QuadrotorSimulator::CarState_t>
GenerateCarTrajectory(size_t sample_num, double duration, double v, double w)
{
  std::vector<QuadrotorSimulator::CarState_t> result;
  Eigen::Matrix3d temp_rotation_WB;
  Eigen::Quaterniond temp_q_WB;
  for (size_t i = 0; i < sample_num; i++) {
    temp_rotation_WB << 
        cos(w * (i * duration / sample_num)), -sin(w * (i * duration / sample_num)), 0.0,  
        sin(w * (i * duration / sample_num)), cos(w * (i * duration / sample_num)), 0.0, 
        0.0, 0.0, 1.0;
    temp_q_WB = Eigen::Quaterniond(temp_rotation_WB);
    temp_q_WB.normalize();
    QuadrotorSimulator::CarState_t point;
    point(0) = v / w * sin(w * (i * duration / sample_num));
    point(1) = -v / w * cos(w * (i * duration / sample_num)) + v / w;
    point(2) = 0.0;
    point(3) = v * cos(w * (i * duration / sample_num));
    point(4) = v * sin(w * (i * duration / sample_num));
    point(5) = 0.0;
    point(6) = temp_q_WB.w();
    point(7) = temp_q_WB.x();
    point(8) = temp_q_WB.y();
    point(9) = temp_q_WB.z();
    point(10) = -v * w * sin(w * (i * duration / sample_num));
    point(11) = v * w * cos(w * (i * duration / sample_num));
    point(12) = 0.0;
    point(13) = 0.0;
    point(14) = 0.0;
    point(15) = w;
    result.push_back(point);
  }
  return result;

}


int main(int argc, char **argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  double r = FLAGS_r;
  double v = FLAGS_v;
  double w = FLAGS_w;

  ros::init(argc, argv, "num_sim_non_one_point_node");
  ros::NodeHandle nh; 
  ros::NodeHandle pnh("~");

  auto control_system_ptr = std::make_shared<coni_mpc::NumSimMpc>(nh, pnh);
  auto quad_system_ptr = std::make_shared<QuadrotorSystem>("Quadrotor");
  // behind the car
  auto trajectory = GenerateOnePointTrajectory(control_system_ptr->WINDOW_NUM, -r, 0.0, START_Z);
  control_system_ptr->setReferenceTrajectory(trajectory);
  control_system_ptr->setImu(Eigen::Vector3d(0.0, 0.0, 9.8));
  auto car_trajectory = GenerateCarTrajectory(10000, DURATION, v, w);
  
  auto plan = [](size_t window_num, double time, double duration, size_t sample_num, 
      const std::vector<RelativeTrajectoryPoint> &trajectory) -> RelativeTrajectory
      {
        RelativeTrajectory result;
        for (size_t i = 0; i < window_num; i++)
          result.points.push_back(trajectory.at(0));
        return result;
      };

  auto car_plan = [](size_t window_num, double time, double duration, size_t sample_num, 
      const std::vector<QuadrotorSimulator::CarState_t> &trajectory) -> std::vector<QuadrotorSimulator::CarState_t>
      {
        size_t target_index = (size_t)(std::fmod(time, duration) / duration * (sample_num - 1));
        std::vector<QuadrotorSimulator::CarState_t> result;
        if (target_index > sample_num - window_num) {
          size_t i = 0;
          for (; i < sample_num - 1 - target_index; i++) 
            result.push_back(trajectory.at(i + target_index));
          for (; i < window_num; i++)
            result.push_back(trajectory.at(sample_num - 1));
        } else {
          for (size_t i = 0; i < window_num; i++)
            result.push_back(trajectory.at(i + target_index));
        }
        return result;
      };



  QuadrotorSimulator::State_t x0 = QuadrotorSimulator::State_t::Zero();
  x0(0) = -r;
  x0(2) = START_Z;
  x0(6) = 1.0;


  QuadrotorSimulator simulator(0.01, 0.01, x0, quad_system_ptr, control_system_ptr, trajectory.points, 
      plan, car_trajectory, car_plan, 
      1.0 /* no use for fixed point */, 
      10, /* no use for fixed point */
      DURATION, 10000);
  simulator.simulate(DURATION);
  simulator.finish();

  return 0;
}
