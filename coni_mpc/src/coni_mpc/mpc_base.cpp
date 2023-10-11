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

#include "coni_mpc/mpc_base.h"

namespace coni_mpc
{

MpcBase::MpcBase(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
    reference_trajectory_(), 
    reference_window_(),
    state_estimate_(),
    mpc_params_(),
    mpc_controller_(nh, pnh)
{
}

acado_mpc_common::ControlCommand MpcBase::run()
{
  if (reference_window_.points.size() < WINDOW_NUM) {
    return acado_mpc_common::ControlCommand();
  }
  return mpc_controller_.run(state_estimate_, reference_window_, mpc_params_);
}

} // namespace coni_mpc