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

#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input. 

int main( ){
  // Use Acado
  USING_NAMESPACE_ACADO

  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */
  const bool CODE_GEN = true;

  // System variables

  /*
   * Note:
   *
   * 1. The first 10 variables are the so-called "relative" state variables, i.e., 
   * they are the relative position, velocity, and orientation of the quadrotor
   * w.r.t. the non-inertial frame (e.g., a frame of a moving car).
   * For convenience, the names of them are plain without additional sub(or supper)-scripts.
   */
  DifferentialState     p_x, p_y, p_z;                              // relative position

  DifferentialState     v_x, v_y, v_z;                              // relative velocity

  DifferentialState     q_w, q_x, q_y, q_z;                         // relative orientation

  DifferentialState     a_imu_x, a_imu_y, a_imu_z;                  // acceleration of the non-inertial frame w.r.t. the inertial frame 
                                                                    // expressed in the non-inertial frame

  // DifferentialState     q_non_w, q_non_x, q_non_y, q_non_z;         // inverse of the orientation of the non-inertial frame in the inertial frame
                                                                    // i.e., transform from non-inertial frame to the inertial frame

  DifferentialState     omega_non_x, omega_non_y, omega_non_z;      // angular velocity of the non-inertial frame (body rates)
                                                                    // expressed in the non-inertial frame

  DifferentialState     beta_non_x, beta_non_y, beta_non_z;             // angular acceleration of the non-inertial frame
                                                                    // expressed in the non-inertial frame

  Control               T, w_x, w_y, w_z;
  DifferentialEquation  f;
  Function              h, hN;
  OnlineData            p_F_x, p_F_y, p_F_z;
  OnlineData            t_B_C_x, t_B_C_y, t_B_C_z;
  OnlineData            q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;

  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 2.0;       // Time horizon [s]
  const double dt = 0.1;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
  const double w_max_yaw = 1;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 1;      // Maximal pitch and roll rate [rad/s]
  const double v_z_max = 0.2;
  const double T_min = 2;         // Minimal thrust [N]
  const double T_max = 20;        // Maximal thrust [N]



  // System Dynamics
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(v_x) ==  beta_non_z * p_y - beta_non_y * p_z + 2 * omega_non_z * v_y - 2 * omega_non_y * v_z 
                    + (omega_non_z * omega_non_z + omega_non_y * omega_non_y) * p_x
                    - omega_non_x * omega_non_y * p_y - omega_non_x * omega_non_z * p_z
                    + 2 * (q_w * q_y + q_x * q_z) * T - a_imu_x;
  f << dot(v_y) ==  beta_non_x * p_z - beta_non_z * p_x + 2 * omega_non_x * v_z - 2 * omega_non_z * v_x
                    - omega_non_x * omega_non_y * p_x 
                    + (omega_non_z * omega_non_z + omega_non_x * omega_non_x) * p_y
                    - omega_non_y * omega_non_z * p_z
                    + 2 * (q_y * q_z - q_w * q_x) * T - a_imu_y;
  f << dot(v_z) ==  beta_non_y * p_x - beta_non_x * p_y + 2 * omega_non_y * v_x - 2 * omega_non_x * v_y
                    - omega_non_x * omega_non_z * p_x - omega_non_y * omega_non_z * p_z
                    + (omega_non_y * omega_non_y + omega_non_x * omega_non_x) * p_z
                    + (1 - 2 * q_x * q_x - 2 * q_y * q_y) * T - a_imu_z;
  f << dot(q_w) ==  0.5 * (omega_non_x*q_x + omega_non_y*q_y + omega_non_z*q_z - q_x*w_x - q_y*w_y - q_z*w_z);
  f << dot(q_x) ==  0.5 * (omega_non_z*q_y - omega_non_x*q_w - omega_non_y*q_z + q_w*w_x + q_y*w_z - q_z*w_y);
  f << dot(q_y) ==  0.5 * (omega_non_x*q_z - omega_non_z*q_x - omega_non_y*q_w + q_w*w_y - q_x*w_z + q_z*w_x);
  f << dot(q_z) ==  0.5 * (omega_non_y*q_x - omega_non_z*q_w - omega_non_x*q_y + q_w*w_z + q_x*w_y - q_y*w_x);
  f << dot(a_imu_x) == 0.0;
  f << dot(a_imu_y) == 0.0;
  f << dot(a_imu_z) == 0.0;
  f << dot(omega_non_x) == beta_non_x;
  f << dot(omega_non_y) == beta_non_y;
  f << dot(omega_non_z) == beta_non_z;
  f << dot(beta_non_x) == 0.0;
  f << dot(beta_non_y) == 0.0;
  f << dot(beta_non_z) == 0.0;

  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x << p_y << p_z
    << v_x << v_y << v_z
    << q_w << q_x << q_y << q_z
    << a_imu_x << a_imu_y << a_imu_z
    << omega_non_x << omega_non_y << omega_non_z
    << beta_non_x << beta_non_y << beta_non_z
    << T << w_x << w_y << w_z;

  // End cost vector consists of all states (no inputs at last state).
  hN << p_x << p_y << p_z
    << v_x << v_y << v_z
    << q_w << q_x << q_y << q_z
    << a_imu_x << a_imu_y << a_imu_z
    << omega_non_x << omega_non_y << omega_non_z
    << beta_non_x << beta_non_y << beta_non_z;

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 200;   // p_x
  Q(1,1) = 200;   // p_y
  Q(2,2) = 200;   // p_z
  Q(3,3) = 1;   // v_x 
  Q(4,4) = 1;   // v_y
  Q(5,5) = 1;   // v_z
  Q(6,6) = 10;   // q_w
  Q(7,7) = 10;   //  q_x
  Q(8,8) = 10;   // q_y
  Q(9,9) = 10;   // q_z
  Q(10,10) = 1e-9;   // a_imu_x
  Q(11,11) = 1e-9;   // a_imu_y
  Q(12,12) = 1e-9;   // a_imu_z
  Q(13,13) = 1e-9;   // omega_non_x
  Q(14,14) = 1e-9;   // omega_non_y
  Q(15,15) = 1e-9;   // omega_non_z
  Q(16,16) = 1e-9;   // beta_non_x
  Q(17,17) = 1e-9;   // beta_non_y
  Q(18,18) = 1e-9;   // beta_non_z
  Q(19,19) = 1;   // T
  Q(20,20) = 1;   // wx
  Q(21,21) = 1;   // wy
  Q(22,22) = 1;   // wz

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = 200;   // p_x
  QN(1,1) = 200;   // p_y
  QN(2,2) = 200;   // p_z
  QN(3,3) = 1;   // v_x 
  QN(4,4) = 1;   // v_y
  QN(5,5) = 1;   // v_z
  QN(6,6) = 10;   // q_w
  QN(7,7) = 10;   //  q_x
  QN(8,8) = 10;   // q_y
  QN(9,9) = 10;   // q_z
  QN(10,10) = 1e-9;   // a_imu_x
  QN(11,11) = 1e-9;   // a_imu_y
  QN(12,12) = 1e-9;   // a_imu_z
  QN(13,13) = 1e-9;   // omega_non_x
  QN(14,14) = 1e-9;   // omega_non_y
  QN(15,15) = 1e-9;   // omega_non_z
  QN(16,16) = 1e-9;   // beta_non_x
  QN(17,17) = 1e-9;   // beta_non_y
  QN(18,18) = 1e-9;   // beta_non_z

  // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at z = 2.0m in hover (qw = 1).
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(2) = 5.0;
  r(6) = 1.0;
  r(12) = g_z;
  // r(19) = g_z;

  DVector rN(hN.getDim());   // End cost reference
  rN.setZero();
  rN(0) = r(0);
  rN(6) = r(6);
  rN(12) = r(12);


  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );
  if(!CODE_GEN)
  {
    // For analysis, set references.
    ocp.minimizeLSQ( Q, h, r );
    ocp.minimizeLSQEndTerm( QN, hN, rN );
  }else{
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ( Q_sparse, h);
    ocp.minimizeLSQEndTerm( QN_sparse, hN );
  }

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo(T_min <= T <= T_max);
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo(-v_z_max <= v_z <= v_z_max);


  ocp.setNOD(10);


  if(!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo( AT_START, p_x ==  0.0 );
    ocp.subjectTo( AT_START, p_y ==  0.0 );
    ocp.subjectTo( AT_START, p_z ==  0.0 );
    ocp.subjectTo( AT_START, v_x ==  0.0 );
    ocp.subjectTo( AT_START, v_y ==  0.0 );
    ocp.subjectTo( AT_START, v_z ==  0.0 );
    ocp.subjectTo( AT_START, q_w ==  1.0 );
    ocp.subjectTo( AT_START, q_x ==  0.0 );
    ocp.subjectTo( AT_START, q_y ==  0.0 );
    ocp.subjectTo( AT_START, q_z ==  0.0 );
    ocp.subjectTo( AT_START, a_imu_x ==  0.0 );
    ocp.subjectTo( AT_START, a_imu_y ==  0.0 );
    ocp.subjectTo( AT_START, a_imu_z ==  g_z );
    ocp.subjectTo( AT_START, omega_non_x ==  0.0 );
    ocp.subjectTo( AT_START, omega_non_y ==  0.0 );
    ocp.subjectTo( AT_START, omega_non_z ==  0.0 );
    ocp.subjectTo( AT_START, beta_non_x ==  0.0 );
    ocp.subjectTo( AT_START, beta_non_y ==  0.0 );
    ocp.subjectTo( AT_START, beta_non_z ==  0.0 );

    // Setup some visualization
    GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
    window1.addSubplot( p_x,"position x" );
    window1.addSubplot( p_y,"position y" );
    window1.addSubplot( p_z,"position z" );
    window1.addSubplot( v_x,"verlocity x" );
    window1.addSubplot( v_y,"verlocity y" );
    window1.addSubplot( v_z,"verlocity z" );


    GnuplotWindow window2( PLOT_AT_EACH_ITERATION );
    window2.addSubplot( w_x,"rotation-vel x" );
    window2.addSubplot( w_y,"rotation-vel y" );
    window2.addSubplot( w_z,"rotation-vel z" ); 
    window2.addSubplot( T,"Thrust" );

    VariablesGrid states, parameters, controls;


    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-3 );
    algorithm.set( KKT_TOLERANCE, 1e-5 );
    algorithm << window1;
    algorithm << window2;
    algorithm.solve();

    algorithm.getDifferentialStates(states);
    algorithm.getParameters(parameters);
    algorithm.getControls(controls);
    // states.print();
    // parameters.print();
    // controls.print();

  }else{
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
    mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
    mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
    mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
    mpc.set(NUM_INTEGRATOR_STEPS,   N);
    mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
    mpc.set(HOTSTART_QP,            YES);
    mpc.set(LEVENBERG_MARQUARDT,    1.0);                 // Regularization
    mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
    mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set( GENERATE_TEST_FILE,          NO);
    mpc.set( GENERATE_MAKE_FILE,          NO);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if(mpc.exportCode("quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
      exit( EXIT_FAILURE );
    mpc.printDimensionsQP( );
  }

  return EXIT_SUCCESS;
}