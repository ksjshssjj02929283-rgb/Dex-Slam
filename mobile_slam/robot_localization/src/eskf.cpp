/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * Copyright (c) 2025 ESKF Implementation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <robot_localization/eskf.hpp>
#include <robot_localization/filter_common.hpp>
#include <robot_localization/filter_utilities.hpp>
#include <angles/angles.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace robot_localization
{

Eskf::Eskf()
: FilterBase()
{
  // Initialize nominal state
  nominal_position_.setZero();
  nominal_orientation_.setIdentity();  // [w=1, x=0, y=0, z=0]
  nominal_linear_velocity_.setZero();
  nominal_angular_velocity_.setZero();
  nominal_linear_acceleration_.setZero();

  // Set gravity vector (world frame, pointing down)
  gravity_ << 0.0, 0.0, -9.81;

  FB_DEBUG("ESKF constructed\n");
}

Eskf::~Eskf()
{
}

void Eskf::setState(const Eigen::VectorXd & state)
{
  FB_DEBUG("========== ESKF setState ==========\n");
  FB_DEBUG("Setting nominal state from 15-dim vector\n");
  
  // Update nominal state from the input vector
  vectorToNominalState(state);
  
  // Reset error state to zero (error state should always be zero after setState)
  state_.setZero();
  
  // Also update the FilterBase::state_ for compatibility
  // (though in ESKF, state_ represents error state, not nominal state)
  FilterBase::setState(state_);
  
  Eigen::Vector3d euler = quaternionToEuler(nominal_orientation_);
  FB_DEBUG(
    "Nominal state set to:\n" <<
    "  Position: [" << nominal_position_(0) << ", " << nominal_position_(1) << ", " << nominal_position_(2) << "]\n" <<
    "  Orientation: [R=" << euler(0)*180.0/M_PI << "°, P=" << euler(1)*180.0/M_PI << "°, Y=" << euler(2)*180.0/M_PI << "°]\n" <<
    "  Quaternion: [w=" << nominal_orientation_.w() << ", x=" << nominal_orientation_.x() << 
                    ", y=" << nominal_orientation_.y() << ", z=" << nominal_orientation_.z() << "]\n" <<
    "  Velocity: [" << nominal_linear_velocity_(0) << ", " << nominal_linear_velocity_(1) << ", " << nominal_linear_velocity_(2) << "]\n" <<
    "Error state: " << state_.transpose() << " (should be zero)\n" <<
    "===================================\n");
}

const Eigen::VectorXd & Eskf::getState()
{
  // For ESKF, getState should return the nominal state, not the error state
  // We need to convert nominal state to 15-dim vector and cache it
  static Eigen::VectorXd cached_state(STATE_SIZE);
  cached_state = nominalStateToVector();
  
  // Note: We cannot modify FilterBase::state_ here because this is a const method
  // The returned reference is to the cached static variable
  return cached_state;
}

void Eskf::predict(
  const rclcpp::Time & reference_time,
  const rclcpp::Duration & delta)
{
  const double dt = filter_utilities::toSec(delta);

  FB_DEBUG(
    "---------------------- Eskf::predict ----------------------\n" <<
      "delta is " << dt << " seconds\n");

  // Save pre-prediction state for detailed logging (currently unused)
  // Eigen::Vector3d pre_position = nominal_position_;
  // Eigen::Quaterniond pre_orientation = nominal_orientation_;
  // Eigen::Vector3d pre_velocity = nominal_linear_velocity_;
  // Eigen::VectorXd pre_error = state_;

  // Step 1: Prepare control input (if used)
  prepareControl(reference_time, delta);

  // Step 2: Propagate nominal state (deterministic, no noise)
  propagateNominalState(dt);

  // Step 3: Propagate error covariance (stochastic)
  propagateErrorCovariance(dt);
  
  // Note: Detailed prediction debug output has been removed
  // Use FB_DEBUG macro for file-based debugging instead
}

void Eskf::propagateNominalState(double dt)
{
  // Get current rotation matrix
  Eigen::Matrix3d R = nominal_orientation_.toRotationMatrix();

  // Step 1: Update position (world frame)
  // Method: Integrate velocity in body frame, then rotate to world frame
  // This is more accurate than rotating velocity at start of timestep only.
  // 
  // CRITICAL: Velocity and acceleration are in BODY frame.
  // We need to account for the full velocity trajectory during dt.
  // Using semi-implicit integration:
  //   1. Compute velocity at start in world frame: v_world = R * v_body
  //   2. Update velocity in body frame: v_body += a_body * dt
  //   3. Compute velocity at end in world frame: v_world_new = R_new * v_body_new
  //   4. Use average velocity: p += 0.5 * (v_world + v_world_new) * dt
  //
  // But since orientation changes during timestep, we use simpler approximation:
  // p_{k+1} = p_k + R(q_k) * (v_k * dt + 0.5 * a_k * dt^2)
  // This assumes rotation matrix doesn't change significantly during dt
  nominal_position_ += R * (nominal_linear_velocity_ * dt + 
                            0.5 * nominal_linear_acceleration_ * dt * dt);

  // Step 2: Update orientation (quaternion integration)
  // q_{k+1} = q_k ⊗ exp(ω_k * dt)
  Eigen::Vector3d angle_increment = nominal_angular_velocity_ * dt;
  Eigen::Quaterniond delta_q = axisAngleToQuaternion(angle_increment);
  nominal_orientation_ = nominal_orientation_ * delta_q;
  nominal_orientation_.normalize();

  // Step 3: Update velocity (in body frame)
  // v_{k+1} = v_k + a_k * dt
  // CRITICAL: Both velocity and acceleration are in BODY frame.
  // Do NOT rotate acceleration - just add directly in body frame.
  // The rotation to world frame happens when updating position.
  nominal_linear_velocity_ += nominal_linear_acceleration_ * dt;

  // Step 4: Update angular velocity (from control or unchanged)
  if (use_control_) {
    // Apply angular acceleration from control
    nominal_angular_velocity_(0) +=
      control_acceleration_(ControlMemberVroll) * dt;
    nominal_angular_velocity_(1) +=
      control_acceleration_(ControlMemberVpitch) * dt;
    nominal_angular_velocity_(2) +=
      control_acceleration_(ControlMemberVyaw) * dt;
  }
  // Otherwise, angular velocity remains constant (or decays if damping is modeled)

  // Step 5: Update acceleration (from control or unchanged)
  if (use_control_) {
    // Set acceleration from control if available
    if (control_update_vector_[ControlMemberVx]) {
      nominal_linear_acceleration_(0) = control_acceleration_(ControlMemberVx);
    }
    if (control_update_vector_[ControlMemberVy]) {
      nominal_linear_acceleration_(1) = control_acceleration_(ControlMemberVy);
    }
    if (control_update_vector_[ControlMemberVz]) {
      nominal_linear_acceleration_(2) = control_acceleration_(ControlMemberVz);
    }
  }
  // Otherwise, acceleration remains constant
}

void Eskf::propagateErrorCovariance(double dt)
{
  // Step 1: Build error state transition matrix F
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F;
  buildErrorStateTransition(F, dt);

  // Step 2: Discretize the transition matrix
  // Φ = I + F * dt (first-order approximation)
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Phi =
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity() + F * dt;

  // Step 3: Get process noise covariance
  Eigen::MatrixXd * Q = &process_noise_covariance_;
  if (use_dynamic_process_noise_covariance_) {
    // Compute dynamic process noise based on current state
    Eigen::VectorXd current_state_vec = nominalStateToVector();
    computeDynamicProcessNoiseCovariance(current_state_vec);
    Q = &dynamic_process_noise_covariance_;
  }

  // Step 4: Propagate covariance
  // P_{k+1} = Φ * P_k * Φ^T + Q
  // Note: Q is already the discrete-time process noise covariance
  // It should NOT be multiplied by dt, as Q represents the noise added per prediction step
  // If Q were continuous-time, we would use Q*dt, but our Q is already discrete
  estimate_error_covariance_ =
    Phi * estimate_error_covariance_ * Phi.transpose() + (*Q);

  // Step 5: Ensure symmetry and positive-definiteness
  estimate_error_covariance_ =
    0.5 * (estimate_error_covariance_ + estimate_error_covariance_.transpose());

  // Add small epsilon to diagonal for numerical stability
  estimate_error_covariance_ += covariance_epsilon_;

  FB_DEBUG("Error covariance propagated\n");
}

void Eskf::buildErrorStateTransition(
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> & F,
  double dt)
{
  F.setZero();

  // Get current rotation matrix
  Eigen::Matrix3d R = nominal_orientation_.toRotationMatrix();

  // Get skew-symmetric matrices
  Eigen::Matrix3d omega_skew = skewSymmetric(nominal_angular_velocity_);
  Eigen::Matrix3d acc_skew = skewSymmetric(nominal_linear_acceleration_);

  // Build the error state transition matrix F (15x15)
  // State order: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
  // Which maps to: [p(0-2), θ(3-5), v(6-8), ω(9-11), a(12-14)]

  // Block 1a: ∂(δṗ)/∂(δθ) = -R * [v̄]×
  // Position error depends on orientation error (velocity rotated in wrong direction)
  // Since p += R * v * dt, an error in θ causes R to be wrong, leading to position error
  Eigen::Matrix3d velocity_skew = skewSymmetric(nominal_linear_velocity_);
  F.block<3, 3>(POSITION_OFFSET, ORIENTATION_OFFSET) = -R * velocity_skew;

  // Block 1b: ∂(δṗ)/∂(δv) = R
  // Position error depends on velocity error (rotated to world frame)
  // NOTE: This is R, not I, because velocity is in body frame!
  F.block<3, 3>(POSITION_OFFSET, POSITION_V_OFFSET) = R;

  // Block 2: ∂(δθ̇)/∂(δθ) = -[ω̄]×
  // Orientation error dynamics
  F.block<3, 3>(ORIENTATION_OFFSET, ORIENTATION_OFFSET) = -omega_skew;

  // Block 3: ∂(δθ̇)/∂(δω) = -I
  // Orientation error depends on angular velocity error
  F.block<3, 3>(ORIENTATION_OFFSET, ORIENTATION_V_OFFSET) =
    -Eigen::Matrix3d::Identity();

  // Block 4: ∂(δv̇)/∂(δa) = I  (NOT R!)
  // CRITICAL FIX: Since velocity is in BODY frame, not world frame:
  //   v_{body,k+1} = v_{body,k} + a_{body,k} * dt
  // Therefore: δv̇_{body} = δa_{body} (no rotation matrix needed!)
  // The old code used R because it assumed velocity in world frame.
  F.block<3, 3>(POSITION_V_OFFSET, POSITION_A_OFFSET) = 
    Eigen::Matrix3d::Identity();
  
  // Block 4b: ∂(δv̇)/∂(δθ) = 0
  // Velocity error dynamics in body frame do NOT depend on orientation error
  // (no need to set explicitly, F is initialized to zero)

  // Blocks 6 & 7: Angular velocity and acceleration error dynamics
  // Modeled as random walk or white noise, so F = 0 for these blocks
  // If damping is desired, add -β*I on the diagonal:
  // F.block<3, 3>(ORIENTATION_V_OFFSET, ORIENTATION_V_OFFSET) = -beta_omega * I;
  // F.block<3, 3>(POSITION_A_OFFSET, POSITION_A_OFFSET) = -beta_acc * I;

  FB_DEBUG("Error state transition matrix built\n");
}

void Eskf::correct(const Measurement & measurement)
{
  // ========== DEBUG OUTPUT CONTROL ==========
  // Set to true to enable detailed console output for this correction step
  bool output_enabled = false;
  
  static int correction_count = 0;
  correction_count++;
  
  FB_DEBUG(
    "\n========== ESKF CORRECT START ==========\n" <<
      "Measurement topic: " << measurement.topic_name_ << "\n");

  // Save pre-correction state for comparison
  Eigen::Vector3d pre_nominal_pos = nominal_position_;
  Eigen::Quaterniond pre_nominal_quat = nominal_orientation_;
  Eigen::VectorXd pre_error = state_;
  
  // Terminal debug output header
  if (output_enabled) {
    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║         ESKF CORRECT #" << correction_count << " - " << measurement.topic_name_ << "\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
  }

  // Step 1: Extract update indices (which states to update)
  std::vector<size_t> update_indices;
  for (size_t i = 0; i < measurement.update_vector_.size(); ++i) {
    if (measurement.update_vector_[i]) {
      // Check for NaN or Inf
      if (std::isnan(measurement.measurement_(i))) {
        FB_DEBUG(
          "Value at index " << i << " was nan. Excluding from update.\n");
      } else if (std::isinf(measurement.measurement_(i))) {
        FB_DEBUG(
          "Value at index " << i << " was inf. Excluding from update.\n");
      } else {
        update_indices.push_back(i);
      }
    }
  }

  FB_DEBUG("Number of valid measurements: " << update_indices.size() << "\n");

  size_t update_size = update_indices.size();
  if (update_size == 0) {
    FB_DEBUG("No valid measurements to update\n========================================\n\n");
    return;
  }

  // Step 2: Build measurement sub-matrices
  Eigen::VectorXd z_meas(update_size);          // Actual measurement
  Eigen::VectorXd z_pred(update_size);          // Predicted measurement
  Eigen::MatrixXd H(update_size, STATE_SIZE);   // Observation matrix
  Eigen::MatrixXd R(update_size, update_size);  // Measurement noise covariance

  H.setZero();
  R.setZero();

  // Get predicted observation from nominal state
  Eigen::VectorXd nominal_vec = nominalStateToVector();

  for (size_t i = 0; i < update_size; ++i) {
    size_t idx = update_indices[i];

    // Extract measurement
    z_meas(i) = measurement.measurement_(idx);

    // Predict measurement from nominal state
    z_pred(i) = nominal_vec(idx);

    // Build observation matrix (direct observation)
    H(i, idx) = 1.0;

    // Extract measurement noise covariance
    R(i, i) = measurement.covariance_(idx, idx);

    // Handle negative covariance
    if (R(i, i) < 0) {
      FB_DEBUG(
        "WARNING: Negative covariance for index " << i <<
          " (value: " << R(i, i) << "). Using absolute value.\n");
      R(i, i) = std::fabs(R(i, i));
    }

    // Ensure minimum covariance for numerical stability
    if (R(i, i) < 1e-9) {
      FB_DEBUG(
        "WARNING: Very small covariance for index " << i <<
          ". Setting to 1e-9.\n");
      R(i, i) = 1e-9;
    }
  }

  // Step 3: Compute innovation (measurement residual)
  Eigen::VectorXd innovation = z_meas - z_pred;

  // Normalize angles in innovation
  for (size_t i = 0; i < update_size; ++i) {
    size_t idx = update_indices[i];
    if (idx == StateMemberRoll || idx == StateMemberPitch || idx == StateMemberYaw) {
      innovation(i) = angles::normalize_angle(innovation(i));
    }
  }

  FB_DEBUG(
    "--- MEASUREMENT vs PREDICTION ---\n" <<
      "Measured:  " << z_meas.transpose() << "\n" <<
      "Predicted: " << z_pred.transpose() << "\n" <<
      "Innovation (z_meas - z_pred): " << innovation.transpose() << "\n");

  // ========== DETAILED MEASUREMENT INFO ==========
  if (output_enabled) {
    const char* state_names[] = {"X", "Y", "Z", "Roll", "Pitch", "Yaw", 
                                 "Vx", "Vy", "Vz", "Vroll", "Vpitch", "Vyaw", 
                                 "Ax", "Ay", "Az"};
    
    std::cout << "\n┌─── Step 1: Measurement Details ───────────────────────────┐\n";
    std::cout << "│ Update states: ";
    for (size_t idx : update_indices) {
      if (idx < 15) std::cout << state_names[idx] << " ";
    }
    std::cout << "\n│ Count: " << update_size << " measurements\n";
    std::cout << "└───────────────────────────────────────────────────────────┘\n";
    
    std::cout << "\n┌─── Step 2: Measurement vs Prediction ─────────────────────┐\n";
    for (size_t i = 0; i < update_size; ++i) {
      size_t idx = update_indices[i];
      std::cout << "│ " << std::setw(8) << std::left << state_names[idx] << ": ";
      std::cout << "meas=" << std::setw(10) << z_meas(i) 
                << " pred=" << std::setw(10) << z_pred(i) 
                << " innov=" << std::setw(10) << innovation(i);
      
      // Show angles in degrees
      if (idx >= StateMemberRoll && idx <= StateMemberYaw) {
        std::cout << " (" << std::setw(8) << (innovation(i)*180.0/M_PI) << "°)";
      }
      std::cout << "\n";
    }
    std::cout << "└───────────────────────────────────────────────────────────┘\n";
  }

  // Step 4: Compute innovation covariance
  Eigen::MatrixXd S = H * estimate_error_covariance_ * H.transpose() + R;

  // Step 5: Compute Kalman gain
  Eigen::MatrixXd K = estimate_error_covariance_ * H.transpose() * S.inverse();
  
  // ========== KALMAN GAIN INFO ==========
  if (output_enabled) {
    std::cout << "\n┌─── Step 3: Kalman Gain ───────────────────────────────────┐\n";
    std::cout << "│ Innovation covariance S diagonal:\n│   ";
    for (size_t i = 0; i < update_size; ++i) {
      std::cout << std::scientific << std::setprecision(2) << S(i,i) << " ";
    }
    std::cout << "\n│\n│ Kalman gain K (key states):\n";
    const char* names[] = {"X", "Y", "Z", "Roll", "Pitch", "Yaw", 
                           "Vx", "Vy", "Vz", "Vroll", "Vpitch", "Vyaw"};
    for (size_t i = 0; i < std::min((size_t)12, (size_t)STATE_SIZE); ++i) {
      std::cout << "│   " << std::setw(8) << std::left << names[i] << ": ";
      for (size_t j = 0; j < update_size; ++j) {
        std::cout << std::setw(9) << std::fixed << std::setprecision(4) << K(i, j) << " ";
      }
      std::cout << "\n";
    }
    std::cout << "└───────────────────────────────────────────────────────────┘\n";
  }

  // Step 6: Mahalanobis distance check
  if (!checkMahalanobisThreshold(
      innovation, S.inverse(), measurement.mahalanobis_thresh_))
  {
    FB_DEBUG(
      "Measurement rejected by Mahalanobis threshold\n" <<
        "---------------------- /Eskf::correct (rejected) ----------------------\n");
    return;
  }

  // Step 7: Update error state
  Eigen::VectorXd error_before_update = state_;
  Eigen::VectorXd error_correction = K * innovation;
  state_ += error_correction;

  FB_DEBUG(
    "--- ERROR STATE UPDATE ---\n" <<
      "Before update: " << error_before_update.transpose() << "\n" <<
      "Kalman gain × innovation: " << error_correction.transpose() << "\n" <<
      "After update:  " << state_.transpose() << "\n");

  // ========== ERROR STATE UPDATE INFO ==========
  if (output_enabled) {
    std::cout << "\n┌─── Step 4: Error State Update ────────────────────────────┐\n";
    std::cout << "│ Error before: norm=" << std::setw(10) << error_before_update.norm() << "\n";
    std::cout << "│   " << error_before_update.transpose() << "\n";
    std::cout << "│\n│ K × innovation:\n";
    std::cout << "│   " << error_correction.transpose() << "\n";
    std::cout << "│\n│ Error after:  norm=" << std::setw(10) << state_.norm() << "\n";
    std::cout << "│   " << state_.transpose() << "\n";
    std::cout << "└───────────────────────────────────────────────────────────┘\n";
  }

  // Step 8: Update error covariance (Joseph form for numerical stability)
  Eigen::MatrixXd I_KH =
    Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H;
  estimate_error_covariance_ =
    I_KH * estimate_error_covariance_ * I_KH.transpose() +
    K * R * K.transpose();

  // Ensure symmetry
  estimate_error_covariance_ =
    0.5 * (estimate_error_covariance_ + estimate_error_covariance_.transpose());

  // Step 9: Inject error state into nominal state
  // Save state before injection for comparison
  Eigen::Vector3d pos_before_inject = nominal_position_;
  Eigen::Vector3d vel_before_inject = nominal_linear_velocity_;
  Eigen::Quaterniond quat_before_inject = nominal_orientation_;
  
  FB_DEBUG("--- INJECTING ERROR INTO NOMINAL STATE ---\n");
  injectErrorState();
  
  // Step 10: Reset error state
  FB_DEBUG("--- RESETTING ERROR STATE TO ZERO ---\n");
  resetErrorState();

  // Calculate changes
  Eigen::Vector3d euler_pre = quaternionToEuler(pre_nominal_quat);
  Eigen::Vector3d euler_post = quaternionToEuler(nominal_orientation_);
  Eigen::Vector3d pos_change = nominal_position_ - pre_nominal_pos;
  Eigen::Vector3d vel_change = nominal_linear_velocity_ - vel_before_inject;
  Eigen::Vector3d angle_change = euler_post - euler_pre;
  
  // Normalize angle change
  angle_change(0) = angles::normalize_angle(angle_change(0));
  angle_change(1) = angles::normalize_angle(angle_change(1));
  angle_change(2) = angles::normalize_angle(angle_change(2));
  
  FB_DEBUG(
    "--- CORRECTION SUMMARY ---\n" <<
      "Position change: [" << pos_change(0) << ", " << pos_change(1) << ", " << pos_change(2) << "]\n" <<
      "Final nominal position: [" << nominal_position_(0) << ", " << nominal_position_(1) << ", " << nominal_position_(2) << "]\n" <<
      "Final nominal orientation: [R=" << euler_post(0)*180.0/M_PI << "°, " <<
                                    "P=" << euler_post(1)*180.0/M_PI << "°, " <<
                                    "Y=" << euler_post(2)*180.0/M_PI << "°]\n" <<
      "Error state (should be ~0): " << state_.norm() << "\n" <<
      "========== ESKF CORRECT END ==========\n\n");

  // ========== FINAL SUMMARY ==========
  if (output_enabled) {
    std::cout << "\n┌─── Step 5: Injection & Final State ───────────────────────┐\n";
    std::cout << "│ POSITION:\n";
    std::cout << "│   Before:  [" << std::setw(9) << std::fixed << std::setprecision(4) 
              << pre_nominal_pos(0) << ", " << pre_nominal_pos(1) << ", " << pre_nominal_pos(2) << "]\n";
    std::cout << "│   After:   [" << std::setw(9) << nominal_position_(0) << ", " 
              << nominal_position_(1) << ", " << nominal_position_(2) << "]\n";
    std::cout << "│   Change:  [" << std::setw(9) << pos_change(0) << ", " 
              << pos_change(1) << ", " << pos_change(2) << "] (norm=" 
              << std::setprecision(6) << pos_change.norm() << "m)\n";
    std::cout << "│\n│ ORIENTATION (RPY in degrees):\n";
    std::cout << "│   Before:  [" << std::setw(8) << std::setprecision(2) 
              << euler_pre(0)*180.0/M_PI << "°, " << euler_pre(1)*180.0/M_PI << "°, " 
              << euler_pre(2)*180.0/M_PI << "°]\n";
    std::cout << "│   After:   [" << std::setw(8) << euler_post(0)*180.0/M_PI << "°, " 
              << euler_post(1)*180.0/M_PI << "°, " << euler_post(2)*180.0/M_PI << "°]\n";
    std::cout << "│   Change:  [" << std::setw(8) << angle_change(0)*180.0/M_PI << "°, " 
              << angle_change(1)*180.0/M_PI << "°, " << angle_change(2)*180.0/M_PI << "°]\n";
    std::cout << "│\n│ VELOCITY (body frame):\n";
    std::cout << "│   Before:  [" << std::setw(9) << std::setprecision(4) 
              << vel_before_inject(0) << ", " << vel_before_inject(1) << ", " 
              << vel_before_inject(2) << "]\n";
    std::cout << "│   After:   [" << std::setw(9) << nominal_linear_velocity_(0) << ", " 
              << nominal_linear_velocity_(1) << ", " << nominal_linear_velocity_(2) << "]\n";
    std::cout << "│   Change:  [" << std::setw(9) << vel_change(0) << ", " 
              << vel_change(1) << ", " << vel_change(2) << "]\n";
    std::cout << "│\n│ Error state after reset: " << std::scientific 
              << std::setprecision(2) << state_.norm() << " (should be ~0)\n";
    std::cout << "└───────────────────────────────────────────────────────────┘\n";
    std::cout << std::flush;
  }
}

void Eskf::injectErrorState()
{
  // Extract error state components
  Eigen::Vector3d delta_position = state_.segment<3>(StateMemberX);
  Eigen::Vector3d delta_angle(
    state_(StateMemberRoll),
    state_(StateMemberPitch),
    state_(StateMemberYaw));
  Eigen::Vector3d delta_velocity = state_.segment<3>(StateMemberVx);
  Eigen::Vector3d delta_angular_velocity = state_.segment<3>(StateMemberVroll);
  Eigen::Vector3d delta_acceleration = state_.segment<3>(StateMemberAx);

  FB_DEBUG(
    "  Error state before injection:\n" <<
      "    δp = [" << delta_position(0) << ", " << delta_position(1) << ", " << delta_position(2) << "]\n" <<
      "    δθ = [" << delta_angle(0)*180.0/M_PI << "°, " << delta_angle(1)*180.0/M_PI << "°, " << delta_angle(2)*180.0/M_PI << "°]\n" <<
      "    δv = [" << delta_velocity(0) << ", " << delta_velocity(1) << ", " << delta_velocity(2) << "]\n");

  Eigen::Vector3d old_pos = nominal_position_;
  Eigen::Quaterniond old_quat = nominal_orientation_;

  // 1. Position injection: p̄⁺ = p̄ + δp
  nominal_position_ += delta_position;

  // 2. Orientation injection: q̄⁺ = q̄ ⊗ exp(δθ)
  Eigen::Quaterniond delta_q = axisAngleToQuaternion(delta_angle);
  nominal_orientation_ = nominal_orientation_ * delta_q;
  nominal_orientation_.normalize();

  // 3. Velocity injection: v̄⁺ = v̄ + δv
  nominal_linear_velocity_ += delta_velocity;

  // 4. Angular velocity injection: ω̄⁺ = ω̄ + δω
  nominal_angular_velocity_ += delta_angular_velocity;

  // 5. Acceleration injection: ā⁺ = ā + δa
  nominal_linear_acceleration_ += delta_acceleration;

  Eigen::Vector3d euler_old = quaternionToEuler(old_quat);
  Eigen::Vector3d euler_new = quaternionToEuler(nominal_orientation_);

  FB_DEBUG(
    "  Injection results:\n" <<
      "    Position:    [" << old_pos(0) << ", " << old_pos(1) << ", " << old_pos(2) << 
            "] → [" << nominal_position_(0) << ", " << nominal_position_(1) << ", " << nominal_position_(2) << "]\n" <<
      "    Orientation: [" << euler_old(0)*180.0/M_PI << "°, " << euler_old(1)*180.0/M_PI << "°, " << euler_old(2)*180.0/M_PI << 
            "°] → [" << euler_new(0)*180.0/M_PI << "°, " << euler_new(1)*180.0/M_PI << "°, " << euler_new(2)*180.0/M_PI << "°]\n" <<
      "    |δp| = " << delta_position.norm() << " m\n" <<
      "    |δθ| = " << delta_angle.norm() * 180.0 / M_PI << " degrees\n" <<
      "    |δv| = " << delta_velocity.norm() << " m/s\n");
}

void Eskf::resetErrorState()
{
  Eigen::VectorXd old_error = state_;
  
  // Reset error state to zero
  state_.setZero();

  FB_DEBUG(
    "  Error state reset:\n" <<
      "    Before: norm = " << old_error.norm() << "\n" <<
      "    After:  norm = " << state_.norm() << " (should be ~0)\n" <<
      "  Note: Error covariance NOT reset (maintains uncertainty estimate)\n");

  // In a full implementation, we would apply the reset Jacobian:
  // G = ∂(δx⁺)/∂(δx⁻)
  // P⁺ = G * P⁻ * G^T
  //
  // For small angles, G ≈ I, so we keep the covariance unchanged.
  // This is a valid approximation for most cases.
  //
  // The full reset Jacobian accounts for the quaternion composition:
  // For orientation: G_θθ = I - 0.5 * [δθ]×
  // For other states: G = I
  //
  // Since we reset frequently and errors are small, the approximation is good.
}

Eigen::VectorXd Eskf::nominalStateToVector()
{
  Eigen::VectorXd vec(STATE_SIZE);

  // Position
  vec.segment<3>(StateMemberX) = nominal_position_;

  // Orientation (convert quaternion to Euler angles)
  Eigen::Vector3d euler = quaternionToEuler(nominal_orientation_);
  vec(StateMemberRoll) = euler(0);
  vec(StateMemberPitch) = euler(1);
  vec(StateMemberYaw) = euler(2);

  // Linear velocity
  vec.segment<3>(StateMemberVx) = nominal_linear_velocity_;

  // Angular velocity
  vec.segment<3>(StateMemberVroll) = nominal_angular_velocity_;

  // Linear acceleration
  vec.segment<3>(StateMemberAx) = nominal_linear_acceleration_;

  return vec;
}

void Eskf::vectorToNominalState(const Eigen::VectorXd & state_vec)
{
  // Position
  nominal_position_ = state_vec.segment<3>(StateMemberX);

  // Orientation (convert Euler angles to quaternion)
  Eigen::Vector3d euler(
    state_vec(StateMemberRoll),
    state_vec(StateMemberPitch),
    state_vec(StateMemberYaw));
  nominal_orientation_ = eulerToQuaternion(euler);

  // Linear velocity
  nominal_linear_velocity_ = state_vec.segment<3>(StateMemberVx);

  // Angular velocity
  nominal_angular_velocity_ = state_vec.segment<3>(StateMemberVroll);

  // Linear acceleration
  nominal_linear_acceleration_ = state_vec.segment<3>(StateMemberAx);
}

// ========== Mathematical Utility Functions ==========

Eigen::Matrix3d Eskf::skewSymmetric(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v(2), v(1),
    v(2), 0.0, -v(0),
    -v(1), v(0), 0.0;
  return m;
}

Eigen::Quaterniond Eskf::axisAngleToQuaternion(const Eigen::Vector3d & axis_angle)
{
  double angle = axis_angle.norm();

  if (angle < 1e-10) {
    // Small angle approximation: q ≈ [1, 0.5*θx, 0.5*θy, 0.5*θz]
    return Eigen::Quaterniond(
      1.0,
      0.5 * axis_angle(0),
      0.5 * axis_angle(1),
      0.5 * axis_angle(2));
  }

  // General case
  Eigen::Vector3d axis = axis_angle / angle;
  double half_angle = 0.5 * angle;
  double s = std::sin(half_angle);

  return Eigen::Quaterniond(
    std::cos(half_angle),
    s * axis(0),
    s * axis(1),
    s * axis(2));
}

Eigen::Vector3d Eskf::quaternionToEuler(const Eigen::Quaterniond & q)
{
  // ZYX Euler angle convention (Yaw-Pitch-Roll)
  // This matches the convention used in robot_localization

  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  double pitch;
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);  // Use 90 degrees if out of range
  } else {
    pitch = std::asin(sinp);
  }

  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond Eskf::eulerToQuaternion(const Eigen::Vector3d & euler)
{
  // ZYX Euler angle convention (Yaw-Pitch-Roll)
  double roll = euler(0);
  double pitch = euler(1);
  double yaw = euler(2);

  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q;
}

Eigen::Matrix4d Eskf::omegaMatrix(const Eigen::Vector3d & omega)
{
  Eigen::Matrix4d Omega;
  Omega << 0.0, -omega(0), -omega(1), -omega(2),
    omega(0), 0.0, omega(2), -omega(1),
    omega(1), -omega(2), 0.0, omega(0),
    omega(2), omega(1), -omega(0), 0.0;
  return Omega;
}

}  // namespace robot_localization
