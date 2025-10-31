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

#ifndef ROBOT_LOCALIZATION__ESKF_HPP_
#define ROBOT_LOCALIZATION__ESKF_HPP_

#include <robot_localization/filter_base.hpp>
#include <Eigen/Geometry>

namespace robot_localization
{

/**
 * @brief Error-State Kalman Filter class
 *
 * Implementation of an Error-State Kalman filter (ESKF). This class derives from
 * FilterBase and overrides the predict() and correct() methods in keeping with
 * the ESKF algorithm.
 * 
 * The ESKF maintains two states:
 * - Nominal state (16-dim): Uses quaternion for orientation [p, q, v, ω, a]
 * - Error state (15-dim): Uses minimal representation [δp, δθ, δv, δω, δa]
 * 
 * Key advantages:
 * - No gimbal lock (quaternion representation)
 * - Better numerical stability (error state reset)
 * - More accurate for large angle rotations
 * - Linearization around zero error (higher accuracy)
 * 
 * Reference:
 * Solà, Joan. "Quaternion kinematics for the error-state Kalman filter." 
 * arXiv preprint arXiv:1711.02508 (2017).
 */
class Eskf : public FilterBase
{
public:
  /**
   * @brief Constructor for the Eskf class
   */
  Eskf();

  /**
   * @brief Destructor for the Eskf class
   */
  ~Eskf();

  /**
   * @brief Set the filter state (updates nominal state for ESKF)
   *
   * For ESKF, the "state" from external perspective is the nominal state.
   * This method updates the nominal state and resets error state to zero.
   *
   * @param[in] state - The new state vector (15-dimensional)
   */
  void setState(const Eigen::VectorXd & state);

  /**
   * @brief Get the filter state (returns nominal state for ESKF)
   *
   * For ESKF, the "state" from external perspective is the nominal state.
   * This method converts the nominal state to a 15-dimensional vector.
   *
   * @return The current nominal state as 15-dimensional vector
   */
  const Eigen::VectorXd & getState();

  /**
   * @brief Carries out the correct step in the predict/update cycle.
   *
   * This method:
   * 1. Computes the innovation (measurement residual)
   * 2. Computes the Kalman gain
   * 3. Updates the error state
   * 4. Updates the error covariance
   * 5. Injects error state into nominal state
   * 6. Resets error state to zero
   *
   * @param[in] measurement - The measurement to fuse with our estimate
   */
  void correct(const Measurement & measurement) override;

  /**
   * @brief Carries out the predict step in the predict/update cycle.
   *
   * This method:
   * 1. Propagates the nominal state (no noise, using kinematics)
   * 2. Propagates the error covariance (linearized dynamics with noise)
   *
   * @param[in] reference_time - The time at which the prediction is being made
   * @param[in] delta - The time step over which to predict
   */
  void predict(
    const rclcpp::Time & reference_time,
    const rclcpp::Duration & delta) override;

protected:
  // ========== Nominal State (16-dimensional) ==========
  
  /**
   * @brief Nominal position [x, y, z] in world frame
   */
  Eigen::Vector3d nominal_position_;

  /**
   * @brief Nominal orientation as unit quaternion [qw, qx, qy, qz]
   * 
   * This avoids gimbal lock and provides better numerical properties
   * compared to Euler angles for large rotations.
   */
  Eigen::Quaterniond nominal_orientation_;

  /**
   * @brief Nominal linear velocity [vx, vy, vz] in world frame
   */
  Eigen::Vector3d nominal_linear_velocity_;

  /**
   * @brief Nominal angular velocity [ωx, ωy, ωz] in body frame
   */
  Eigen::Vector3d nominal_angular_velocity_;

  /**
   * @brief Nominal linear acceleration [ax, ay, az] in body frame
   */
  Eigen::Vector3d nominal_linear_acceleration_;

  // ========== Error State (15-dimensional, stored in FilterBase::state_) ==========
  // state_ = [δp(3), δθ(3), δv(3), δω(3), δa(3)]
  // δp: position error
  // δθ: orientation error (rotation vector / axis-angle)
  // δv: linear velocity error
  // δω: angular velocity error
  // δa: linear acceleration error

  // ========== Auxiliary Variables ==========

  /**
   * @brief Gravity vector in world frame [0, 0, -9.81]
   */
  Eigen::Vector3d gravity_;

  // ========== Private Methods ==========

  /**
   * @brief Propagate the nominal state using kinematic equations (no noise)
   *
   * Updates:
   * - Position: p = p + v*dt + 0.5*R(q)*a*dt^2
   * - Orientation: q = q ⊗ exp(ω*dt)
   * - Velocity: v = v + (R(q)*a + g)*dt
   * - Angular velocity: ω (from control or unchanged)
   * - Acceleration: a (from control or unchanged)
   *
   * @param[in] dt - Time step in seconds
   */
  void propagateNominalState(double dt);

  /**
   * @brief Propagate the error covariance using linearized dynamics
   *
   * Computes:
   * - Error state transition matrix F
   * - Discretized transition Φ = I + F*dt
   * - Covariance propagation P = Φ*P*Φ^T + Q*dt
   *
   * @param[in] dt - Time step in seconds
   */
  void propagateErrorCovariance(double dt);

  /**
   * @brief Build the error state transition matrix F
   *
   * F is a 15x15 matrix that describes how the error state evolves:
   *        δp      δθ      δv      δω      δa
   * δp  [  0    -R[v]×     R       0       0     ]
   * δθ  [  0    -[ω]×      0      -I       0     ]
   * δv  [  0      0        0       0       I     ]
   * δω  [  0      0        0       0       0     ]
   * δa  [  0      0        0       0       0     ]
   *
   * NOTE: This assumes velocity is in BODY frame (which matches
   * the robot_localization convention for twist measurements).
   *
   * @param[out] F - The 15x15 state transition matrix
   * @param[in] dt - Time step (for optional damping terms)
   */
  void buildErrorStateTransition(
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> & F,
    double dt);

  /**
   * @brief Inject the error state into the nominal state
   *
   * Updates:
   * - Position: p = p + δp
   * - Orientation: q = q ⊗ exp(δθ)
   * - Velocity: v = v + δv
   * - Angular velocity: ω = ω + δω
   * - Acceleration: a = a + δa
   */
  void injectErrorState();

  /**
   * @brief Reset the error state to zero after injection
   *
   * Sets state_ = 0 and optionally updates covariance with reset Jacobian.
   * For small angles, the reset Jacobian is approximately identity, so we
   * keep the covariance unchanged in this simplified implementation.
   */
  void resetErrorState();

  /**
   * @brief Convert nominal state to 15-dimensional vector for measurement prediction
   *
   * The output vector follows the STATE_SIZE ordering:
   * [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
   *
   * Note: Orientation is converted from quaternion to Euler angles.
   *
   * @return 15-dimensional state vector
   */
  Eigen::VectorXd nominalStateToVector();

  /**
   * @brief Initialize nominal state from 15-dimensional vector
   *
   * Used during filter initialization to set up the nominal state from
   * the first measurement or initial estimate.
   *
   * @param[in] state_vec - 15-dimensional state vector
   */
  void vectorToNominalState(const Eigen::VectorXd & state_vec);

  // ========== Mathematical Utility Functions ==========

  /**
   * @brief Compute skew-symmetric matrix (cross product operator)
   *
   * For vector v = [v1, v2, v3]^T, returns:
   *     [  0  -v3   v2 ]
   * [v]×= [ v3   0  -v1 ]
   *     [-v2  v1    0  ]
   *
   * Such that [v]× * w = v × w (cross product)
   *
   * @param[in] v - 3D vector
   * @return 3x3 skew-symmetric matrix
   */
  Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & v);

  /**
   * @brief Convert axis-angle (rotation vector) to quaternion
   *
   * For small angles: q ≈ [1, 0.5*θx, 0.5*θy, 0.5*θz]
   * For general case: q = [cos(θ/2), sin(θ/2)*axis]
   * where θ = ||axis_angle|| and axis = axis_angle/θ
   *
   * @param[in] axis_angle - 3D rotation vector
   * @return Unit quaternion
   */
  Eigen::Quaterniond axisAngleToQuaternion(const Eigen::Vector3d & axis_angle);

  /**
   * @brief Convert quaternion to Euler angles (ZYX convention)
   *
   * Returns [roll, pitch, yaw] in radians.
   * - Roll: rotation about X axis
   * - Pitch: rotation about Y axis
   * - Yaw: rotation about Z axis
   *
   * @param[in] q - Unit quaternion
   * @return 3D vector [roll, pitch, yaw]
   */
  Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond & q);

  /**
   * @brief Convert Euler angles to quaternion (ZYX convention)
   *
   * @param[in] euler - 3D vector [roll, pitch, yaw]
   * @return Unit quaternion
   */
  Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d & euler);

  /**
   * @brief Compute quaternion from omega matrix for integration
   *
   * Ω(ω) is a 4x4 matrix used in quaternion differential equation:
   * q̇ = 0.5 * Ω(ω) * q
   *
   *       [  0  -ωx -ωy -ωz ]
   * Ω(ω) = [ ωx   0   ωz -ωy ]
   *       [ ωy -ωz   0   ωx ]
   *       [ ωz  ωy -ωx   0  ]
   *
   * @param[in] omega - Angular velocity vector
   * @return 4x4 quaternion derivative matrix
   */
  Eigen::Matrix4d omegaMatrix(const Eigen::Vector3d & omega);
};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__ESKF_HPP_
