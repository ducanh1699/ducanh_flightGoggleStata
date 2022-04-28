/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Common library
 *
 * Common library for geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>

static Eigen::Matrix3d matrix_hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  // Sanity checks on M
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}

static Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m) {
  Eigen::Vector3d v;
  // TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
  Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
  return ev3;
}

Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
      2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Vector3f quat2Euler(const Eigen::Vector4d &q){
  Eigen::Vector3f angles;
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
  double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
  angles(0) = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q(0) * q(2) - q(3) * q(1));
  if (std::abs(sinp) >= 1)
      angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      angles(1) = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
  double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
  angles(2) = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}
Eigen::Vector4d ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Vector4d q;
    q(3) = cr * cp * cy + sr * sp * sy;
    q(0) = sr * cp * cy - cr * sp * sy;
    q(1) = cr * sp * cy + sr * cp * sy;
    q(2) = cr * cp * sy - sr * sp * cy;

    return q;
}
#endif
