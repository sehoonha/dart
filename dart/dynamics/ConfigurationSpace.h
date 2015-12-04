/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_DYNAMICS_CONFIGURATIONSPACE_H_
#define DART_DYNAMICS_CONFIGURATIONSPACE_H_

#include <Eigen/Core>

#include "dart/math/Geometry.h"
#include "dart/dynamics/Joint.h"

namespace dart {
namespace dynamics {

template <int Dimension>
struct RealVectorSpace
{
  static const int NumDofs = Dimension;

  using TangentSpace = RealVectorSpace<NumDofs>;

  using Point          = Eigen::Matrix<double, NumDofs, 1>;
  using EuclideanPoint = Eigen::Matrix<double, NumDofs, 1>;
  using Vector         = Eigen::Matrix<double, NumDofs, 1>;
  using Matrix         = Eigen::Matrix<double, NumDofs, NumDofs>;
  using JacobianMatrix = Eigen::Matrix<double, 6, NumDofs>;
};

using NullSpace = RealVectorSpace<0>;
using RealSpace = RealVectorSpace<1>;

//struct SO2Space
//{
//  static const int NumDofs = 1;

//  using TangentSpace = RealVectorSpace<NumDofs>;

//  using Point          = Eigen::Matrix2d;
//  using EuclideanPoint = Eigen::Matrix<double, NumDofs, 1>;
//  using Vector         = Eigen::Matrix<double, NumDofs, 1>;
//  using Matrix         = Eigen::Matrix<double, NumDofs, NumDofs>;
//  using JacobianMatrix = Eigen::Matrix<double, 6, NumDofs>;
//};
// TODO(JS): Not implemented yet.

//struct SE2Space
//{
//  static const int NumDofs = 3;

//  using TangentSpace = RealVectorSpace<NumDofs>;

//  using Point          = Eigen::Isometry2d;
//  using EuclideanPoint = Eigen::Matrix<double, NumDofs, 1>;
//  using Vector         = Eigen::Matrix<double, NumDofs, 1>;
//  using Matrix         = Eigen::Matrix<double, NumDofs, NumDofs>;
//  using JacobianMatrix = Eigen::Matrix<double, 6, NumDofs>;
//};
// TODO(JS): Not implemented yet.

struct SO3Space
{
  static const int NumDofs = 3;

  using TangentSpace = RealVectorSpace<NumDofs>;

  using Point          = Eigen::Matrix3d;
  using EuclideanPoint = Eigen::Vector3d;
  using Vector         = Eigen::Vector3d;
  using Matrix         = Eigen::Matrix3d;
  using JacobianMatrix = Eigen::Matrix<double, 6, NumDofs>;
};

struct SE3Space
{
  static const int NumDofs = 6;

  using TangentSpace = RealVectorSpace<NumDofs>;

  using Point          = Eigen::Isometry3d;
  using EuclideanPoint = Eigen::Vector6d;
  using Vector         = Eigen::Vector6d;
  using Matrix         = Eigen::Matrix6d;
  using JacobianMatrix = Eigen::Matrix6d;
};

//==============================================================================
// detail
//==============================================================================

namespace detail {

template<bool> struct Range;

// General case implementation
template <typename MatrixType, int Size, typename Enable = Range<true>>
struct compute_inverse
{
  static inline void run(const MatrixType& matrix, MatrixType& result)
  {
    result = matrix.ldlt().solve(MatrixType::Identity());
  }
};

template <typename MatrixType, int Size>
struct compute_inverse<MatrixType, Size, Range<(0 <= Size && Size <= 4)>>
{
  static inline void run(const MatrixType& matrix, MatrixType& result)
  {
    result = matrix.inverse();
  }
};

template <typename SpaceType>
inline typename SpaceType::Matrix inverse(const typename SpaceType::Matrix& mat)
{
  typename SpaceType::Matrix res;

  compute_inverse<typename SpaceType::Matrix,
                  SpaceType::NumDofs>::run(mat, res);

  return res;
}

// General case implementation
template <typename SpaceType>
struct mapToEuclideanPointImpl
{
  static inline typename SpaceType::EuclideanPoint
  run(const typename SpaceType::Point& point)
  {
    return point;
  }
};

template <>
struct mapToEuclideanPointImpl<SO3Space>
{
  static inline typename SO3Space::EuclideanPoint
  run(const typename SO3Space::Point& point)
  {
    return math::logMap(point);
  }
};

template <>
struct mapToEuclideanPointImpl<SE3Space>
{
  static inline typename SE3Space::EuclideanPoint
  run(const typename SE3Space::Point& point)
  {
    Eigen::Vector6d x;

    x.head<3>() = math::logMap(point.linear());
    x.tail<3>() = point.translation();

    return x;
  }
};

template <typename SpaceType>
inline typename SpaceType::EuclideanPoint
mapToEuclideanPoint(const typename SpaceType::Point& point)
{
  return mapToEuclideanPointImpl<SpaceType>::run(point);
}

// General case implementation
template <typename SpaceType>
struct mapToManifoldPointImpl
{
  static inline typename SpaceType::Point
  run(const typename SpaceType::EuclideanPoint& point)
  {
    return point;
  }
};

template <>
struct mapToManifoldPointImpl<SO3Space>
{
  static inline typename SO3Space::Point
  run(const typename SO3Space::EuclideanPoint& point)
  {
    return math::expMapRot(point);
  }
};

template <>
struct mapToManifoldPointImpl<SE3Space>
{
  static inline typename SE3Space::Point
  run(const typename SE3Space::EuclideanPoint& point)
  {
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    tf.linear() = math::expMapRot(point.head<3>());
    tf.translation() = point.tail<3>();

    return tf;
  }
};

template <typename SpaceType>
inline typename SpaceType::Point
mapToManifoldPoint(const typename SpaceType::EuclideanPoint& point)
{
  return mapToManifoldPointImpl<SpaceType>::run(point);
}

// General case implementation
template <typename SpaceType>
struct integratePositionImpl
{
  static inline typename SpaceType::Point run(
      const typename SpaceType::Point& pos,
      const typename SpaceType::Vector& vel,
      double dt)
  {
    return pos + dt * vel;
  }
};

template <>
struct integratePositionImpl<SO3Space>
{
  static inline typename SO3Space::Point run(
      const typename SO3Space::Point& pos,
      const typename SO3Space::Vector& vel,
      double dt)
  {
    return pos * mapToManifoldPoint<SO3Space>(vel * dt);
  }
};

template <>
struct integratePositionImpl<SE3Space>
{
  static inline typename SE3Space::Point run(
      const typename SE3Space::Point& pos,
      const typename SE3Space::Vector& vel,
      double dt)
  {
    return pos * mapToManifoldPoint<SE3Space>(vel * dt);
  }
};

template <typename SpaceType>
inline typename SpaceType::Point integratePosition(
    const typename SpaceType::Point& pos,
    const typename SpaceType::Vector& vel,
    double dt)
{
  return integratePositionImpl<SpaceType>::run(pos, vel, dt);
}

template <typename SpaceType>
inline typename SpaceType::Vector integrateVelocity(
    const typename SpaceType::Vector& vel,
    const typename SpaceType::Vector& acc,
    double dt)
{
  return vel + acc * dt;
}

} // namespace detail

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_CONFIGURATIONSPACE_H_
