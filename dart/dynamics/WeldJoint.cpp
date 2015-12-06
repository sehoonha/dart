/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include "dart/dynamics/WeldJoint.h"

namespace dart {
namespace dynamics {

//==============================================================================
WeldJoint::Properties::Properties(const Joint::Properties& properties)
  : GenericJoint<NullSpace>::Properties(properties)
{
  // Do nothing
}

//==============================================================================
WeldJoint::~WeldJoint()
{
  // Do nothing
}

//==============================================================================
const std::string& WeldJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& WeldJoint::getStaticType()
{
  static const std::string name = "WeldJoint";
  return name;
}

//==============================================================================
bool WeldJoint::isCyclic(size_t /*index*/) const
{
  return false;
}

//==============================================================================
WeldJoint::Properties WeldJoint::getWeldJointProperties() const
{
  return getGenericJointProperties();
}

//==============================================================================
void WeldJoint::setTransformFromParentBodyNode(const Eigen::Isometry3d& T)
{
  Joint::setTransformFromParentBodyNode(T);

  mT = mJointP.mT_ParentBodyToJoint * mJointP.mT_ChildBodyToJoint.inverse();
}

//==============================================================================
void WeldJoint::setTransformFromChildBodyNode(const Eigen::Isometry3d& T)
{
  Joint::setTransformFromChildBodyNode(T);

  mT = mJointP.mT_ParentBodyToJoint * mJointP.mT_ChildBodyToJoint.inverse();
}

//==============================================================================
const WeldJoint::JacobianMatrix WeldJoint::getLocalJacobianStatic(
    const GenericJoint::Vector& /*positions*/) const
{
  return JacobianMatrix();
}

//==============================================================================
WeldJoint::WeldJoint(const Properties& properties)
  : GenericJoint<NullSpace>(properties)
{
  setProperties(properties);
}

//==============================================================================
Joint* WeldJoint::clone() const
{
  return new WeldJoint(getWeldJointProperties());
}

//==============================================================================
void WeldJoint::updateDegreeOfFreedomNames()
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateLocalTransform() const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateLocalSpatialVelocity() const
{
  // Do nothing
  // Should we have mSpatialVelocity.setZero() here instead?
}

//==============================================================================
void WeldJoint::updateLocalSpatialAcceleration() const
{
  // Do nothing
  // Should we have mSpatialAcceleration.setZero() here instead?
}

//==============================================================================
void WeldJoint::updateLocalPrimaryAcceleration() const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateLocalJacobian(bool /*mandatory*/) const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateLocalJacobianTimeDeriv() const
{
  // Do nothing
}

}  // namespace dynamics
}  // namespace dart
