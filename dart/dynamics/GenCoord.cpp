/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include <cassert>

#include "dart/math/MathTypes.h"
#include "dart/dynamics/GenCoord.h"

namespace dart {
namespace dynamics {

//==============================================================================
GenCoord::GenCoord()
  : mConfig(0.0),
    mVel(0.0),
    mAcc(0.0),
    mForce(0.0),
    mConfigMin(-DART_DBL_INF),
    mVelMin(-DART_DBL_INF),
    mAccMin(-DART_DBL_INF),
    mForceMin(-DART_DBL_INF),
    mConfigMax(DART_DBL_INF),
    mVelMax(DART_DBL_INF),
    mAccMax(DART_DBL_INF),
    mForceMax(DART_DBL_INF),
    mConfigDeriv(0.0),
    mVelDeriv(0.0),
    mAccDeriv(0.0),
    mForceDeriv(0.0),
    mSkelIndex(0u),
    mName("dof")
{
}

//==============================================================================
GenCoord::~GenCoord()
{
}

//==============================================================================
void GenCoord::setName(const std::string& _name)
{
  mName = _name;
}

//==============================================================================
const std::string& GenCoord::getName() const
{
  return mName;
}

//==============================================================================
size_t GenCoord::getSkeletonIndex() const
{
  return mSkelIndex;
}

//==============================================================================
double GenCoord::getConfig() const
{
  return mConfig;
}

//==============================================================================
double GenCoord::getVel() const
{
  return mVel;
}

//==============================================================================
double GenCoord::getAcc() const
{
  return mAcc;
}

//==============================================================================
double GenCoord::getForce() const
{
  return mForce;
}

//==============================================================================
double GenCoord::getConfigMin() const
{
  return mConfigMin;
}

//==============================================================================
double GenCoord::getVelMin() const
{
  return mVelMin;
}

//==============================================================================
double GenCoord::getAccMin() const
{
  return mAccMin;
}

//==============================================================================
double GenCoord::getForceMin() const
{
  return mForceMin;
}

//==============================================================================
double GenCoord::getConfigMax() const
{
  return mConfigMax;
}

//==============================================================================
void GenCoord::setConfigDeriv(double _configDeriv)
{
  assert(!math::isNan(_configDeriv));
  mConfigDeriv = _configDeriv;
}

//==============================================================================
double GenCoord::getConfigDeriv() const
{
  return mConfigDeriv;
}

//==============================================================================
double GenCoord::getVelMax() const
{
  return mVelMax;
}

//==============================================================================
void GenCoord::setVelDeriv(double _velDeriv)
{
  assert(!math::isNan(_velDeriv));
  mVelDeriv = _velDeriv;
}

//==============================================================================
double GenCoord::getVelDeriv() const
{
  return mVelDeriv;
}

//==============================================================================
double GenCoord::getAccMax() const
{
  return mAccMax;
}

//==============================================================================
void GenCoord::setAccDeriv(double _accDeriv)
{
  assert(!math::isNan(_accDeriv));
  mAccDeriv = _accDeriv;
}

//==============================================================================
double GenCoord::getAccDeriv() const
{
  return mAccDeriv;
}

//==============================================================================
double GenCoord::getForceMax() const
{
  return mForceMax;
}

//==============================================================================
void GenCoord::setForceDeriv(double _forceDeriv)
{
  assert(!math::isNan(_forceDeriv));
  mForceDeriv = _forceDeriv;
}

//==============================================================================
double GenCoord::getForceDeriv() const
{
  return mForceDeriv;
}

//==============================================================================
void GenCoord::setConfig(double _config)
{
  assert(!math::isNan(_config));
  mConfig = _config;
}

//==============================================================================
void GenCoord::setVel(double _vel)
{
  assert(!math::isNan(_vel));
  mVel = _vel;
}

//==============================================================================
void GenCoord::setAcc(double _acc)
{
  assert(!math::isNan(_acc));
  mAcc = _acc;
}

//==============================================================================
void GenCoord::setForce(double _force)
{
  assert(!math::isNan(_force));
  mForce = _force;
}

//==============================================================================
void GenCoord::setConfigMin(double _configMin)
{
  mConfigMin = _configMin;
}

//==============================================================================
void GenCoord::setVelMin(double _velMin)
{
  mVelMin = _velMin;
}

//==============================================================================
void GenCoord::setAccMin(double _accMin)
{
  mAccMin = _accMin;
}

//==============================================================================
void GenCoord::setForceMin(double _forceMin)
{
  mForceMin = _forceMin;
}

//==============================================================================
void GenCoord::setConfigMax(double _configMax)
{
  mConfigMax = _configMax;
}

//==============================================================================
void GenCoord::setVelMax(double _velMax)
{
  mVelMax = _velMax;
}

//==============================================================================
void GenCoord::setAccMax(double _accMax)
{
  mAccMax = _accMax;
}

//==============================================================================
void GenCoord::setForceMax(double _forceMax)
{
  mForceMax = _forceMax;
}

//==============================================================================
void GenCoord::setSkeletonIndex(size_t _idx)
{
  mSkelIndex = _idx;
}

}  // namespace dynamics
}  // namespace dart
