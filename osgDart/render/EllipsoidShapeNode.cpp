/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Light>
#include <osg/Material>

#include "osgDart/render/EllipsoidShapeNode.h"
#include "osgDart/Utils.h"

#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/SimpleFrame.h"

namespace osgDart {
namespace render {

class EllipsoidShapeGeode : public ShapeNode, public osg::Geode
{
public:

  EllipsoidShapeGeode(dart::dynamics::EllipsoidShape* shape,
                      ShapeFrameNode* parentShapeFrame,
                      EllipsoidShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~EllipsoidShapeGeode();

  EllipsoidShapeNode* mParentNode;
  dart::dynamics::EllipsoidShape* mEllipsoidShape;
  EllipsoidShapeDrawable* mDrawable;

};

//==============================================================================
class EllipsoidShapeDrawable : public osg::ShapeDrawable
{
public:

  EllipsoidShapeDrawable(dart::dynamics::EllipsoidShape* shape,
                         dart::dynamics::VisualAddon* visualAddon,
                         EllipsoidShapeGeode* parent);

  void refresh(bool firstTime);

protected:

  virtual ~EllipsoidShapeDrawable();

  dart::dynamics::EllipsoidShape* mEllipsoidShape;
  dart::dynamics::VisualAddon* mVisualAddon;
  EllipsoidShapeGeode* mParent;

};

//==============================================================================
EllipsoidShapeNode::EllipsoidShapeNode(
    std::shared_ptr<dart::dynamics::EllipsoidShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mEllipsoidShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAddon->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void EllipsoidShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAddon->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
double smallestComponent(const Eigen::Vector3d& v)
{
  return std::min( v[0], std::min( v[1], v[2] ) );
}

//==============================================================================
void EllipsoidShapeNode::extractData(bool firstTime)
{
  if(   mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_TRANSFORM)
     || mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime )
  {
    Eigen::Matrix4d S(Eigen::Matrix4d::Zero());
    const Eigen::Vector3d& s =
       mEllipsoidShape->getSize()/smallestComponent(mEllipsoidShape->getSize());
    S(0,0) = s[0]; S(1,1) = s[1]; S(2,2) = s[2]; S(3,3) = 1.0;
    setMatrix(eigToOsgMatrix(S));
  }

  if(nullptr == mGeode)
  {
    mGeode = new EllipsoidShapeGeode(mEllipsoidShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
EllipsoidShapeNode::~EllipsoidShapeNode()
{
  // Do nothing
}

//==============================================================================
EllipsoidShapeGeode::EllipsoidShapeGeode(dart::dynamics::EllipsoidShape* shape,
                                         ShapeFrameNode* parentShapeFrame,
                                         EllipsoidShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mParentNode(parentNode),
    mEllipsoidShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
  extractData();
}

//==============================================================================
void EllipsoidShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void EllipsoidShapeGeode::extractData()
{
  if(nullptr == mDrawable)
  {
    mDrawable = new EllipsoidShapeDrawable(mEllipsoidShape, mVisualAddon, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
EllipsoidShapeGeode::~EllipsoidShapeGeode()
{
  // Do nothing
}

//==============================================================================
EllipsoidShapeDrawable::EllipsoidShapeDrawable(
    dart::dynamics::EllipsoidShape* shape,
    dart::dynamics::VisualAddon* visualAddon,
    EllipsoidShapeGeode* parent)
  : mEllipsoidShape(shape),
    mVisualAddon(visualAddon),
    mParent(parent)
{
  refresh(true);
}

//==============================================================================
void EllipsoidShapeDrawable::refresh(bool firstTime)
{
  if(mEllipsoidShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(osg::Object::STATIC);
  else
    setDataVariance(osg::Object::DYNAMIC);

  if(mEllipsoidShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    osg::ref_ptr<osg::Sphere> osg_shape = nullptr;
    osg_shape = new osg::Sphere(osg::Vec3(0,0,0),
                          0.5*smallestComponent(mEllipsoidShape->getSize()));

    setShape(osg_shape);
    dirtyDisplayList();
  }

  if(mEllipsoidShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    setColor(eigToOsgVec4(mVisualAddon->getRGBA()));
  }
}

//==============================================================================
EllipsoidShapeDrawable::~EllipsoidShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osgDart
