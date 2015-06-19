/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
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

#include <iostream>
#include <vector>
#include <thread>
#include <future>

#include "dart/dart.h"

#include "apps/bipedStand/MyWindow.h"

const bool runTest = true;
const bool multiThreaded = true;

const size_t numIterationsPerThread = 5000;

typedef std::vector< std::vector<dart::collision::Contact> > CollisionData;

void singleThreadedTest(const std::vector<dart::simulation::WorldPtr>& worlds, size_t numClones)
{
  size_t singleForceCount = 0;
  size_t inconsistentForceCount = 0;
  size_t iterations = 0;

  while(runTest)
  {
    ++iterations;
    bool inconsistentForce = false;
    std::unordered_map<dart::dynamics::BodyNode*, size_t> forceTally;

    std::vector<dart::collision::Contact> contacts;

    for(size_t i=0; i<worlds.size(); ++i)
    {
      const dart::simulation::WorldPtr& world = worlds[i];
      world->step();

      dart::collision::CollisionDetector* cd =
          world->getConstraintSolver()->getCollisionDetector();
      size_t colCount = cd->getNumContacts();

      for(size_t c=0; c < colCount; ++c)
      {
        if(0 == i)
        {
          const dart::collision::Contact& contact = cd->getContact(c);
          contacts.push_back(contact);

          auto it = forceTally.find(contact.bodyNode1.lock());
          if(it == forceTally.end())
            forceTally[contact.bodyNode1.lock()] = 1;
          else
            ++it->second;

          it = forceTally.find(contact.bodyNode2.lock());
          if(it == forceTally.end())
            forceTally[contact.bodyNode2.lock()] = 1;
          else
            ++it->second;
        }
        else if(!inconsistentForce)
        {
          const dart::collision::Contact& contact = cd->getContact(c);
          const dart::collision::Contact& originalContact = contacts[c];

          if(contact.bodyNode1.lock()->getName() != originalContact.bodyNode1.lock()->getName() ||
             contact.bodyNode2.lock()->getName() != originalContact.bodyNode2.lock()->getName() )
          {
            std::cout << "section 1 violation" << std::endl;
            inconsistentForce = true;
          }

          if(contact.force != originalContact.force ||
             contact.normal != originalContact.normal ||
             contact.penetrationDepth != originalContact.penetrationDepth ||
             contact.point != originalContact.point)
          {
            std::cout << "section 2 violation" << std::endl;
            inconsistentForce = true;
          }

          if(i < numClones && // only check this for clones
             (contact.shape1 != originalContact.shape1 ||
              contact.shape2 != originalContact.shape2) )
          {
            std::cout << "section 3 violation" << std::endl;
            inconsistentForce = true;
          }
        }
      } // for contacts
    } // for world

    if(inconsistentForce)
      ++inconsistentForceCount;

    bool singleForcePerBody = true;
    for(const auto& tally : forceTally)
    {
      if(tally.second > 1)
        singleForcePerBody = false;
    }

    if(singleForcePerBody)
      ++singleForceCount;

    if(inconsistentForce || iterations%1000 == 0)
    {
      std::cout << "inconsistent force count: " << inconsistentForceCount
                << "\t\tsingle force count: " << singleForceCount << std::endl;
    }
  } // while true
}

CollisionData runThread(const dart::simulation::WorldPtr& world, size_t index)
{
  CollisionData myData;
  myData.resize(numIterationsPerThread);

  for(size_t i=0; i<numIterationsPerThread; ++i)
  {
    std::vector<dart::collision::Contact>& contacts = myData[i];

    world->step();

    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    size_t colCount = cd->getNumContacts();

    contacts.reserve(colCount);
    for(size_t j=0; j<colCount; ++j)
    {
      contacts.push_back(cd->getContact(j));
    }
  }

  return myData;
}

void multiThreadedTest(const std::vector<dart::simulation::WorldPtr>& worlds,
                       size_t numClones)
{
  size_t singleForceCount = 0;
  size_t inconsistentForceCount = 0;

  while(true)
  {
    bool inconsistentForce = false;

    std::vector< std::future<CollisionData> > futures;

    for(size_t i=0; i<worlds.size(); ++i)
    {
      futures.push_back(std::async(std::launch::async, &runThread, worlds[i], i));
    }

    for(size_t i=0; i<futures.size(); ++i)
      futures[i].wait();

    std::vector<CollisionData> allData;
    allData.reserve(futures.size());
    for(auto& future : futures)
      allData.push_back(future.get());

    for(size_t i=1; i<worlds.size(); ++i)
    {
      const CollisionData& originalData = allData[0];
      const CollisionData& worldData = allData[i];

      if(originalData.size() != worldData.size())
      {
        std::cout << "Mismatch in number of iterations ( " << worldData.size()
                  << " : " << originalData.size() << "). This is a bug!! These "
                  << "values should always be " << numIterationsPerThread
                  << std::endl;
        inconsistentForce = true;
        break;
      }

      for(size_t j=0; j<originalData.size(); ++j)
      {
        const std::vector<dart::collision::Contact>& originalContacts =
            originalData[j];
        const std::vector<dart::collision::Contact>& contacts = worldData[j];

        if(originalContacts.size() != contacts.size())
        {
          std::cout << "difference in number of contacts (" << i << ":" << j
                    << ")" << std::endl;
          inconsistentForce = true;
          break;
        }

        for(size_t k=0; k<originalContacts.size(); ++k)
        {
          const dart::collision::Contact& originalContact = originalContacts[k];
          const dart::collision::Contact& contact = contacts[k];

          if(contact.bodyNode1.lock()->getName() != originalContact.bodyNode1.lock()->getName() ||
             contact.bodyNode2.lock()->getName() != originalContact.bodyNode2.lock()->getName())
          {
            std::cout << "section 1 violation" << std::endl;
            inconsistentForce = true;
          }

          if(contact.force != originalContact.force ||
             contact.normal != originalContact.normal ||
             contact.penetrationDepth != originalContact.penetrationDepth ||
             contact.point != originalContact.point)
          {
            std::cout << "section 2 violation" << std::endl;
            inconsistentForce = true;
          }

          if(i < numClones && // only check this for clones
             (contact.shape1 != originalContact.shape1 ||
              contact.shape2 != originalContact.shape2) )
          {
            std::cout << "section 3 violation" << std::endl;
            inconsistentForce = true;
          }
        } // for contacts
      } // for time step
    } // for world

    if(inconsistentForce)
      ++inconsistentForceCount;

    const CollisionData& originalData = allData[0];
    for(size_t i=0; i<originalData.size(); ++i)
    {
      const std::vector<dart::collision::Contact>& originalContacts =
          originalData[i];

      std::unordered_map<dart::dynamics::BodyNode*, size_t> forceTally;

      for(size_t j=0; j<originalContacts.size(); ++j)
      {
        const dart::collision::Contact& contact = originalContacts[j];

        auto it = forceTally.find(contact.bodyNode1.lock());
        if(it == forceTally.end())
          forceTally[contact.bodyNode1.lock()] = 1;
        else
          ++it->second;

        it = forceTally.find(contact.bodyNode2.lock());
        if(it == forceTally.end())
          forceTally[contact.bodyNode2.lock()] = 1;
        else
          ++it->second;
      }

      bool singleForcePerBody = true;
      for(const auto& tally : forceTally)
      {
        if(tally.second > 1)
          singleForcePerBody = false;
      }

      if(singleForcePerBody)
        ++singleForceCount;
    }

    std::cout << "inconsistent force count: " << inconsistentForceCount
              << "\t\tsingle force count: " << singleForceCount << std::endl;

  } // while true
}

int main(int argc, char* argv[]) {

  // create and initialize the world
  dart::simulation::WorldPtr myWorld
      = dart::utils::SkelParser::readWorld(
          DART_DATA_PATH"skel/fullbody1.skel");
  assert(myWorld != nullptr);

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  // create controller
  Controller* myController = new Controller(myWorld->getSkeleton(1),
                                            myWorld->getTimeStep());
  dart::dynamics::MetaSkeletonPtr group = myController->getSkel();

  std::vector<size_t> genCoordIds;
  genCoordIds.push_back(1);
  genCoordIds.push_back(6);   // left hip
  genCoordIds.push_back(14);  // left knee
  genCoordIds.push_back(17);  // left ankle
  genCoordIds.push_back(9);   // right hip
  genCoordIds.push_back(15);  // right knee
  genCoordIds.push_back(19);  // right ankle
  genCoordIds.push_back(13);  // lower back
  Eigen::VectorXd initConfig(8);
  initConfig << -0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;
  group->setPositions(genCoordIds, initConfig);

  myController->resetDesiredDofs();

  const dart::dynamics::SkeletonPtr& skel = myWorld->getSkeleton(1);
  for(size_t i=1; i<skel->getNumJoints(); ++i)
  {
    skel->getJoint(i)->setActuatorType(dart::dynamics::Joint::VELOCITY);
  }

  std::vector<dart::simulation::WorldPtr> worlds;

  // Using cloning
  worlds.push_back(myWorld);
  worlds.push_back(myWorld->clone());
  worlds.push_back(myWorld->clone());
  size_t numClones = worlds.size();

  // Using reloading
//  worlds.push_back(dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/fullbody1.skel"));
//  worlds.back()->setGravity(gravity);
//  worlds.push_back(dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/fullbody1.skel"));
//  worlds.back()->setGravity(gravity);

  for(size_t i=1; i<worlds.size(); ++i)
  {
    const dart::simulation::WorldPtr& world = worlds[i];
    for(size_t j=0; j<world->getNumSkeletons(); ++j)
    {
      const dart::dynamics::SkeletonPtr& cloneSkel = world->getSkeleton(j);
      const dart::dynamics::SkeletonPtr& originalSkel = worlds[0]->getSkeleton(j);

      cloneSkel->setPositions(originalSkel->getPositions());
      cloneSkel->setVelocities(originalSkel->getVelocities());
      cloneSkel->setAccelerations(originalSkel->getAccelerations());
      cloneSkel->setForces(originalSkel->getForces());

      // Note: this for loop is only needed for the worlds that were reloaded
      // from the file instead of being cloned
      for(size_t k=0; k<cloneSkel->getNumJoints(); ++k)
      {
        cloneSkel->getJoint(k)->setActuatorType(
              originalSkel->getJoint(k)->getActuatorType());
      }
    }
  }

  if(runTest)
  {
    if(multiThreaded)
      multiThreadedTest(worlds, numClones);
    else
      singleThreadedTest(worlds, numClones);
  }

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);
//  window.setController(myController);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'1'--'4': programmed interaction" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Balance");
  glutMainLoop();

  return 0;
}

