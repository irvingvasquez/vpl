/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ROBOTCOMPOSED_H
#define ROBOTCOMPOSED_H

#include <vprobot.h>
#include <pmutils.h>

#include <mrpt/utils.h>
#include <mrpt/math.h>
#include <mrpt/system.h>
#include <mrpt/random.h>

#include "model3drigideva.h"
#include <geomPQP.h>
#include <problem.h>

#include "nbvs_utils.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;

class RobotComposed : public vpRobot
{

public:	

/**
  * Robots must be already initialized
  */
void addRobot(mrpt::poses::CPose3D pose, vpRobot* robot);

virtual void getCurrentConfiguration(std::vector< double >& q);
  
virtual void getPoseFromConfiguration(mrpt::poses::CPose3D& pose, std::vector< double, std::allocator< double > > q);

virtual void getHTMfromConfiguration(BoostMatrix& HTM, std::vector< double, std::allocator< double > > q);

virtual bool moveToConfiguration(std::vector< double, std::allocator< double > > configuration);

virtual float executeTrajectory(std::vector< std::vector< double > > controls, double delta_t, std::vector< double > goal_q);

virtual float executeMovement();

virtual bool setVelocities(std::vector< double > velocities);

virtual bool init();

virtual void updateRobotLocalization(mrpt::poses::CPose3D transformation);

RobotComposed();

protected:
  std::vector<vpRobot*> robots;
  std::vector<mrpt::poses::CPose3D> robotPoses;
  
  int numberOfRobots;
};


class RobotComposedEVA : public RobotComposed
{
public:
 virtual float executeMovement();
  
 virtual void updateRobotLocalization(mrpt::poses::CPose3D transformation);
 
};


/**
 * Introduces a random error in the applied velocities
 */
class RobotComposedEVAError : public RobotComposedEVA
{
  virtual float executeMovement();
  
  ///virtual void getCurrentConfiguration(std::vector< double >& q);
  
//   virtual bool moveToConfiguration(std::vector< double > configuration);
  
  virtual bool init();
  
protected:
  std::vector <double> Sigmas;
  
  std::vector <double> Means;

  Model * m;
  Geom *g;
  
//   MSLVector currentState;
//   MSLVector initialState;
//   std::vector<double> currentConfiguration;
//   std::vector<double> initialConfiguration;
};

#endif // ROBOTCOMPOSED_H
