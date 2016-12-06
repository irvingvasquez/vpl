/*
    Copyright 2013 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef VPRFREEFLYER_H
#define VPRFREEFLYER_H

#include "vprobot.h"

#include <mrpt/poses.h>
#include <viewstructure.h>
#include <vpfilereader.h>

/**
 * A configuration is especified by
 * x, y, z, yaw z, pitch y, roll x
 * all robots work with mm and rads
 */

class vprFreeFlyer : public vpRobot
{

private:
  std::vector<double> currentConfig;
  
public:
vprFreeFlyer();
  
virtual void getPoseFromConfiguration(mrpt::poses::CPose3D &pose, std::vector<double> q);

virtual void getCurrentHTM(BoostMatrix& HTM);

virtual void getHTMfromConfiguration(BoostMatrix& HTM, std::vector< double, std::allocator< double > > q);

virtual bool moveToConfiguration(std::vector< double, std::allocator< double > > configuration);

virtual float executeTrajectory(std::vector< std::vector< double > > controls, double delta_t, std::vector< double > goal_q);

virtual float executeMovement();

virtual void getCurrentConfiguration(std::vector< double, std::allocator< double > >& q);

virtual bool setVelocities(std::vector< double > velocities);

virtual bool init();

 /**
   * Generates a set of views which point to a object
   * @param radio in mts
  */
  void generatePointedViews(std::list< ViewStructure > &viewList, std::string points_file, std::vector< double > object_center, double radio);
  
virtual void updateRobotLocalization(mrpt::poses::CPose3D transformation);
  
};

#endif // VPRFREEFLYER_H
