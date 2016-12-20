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


#include "robotmobile.h"

void RobotMobile::getPoseFromConfiguration(mrpt::poses::CPose3D& pose, std::vector< double > q)
{
  pose.setFromValues(q[0],q[1],0,q[2],0,0);
}

void RobotMobile::getHTMfromConfiguration(BoostMatrix& HTM, std::vector< double, std::allocator< double > > q)
{
  float x,y,theta;
  x = (float) q[0];
  y = (float) q[1];
  theta = (float) (q[2]);
  
  BoostMatrix Robot(4,4);
  
  Robot(0,0) = cos(theta);
  Robot(0,1) = -sin(theta); 
  Robot(0,2) = 0; 
  
  Robot(1,0) =  sin(theta); 
  Robot(1,1) =  cos(theta); 
  Robot(1,2) = 0;
  
  Robot(2,0) = 0;
  Robot(2,1) = 0;
  Robot(2,2) = 1;
  
  Robot(3,0) = 0.0;
  Robot(3,1) = 0.0;
  Robot(3,2) = 0.0;
  Robot(3,3) = 1;
  
  Robot(0,3) = x;
  Robot(1,3) = y;
  Robot(2,3) = 0;
  
  HTM = Robot;
}

// bool RobotMobile::moveToConfiguration(std::vector< double, std::allocator< double > > configuration)
// {
//   currentConfig = configuration;
//   return true;
// }


// bool RobotMobile::executeTrajectory(std::vector< std::vector< double > > controls, double delta_t, std::vector< double > goal_q)
// {
//   currentConfig = goal_q;
//   return true;
// }

// bool RobotMobile::setVelocities(std::vector< double > velocities)
// {
//   return true;
// }


bool RobotMobile::init()
{
  vpRobot::init();
  
  configurationDim = 3;
  inputDim = 2;
  info.assign("Name: Mobile Robot");
  
  return true;
}

RobotMobile::RobotMobile()
{

}



RobotMobileDummie::RobotMobileDummie()
{

}


float RobotMobileDummie::executeTrajectory(std::vector< std::vector< double > > controls, double delta_t, std::vector< double > goal_q)
{
  currentConfig = goal_q;
  return true;
}

float RobotMobileDummie::executeMovement()
{
  currentConfig = goalConfig;
  return true;
}

bool RobotMobileDummie::setVelocities(std::vector< double > velocities)
{
  return true;
}

void RobotMobileDummie::updateRobotLocalization(mrpt::poses::CPose3D transformation)
{

}


void RobotMobileDummie::getCurrentConfiguration(std::vector< double >& q)
{
  q = currentConfig;
}

bool RobotMobileDummie::moveToConfiguration(std::vector< double > configuration)
{
  currentConfig = configuration;
  return true;
}
