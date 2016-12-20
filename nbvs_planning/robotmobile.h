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


#ifndef ROBOTMOBILE_H
#define ROBOTMOBILE_H

#include "vprobot.h"

class RobotMobile : public vpRobot
{
public:
virtual void getPoseFromConfiguration(mrpt::poses::CPose3D &pose, std::vector< double, std::allocator< double > > q);
virtual void getHTMfromConfiguration(BoostMatrix& HTM, std::vector< double, std::allocator< double > > q);
virtual bool init();
// virtual bool setVelocities(std::vector< double > velocities);
RobotMobile();
};



class RobotMobileDummie : public RobotMobile{
protected:
  std::vector<double> currentConfig;

public:
RobotMobileDummie();
  virtual void getCurrentConfiguration(std::vector< double >& q);

  virtual bool moveToConfiguration(std::vector< double > configuration);

  virtual float executeTrajectory(std::vector< std::vector< double > > controls, double delta_t, std::vector< double > goal_q);

  virtual float executeMovement();
  
  virtual bool setVelocities(std::vector< double > velocities);
  
  virtual void updateRobotLocalization(mrpt::poses::CPose3D transformation);
}; 



#endif // ROBOTMOBILE_H
