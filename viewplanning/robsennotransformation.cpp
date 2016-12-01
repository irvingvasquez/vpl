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


#include "robsennotransformation.h"

RobSenNoTransformation::RobSenNoTransformation(vpRobot* r, RangeSensor* s): RobotSensor(r, s)
{
  
}


long int RobSenNoTransformation::performScan()
{
//return RobotSensor::performScan();
  std::vector< mrpt::poses::CPoint3D > points;
  std::vector< mrpt::poses::CPoint3D > origins;
  //std::vector< mrpt::poses::CPoint3D > converted_points;
  std::vector< mrpt::poses::CPoint3D > converted_origins;
  
  std::vector<double> q;
  ViewStructure v_crrnt;
  this->getCurrentConfiguration(q);
  this->getViewFromState(v_crrnt, q);
  
  sensor->setCurrentView(v_crrnt);
  sensor->getPoints(points);
  std::cout << "Robot sensor: obtained points from sensor: " << points.size() << std::endl;
  //convertPointsToRobotReference(points, converted_points);
   
  origins.clear();
  origins.push_back( *(new mrpt::poses::CPoint3D(0,0,0)) );
  convertPointsToRobotReference(origins, converted_origins);
  std::cout << "Converted origin: " << converted_origins[0] << std::endl;
  addPointsToPointCloud(points, converted_origins);
  
  std::cout << "Robot sensor: scan performed" << std::endl;
  
  return points.size();
}
