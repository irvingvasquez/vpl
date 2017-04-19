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


#ifndef RRTNBV_H
#define RRTNBV_H

#include <rrt.h>
#include <partialmodelbase.h>
#include "robotsensor.h"
#include <viewstructure.h>
#include <list> //rem


class RRTNBV : public RRT
{

protected:

  PartialModelBase *partialModel;
  RobotSensor *robotWithSensor;

  MSLVector nbv_x;
  ViewStructure nbv_v;
  bool foundNBV;

  MSLVector Interval;
  std::vector<double> Weights;

  virtual bool EvaluateCandidate(MSLNode *node, MSLNode *parent);

  double weightedDistance(const MSLVector& x1, const MSLVector& x2);

  virtual bool Extend(const MSLVector& x, MSLTree* t, MSLNode*& nn, bool forward, bool &new_nbv);

  std::string configFolder;
  std::string dataFolder;
  
public:
  virtual bool Plan();
  void setPartialModel(PartialModelBase *pm);
  void setRobotWithSensor(RobotSensor *rs);
  void setConfigFolder(std::string folder);
  void setDataFolder(std::string folder);
  ViewStructure getNBV();

  RRTNBV(Problem* problem);
  RRTNBV(Problem* problem, std::string config_folder, std::string data_folder);
  
  std::list< list<double> > t_evaluations; // rem
  std::string tree_file;
};


#endif // RRTNBV_H
