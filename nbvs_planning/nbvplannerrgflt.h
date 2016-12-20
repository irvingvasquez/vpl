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


#ifndef NBVPLANNERRGFLT_H
#define NBVPLANNERRGFLT_H

#include "nbvplanner.h"
#include "model3drigideva.h"
#include <geomPQP.h>
#include <problem.h>
#include <rrt.h>

#include "nbvs_utils.h"
#include "rrtnbv.h"


/**
 * Tries to reach all the subset of goals.
 */
class NBVPlannerRGFlt : public NBVPlanner
{
protected:
  bool readCandidateViews;
  // wheter the view file is readed only once
  bool readOnce; 
  bool alreadyReadedOnce;
  
  // Flag to delete the views that see zero unknown voxels
  bool deleteUnfeasibleUnknown;
  
  std::string viewsFile;
  
  MSLVector Interval;
  std::vector<double> Weights;
  
  int nViews;
  ViewList pointingViews;
  int max_i;
  
  // Robot motion model.
  Model *m;
  
  // Geometry description.
  // Geom *g;
  
  /**
   * Copies the parameters to the class parameters of solution when 2 trees were gronw
   */
  void copySolution(list<MSLVector> path, list<MSLVector> policy1, list<MSLVector> policy2);
  
  void copySingleSolution(list<MSLVector> path, list<MSLVector> policy1);
  
  double weightedDistance(const MSLVector& x1, const MSLVector& x2);
  
  double accumulatedWeightedDist(list<MSLVector> &path);
  
  /**
   * Lee un conjunto de vistas
   * En una versión futura podría leer las vistas cada vez que se llama a esta función
   */
  bool readViews(ViewList &views, Model *model, Geom *geometry);
 
public:
NBVPlannerRGFlt(RobotSensor* rs, PartialModelBase* pm);
~NBVPlannerRGFlt();

/**
 * Tries to reach all the subset of goals.
 */
virtual bool planNBV(ViewStructure& v);

virtual bool generateCandidateViews(ViewList &views, Model *model, Geom *geometry);

virtual bool init();
};



/**
 * Selects only one goal for the RRT
 */
class NBVPlannerRGFltOneGoal : public NBVPlannerRGFlt
{
public:
NBVPlannerRGFltOneGoal(RobotSensor* rs, PartialModelBase* pm);


  /**
   * Selects only one goal for the RRT
   */
  virtual bool planNBV(ViewStructure& v);
};



class NBVPlannerExtendTree : public NBVPlannerRGFlt
{
  public:
NBVPlannerExtendTree(RobotSensor* rs, PartialModelBase* pm);
  
  /**
   * Extends the tree and evaluates the nodes
   */
  virtual bool planNBV(ViewStructure& v);

//  virtual void copySolution(list< MSLVector > path, list< MSLVector > policy1, list< MSLVector > policy2);
};


#endif // NBVPLANNERRGFLT_H
