/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

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


#ifndef RECONSTRUCTOR3D_H
#define RECONSTRUCTOR3D_H

#include <viewstructure.h>

#include <list>
#include <string>
#include <sstream>

#include "robotsensor.h"
#include "nbvplanner.h"
#include "vplutils.h"



/**
 * Solves a Reconstruction Problem
 * At the indicated directory must be the following files:
 * 	-p0.vs	Initial view in ViewStructure Format
 */
class Reconstructor3D
{

public:
Reconstructor3D(RobotSensor *rs, NBVPlanner *p);

/**
 * Initialices the NBV problem
 */

virtual bool init();

virtual bool positioning(ViewStructure v)=0;

virtual void saveData();

/** 
 * Performs a scan and returns the name of the scan file and the origin file
 */
virtual bool takeScan(std::string &scan_name, std::string &origin_name);

virtual ViewStructure PlanNextBestView();

virtual bool updateModel(std::string scan_name, std::string origin_name);

virtual bool stopCriteriaSatisfied();

virtual void finishReconstruction();

void solveReconstruction();

void setNGeneratedViews(int n);

void setDataFolder(std::string folder);

void setConfigFolder(std::string folder);

  bool startLogFile();
  bool saveToLogFile();

  void setPartialModel(PartialModelBase *pm);
  
protected:
  bool waitForUser;
  bool updateRobotLocalization;
  int maxIterations;
  
  PartialModelBase *partialModel;
  RobotSensor *robot_sensor;
  NBVPlanner *planner;
  
  std::string dataFolder;
  std::string configFolder;

  std::string scanFileName;
  std::string originFileName;

  std::string candidateViewsFileName;
  std::string raysFileName;
  
  std::string goalsFileName;
  std::string reachedGoalsFilename;
  std::string partialModelFileName;
  std::string occupiedVoxelsFileName;
  std::string unknownVoxelsFilename;
  
  std::string currentPointCloudFile;
  std::string currentOriginFile;
  
  long int n_views;
  
  int iteration;
  
  /// config to reach NBV
  std::vector< std::vector<int> > configs;

  int n_subset_to_motion;
  
   // A robot with a sensor
  
 /**
  * Reconstruction Plan (series of views)
  * P in paper
  */
 std::list<ViewStructure> Plan;
 
 /**
  * Current View
  */
 ViewStructure currentView;
 
 //std::list<PathStructureQ> Paths;
 
 bool initiated;
 
 /// stadistical variables
 float std_utility;
 float std_surface_utility;
 float std_distance_utility;
 float std_unk_voxel;
 float std_occ_voxel;
 float std_reconstruction_percent;
 float std_dist;
 float std_accumulated_dist;
 float std_update_t;
 float std_registration_t;
 float std_vision_t;
 float std_motionplanning_t;
 float std_positioning_t;
 float std_total_time;
 double std_unknown_volume;
 
 float std_probability;
 
};

#endif // RECONSTRUCTOR3D_H
