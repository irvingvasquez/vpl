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


#ifndef NBVPLANNER_H
#define NBVPLANNER_H

#include "robotsensor.h"
#include <partialmodels.h>
#include <views.h>
// #include <model.h>

class NBVPlanner
{

public:
NBVPlanner(RobotSensor *rs, PartialModelBase *pm);
  virtual bool init();
  
  void getSolutionControls(std::vector< std::vector<double> > &controls, double &delta_t);
  
  void getSolutionPath(std::vector< std::vector<double> > &path);
  
  void setConfigFolder(std::string folder);
  
  void setDataFolder(std::string folder);
  
  /**
   * The planner updates its internal representation for example octree
   */
  virtual float updateWithPoinCloud(std::string pc_file, std::string origin_file);
  
  virtual bool planNBV(ViewStructure &v)=0;
  
  virtual bool savePartialModel(std::string file_name);
  
  void finishPlanning();
  
  bool savePlannerData();
  
//   double accumulatedDistance(list<MSLVector> path, Model *m);
  
  //bool saveFrames();
  
  /// stadistical variables
  float std_v_time;
  float std_mp_time;
  float std_distance;
  float std_accu_distance;
  
  float std_utility;
  float std_surface_uf;
  float std_distance_uf;
  
  
protected:
  RobotSensor *robotWithSensor;
  PartialModelBase *partialModel;
  
  ViewStructure NBV;
  std::vector< std::vector<double> > solutionControls;
  std::vector< std::vector<double> > solutionPath;
  double plannerDeltaT;
  long int rrtNodes;
  
  std::string configFolder;
  std::string dataFolder;
  
  /// stadistical variables
  std::vector<float> visionTimes;
  std::vector<float> motionPTimes;
  std::vector<double> distances_per_it;
  
};

#endif // NBVPLANNER_H
