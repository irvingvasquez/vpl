/*
 * 
 * 
View Planning Library
Copyright (c) 2016, J. Irving Vasquez ivasquez@ccc.inaoep.mx
Consejo Nacional de Ciencia y Tecnología (CONACYT)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



#ifndef NBVPLANNER_H
#define NBVPLANNER_H

#include "robotsensor.h"
#include <partialmodelbase.h>
#include <viewstructure.h>

/**
 * 
 * Planner base class
 * 
 */
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
  
  /**
   * Main function, it computes the NBV
   */
  virtual bool planNBV(ViewStructure &v)=0;
  
  virtual bool savePartialModel(std::string file_name);
  
  void finishPlanning();
  
  bool savePlannerData();
  
  // double accumulatedDistance(list<MSLVector> path, Model *m);
  
  // bool saveFrames();
  
  /// statistical variables
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
  
  /// statistical variables
  std::vector<float> visionTimes;
  std::vector<float> motionPTimes;
  std::vector<double> distances_per_it;
  
};

#endif // NBVPLANNER_H
