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


#ifndef WORKSPACENBVPLANNER_H
#define WORKSPACENBVPLANNER_H

#include "nbvplanner.h"
#include "viewsynthesis.h"

/**
 * This planner generates a view sphere and selects the NBV as the view with the highest evaluation
*/
class WorkspaceNBVPlanner : public NBVPlanner
{
public:
WorkspaceNBVPlanner(RobotSensor* rs, PartialModelBase* pm);
  
virtual bool planNBV(ViewStructure &v);

virtual bool init();

protected:
  std::string viewSphereFileName;
  std::string evaluatedViewsFile;
  
  ViewList pointedViews;
  
  std::vector<double> objectCenter;
  
  // radio of the view sphere
  double radio;
  
  
  /**
   * only works in workspace. And for freeflyer robot.
   */
  void generatePointedConfigurations(std::list <vector <double> > &configurations, std::string points_file, std::vector<double> object_center, double radio);
};

#endif // WORKSPACENBVPLANNER_H
