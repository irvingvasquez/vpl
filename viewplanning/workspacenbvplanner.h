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


#ifndef WORKSPACENBVPLANNER_H
#define WORKSPACENBVPLANNER_H

#include "nbvplanner.h"
#include "viewsynthesis.h"

/**
 * Planner that computes the NBV as a pose (x,y,z,yaw,pitch,roll)
 * In particular, this implemented object generates a view sphere and selects the NBV as the view with the highest evaluation depending on the partial model selected
 * 
 * At initialization:
 * 	It generates a view sphere assuming a freeflyer robot which is pointing to the x axis and z is upward
 * 	The view sphere is taken as configurations of the robot, then It asks to the robot to return the end effector pose and HTM
 *      The list of candidateViews is created
 * 
 * For NBV computation:
 * 	It queries to the partial model to evaluate the pointed views
 * 
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
  
  ViewList candidateViews;
  
  std::vector<double> objectCenter;
  
  // radio of the view sphere
  double radio;
  
  /**
   * only works in workspace. And for freeflyer robot.
   */
  //void generatePointedConfigurations(std::list <vector <double> > &configurations, std::string points_file, std::vector<double> object_center, double radio);
};

#endif // WORKSPACENBVPLANNER_H
