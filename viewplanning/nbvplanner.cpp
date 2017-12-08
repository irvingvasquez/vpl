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



#include "nbvplanner.h"


NBVPlanner::NBVPlanner(RobotSensor* rs, PartialModelBase* pm)
{
  robotWithSensor = rs;
  partialModel = pm;
  std_distance = 0.0;
  std_accu_distance = 0.0;
  std_mp_time = 0.0;
  std_v_time =0.0;
}

void NBVPlanner::setConfigFolder(std::string folder)
{
  configFolder = folder;
}

void NBVPlanner::setDataFolder(std::string folder)
{
  dataFolder = folder;
}

bool NBVPlanner::init()
{
  // Read some parameters
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("plannerConfig.ini");
  
  mrpt::utils::CConfigFile parser;
  ASSERT_FILE_EXISTS_(config_file);
  parser.setFileName(config_file);
  
  plannerDeltaT = parser.read_double("MotionPlanning", "deltaT", 1.0, true);
  rrtNodes = parser.read_int("MotionPlanning", "NumNodes", 10000, true);
  
  std::cout << "---------- NBV Planner -------" << std::endl;
  std::cout << "Delta T: " << plannerDeltaT << std::endl;
  std::cout << "Number of Nodes: " << rrtNodes << std::endl;
  std::cout << "------------------------------" << std::endl;
  
  return true;
}

void NBVPlanner::finishPlanning()
{
  //this->savePartialModel();
}


void NBVPlanner::getSolutionControls(std::vector< std::vector< double > >& controls, double& delta_t)
{
  controls = solutionControls;
  delta_t = plannerDeltaT;
}

void NBVPlanner::getSolutionPath(std::vector< std::vector< double > >& path)
{
  path = solutionPath;
}



// double NBVPlanner::accumulatedDistance(list< MSLVector > path, Model* m)
// {
//   double d = 0;
//   
//   list< MSLVector >::iterator a = path.begin();
//   list< MSLVector >::iterator b = path.begin();
//   b++;
//   while(b != path.end()){
//     d += m->Metric(*a, *b);
//     a = b;
//     b++;
//   }
//   
//   return d;
// }


float NBVPlanner::updateWithPoinCloud(std::string pc_file, std::string origin_file)
{
  return partialModel->updateWithScan(pc_file, origin_file);
}


bool NBVPlanner::savePartialModel(std::string file_name)
{
  partialModel->savePartialModel(file_name);
}


bool NBVPlanner::savePlannerData()
{
  std::string file_name;
  vpFileReader reader;
  
  /// Guardar datos del RRTNBV
  file_name.clear();
  file_name = configFolder + "/InitialState";
  std::vector<double> current_config;
  robotWithSensor->getCurrentConfiguration(current_config);
  reader.saveToMSLVector<double>(current_config, file_name);
  
  
  /// Save Obstacle information
  // Read environment 
  file_name.clear();
  file_name = configFolder + "/ObstEnvironment.raw";
  vpTriangleList triangles;
  triangles.readFile(file_name);
  
  vpTriangleList tris;
  partialModel->getOccupiedTriangles(tris);
  triangles.insert(triangles.end(), tris.begin(), tris.end());
  tris.clear();
  
  partialModel->getUnknownTriangles(tris);
  triangles.insert(triangles.end(), tris.begin(), tris.end());
  file_name.clear();
  file_name = configFolder + "/Obst";
  triangles.saveToMSLTriangle(file_name);
    
  file_name.clear();
  file_name = dataFolder + "/occupied.raw";
  partialModel->saveObjectAsRawT(file_name);
    
  file_name.clear();
  file_name = dataFolder + "/unknown.raw";
  partialModel->saveUnknownVolumeAsRawT(file_name); 
    
  // TODO
  //file_name.clear();
  //file_name = dataFolder + "/FrontierVertices.dat";
  //std::string file_name2;
  //file_name2 = dataFolder + "/FrontierNormals.dat";
  //partialModel->saveFrontierUnknown(file_name, file_name2);
  
  return true;
}

