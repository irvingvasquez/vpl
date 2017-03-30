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
  
  dictionary * ini_file;
    
  ini_file = iniparser_load(config_file.c_str());
  if (ini_file ==NULL ) {
      fprintf(stderr, "cannot parse file: %s\n", config_file.c_str());
      exit(0);
      return false ;
  }
  
  plannerDeltaT = iniparser_getdouble(ini_file, "MotionPlanning:deltaT", 1.0);
  rrtNodes = iniparser_getint(ini_file, "MotionPlanning:NumNodes", 10000);
  
  std::cout << "---------- NBV Planner -------" << std::endl;
  std::cout << "Delta T: " << plannerDeltaT << std::endl;
  std::cout << "Number of Nodes: " << rrtNodes << std::endl;
  std::cout << "------------------------------" << std::endl;
  
  //plannerDeltaT = 1;
  //rrtNodes = 10000;
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
  
  
  file_name.clear();
  file_name = dataFolder + "/FrontierVertices.dat";
  std::string file_name2;
  file_name2 = dataFolder + "/FrontierNormals.dat";
  partialModel->saveFrontierUnknown(file_name, file_name2);
  return true;
}

