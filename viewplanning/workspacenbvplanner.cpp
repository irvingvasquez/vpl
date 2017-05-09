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


#include "workspacenbvplanner.h"

WorkspaceNBVPlanner::WorkspaceNBVPlanner(RobotSensor* rs, PartialModelBase* pm): NBVPlanner(rs, pm)
{

}


bool WorkspaceNBVPlanner::planNBV(ViewStructure& v)
{
//  std::list<ViewStructure> evaluated_views;
  
  //vsSaveViewList(pointedViews, viewSphereFileName);
  //partialModel->readCandidateViews(viewSphereFileName);
  partialModel->evaluateCandidateViews(pointedViews);
  //partialModel->saveEvaluations();
  //partialModel->sortCandidateViews();
  //partialModel->saveEvaluatedViews(evaluatedViewsFile);
  
  //vsReadViewList(evaluated_views, evaluatedViewsFile);
  
  pointedViews.sortHighToLow();
  
 // if(evaluated_views.size() > 0){
    v = bestViewOfList(pointedViews);
    std::cout << v << std::endl;
 //   v = NBV;
    return true;
//  }
  
//  return false;
}


bool WorkspaceNBVPlanner::init()
{
  NBVPlanner::init();
  
  std::cout << "\n----------------- WorkspaceNBVPlanner Configuration ------------ " << std::endl;
  
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("plannerConfig.ini");
  
  mrpt::utils::CConfigFile parser;
  ASSERT_FILE_EXISTS_(config_file);
  parser.setFileName(config_file);
  
  double x1,y1,z1,x2,y2,z2;
  partialModel->getOBB(x1,y1,z1,x2,y2,z2);
  
  objectCenter.clear();
  objectCenter.resize(3);
  
  objectCenter[0] = (x2+x1)/2;
  objectCenter[1] = (y2+y1)/2;
  objectCenter[2] = (z2+z1)/2;
  std::cout << "View sphere center: [" << objectCenter[0] << ", " << objectCenter[1] << ", " << objectCenter[2] << "]" << std:: endl;
  
  radio = parser.read_double("workSpacePlanner", "radius", 1);
  std::cout << "Sphere points radious: " << radio << std::endl;
  
  viewSphereFileName.clear();
  viewSphereFileName = dataFolder + "/view_sphere.vs";
  
  evaluatedViewsFile.clear();
  evaluatedViewsFile.assign( parser.read_string("workSpacePlanner", "evaluated_views", "evaluated_views.vs"));
  evaluatedViewsFile = dataFolder + "/" + evaluatedViewsFile;
  
  int tesselation_level = parser.read_int("workSpacePlanner", "tesselationLevel", 1);
  std::cout << "Tesselation level: " << tesselation_level << std::endl; 
  
  ViewSphereSynthesis generator(radio, objectCenter[0], objectCenter[1], objectCenter[2], tesselation_level);
  generator.getViews(pointedViews); 
  std::cout << pointedViews.size() << " views were generated" << std::endl;
  
  // This step is needed because the rays depend on the sensor pose with respect of the robot
  robotWithSensor->getViewsFromComfigurations(pointedViews);
  
  return true;
}


void WorkspaceNBVPlanner::generatePointedConfigurations(std::list< std::vector< double > >& configurations, std::string points_file, std::vector< double > object_center, double radio)
{
  std::vector< std::vector<double> > points;
  std::vector< std::vector<double> >::iterator point_it;
  
  vpFileReader reader;
  reader.readDoubleCoordinates(points_file, points);
  
  //viewList.clear();
  configurations.clear();
  /// unit sphere point
  std::vector<double> usp(3); 
  
  /// view sphere point
  std::vector<double> vsp(3);
  
  std::vector<double> pointing_v(3);
  
  std::vector<double> config(6);
  
  std::vector<double> coordinates(6);
  
  double yaw, pitch, roll;
  double norm;
  //BoostMatrix Robot(4,4);
  
  //mrpt::math::CMatrixDouble44 htm_p;
  
  for(point_it = points.begin(); point_it != points.end(); point_it++){
    if(point_it->size() != 3){
      std::cout << "Error in points " << std::endl;
      exit(0);
    }
    
    usp[0] = (*point_it)[0];
    usp[1] = (*point_it)[1];
    usp[2] = (*point_it)[2];
    
    // expander el punto al radio indicado; 
    usp[0] = radio * usp[0];
    usp[1] = radio * usp[1];
    usp[2] = radio * usp[2];
        
    // trasladar el punto con respecto al centro del objeto
    if(object_center.size() != 3){
      std::cout << "Error in object point " << std::endl;
      exit(0);
    }
    vsp[0] = object_center[0] + usp[0];
    vsp[1] = object_center[1] + usp[1];
    vsp[2] = object_center[2] + usp[2];
    
    // calcular el vector que apunta al objeto
    pointing_v[0] = object_center[0] - vsp[0];
    pointing_v[1] = object_center[1] - vsp[1];
    pointing_v[2] = object_center[2] - vsp[2];
    
    // calcular los angulos de rotación
    yaw = atan2(pointing_v[1], pointing_v[0]);
    
    norm = sqrt( pow(pointing_v[0],2) + pow(pointing_v[1],2) + pow(pointing_v[2],2) );
    //norm = radio;
    pitch = asin( pointing_v[2] / norm);
    pitch = -pitch; // I did this because the positive angles lie before the x-y plane.  
    roll = 0;
   
    // determinar la configuración
    config[0] = (double) vsp[0];
    config[1] = (double) vsp[1];
    config[2] = (double) vsp[2];
    config[3] = (double) (yaw);
    config[4] = (double) (pitch);
    config[5] = (double) (roll);
    
    configurations.push_back(config);
  }
}


/*
void WorkspaceNBVPlanner::generatePointedViews(list< ViewStructure >& viewList, std::string points_file, std::vector< double > object_center, double radio)
{
  std::vector< std::vector<double> > points;
  std::vector< std::vector<double> >::iterator point_it;
  
  vpFileReader reader;
  reader.readDoubleCoordinates(points_file, points);
  
  viewList.clear();
  /// unit sphere point
  std::vector<double> usp(3); 
  
  /// view sphere point
  std::vector<double> vsp(3);
  
  std::vector<double> pointing_v(3);
  
  std::vector<double> config(6);
  
  std::vector<double> coordinates(6);
  
  double yaw, pitch, roll;
  double norm;
  BoostMatrix Robot(4,4);
  
  mrpt::math::CMatrixDouble44 htm_p;
  
  for(point_it = points.begin(); point_it != points.end(); point_it++){
    if(point_it->size() != 3){
      std::cout << "Error in points " << std::endl;
      exit(0);
    }
    
    usp[0] = (*point_it)[0];
    usp[1] = (*point_it)[1];
    usp[2] = (*point_it)[2];
    
    // expander el punto al radio indicado; 
    usp[0] = radio * usp[0];
    usp[1] = radio * usp[1];
    usp[2] = radio * usp[2];
        
    // trasladar el punto con respecto al centro del objeto
    if(object_center.size() != 3){
      std::cout << "Error in object point " << std::endl;
      exit(0);
    }
    vsp[0] = object_center[0] + usp[0];
    vsp[1] = object_center[1] + usp[1];
    vsp[2] = object_center[2] + usp[2];
    
    // calcular el vector que apunta al objeto
    pointing_v[0] = object_center[0] - vsp[0];
    pointing_v[1] = object_center[1] - vsp[1];
    pointing_v[2] = object_center[2] - vsp[2];
    
    // calcular los angulos de rotación
    yaw = atan2(pointing_v[1], pointing_v[0]);
    
    norm = sqrt( pow(pointing_v[0],2) + pow(pointing_v[1],2) + pow(pointing_v[2],2) );
    //norm = radio;
    pitch = asin( pointing_v[2] / norm);
    pitch = -pitch; // I did this because the positive angles lie before the x-y plane.  
    roll = 0;
   
    // determinar la configuración
    config[0] = (double) (vsp[0] * 1000);
    config[1] = (double) (vsp[1] * 1000);
    config[2] = (double) (vsp[2] * 1000);
    config[3] = (double) (yaw * 180/M_PI);
    config[4] = (double) (pitch * 180/M_PI);
    config[5] = (double) (roll * 180/M_PI);
    
    coordinates[0] = vsp[0];
    coordinates[1] = vsp[1];
    coordinates[2] = vsp[2];
    coordinates[3] = yaw;
    coordinates[4] = pitch;
    coordinates[5] = roll;
    
    
    // determinar la matriz de transformación homogenea
    mrpt::poses::CPose3D pose(vsp[0], vsp[1], vsp[2], yaw, pitch, roll);
    pose.getHomogeneousMatrix( htm_p);
  
    Robot(0,0) = htm_p(0,0);
    Robot(0,1) = htm_p(0,1); 
    Robot(0,2) = htm_p(0,2); 
    Robot(0,3) = htm_p(0,3);
    
    Robot(1,0) = htm_p(1,0); 
    Robot(1,1) = htm_p(1,1); 
    Robot(1,2) = htm_p(1,2);
    Robot(1,3) = htm_p(1,3);
    
    Robot(2,0) = htm_p(2,0);
    Robot(2,1) = htm_p(2,1);
    Robot(2,2) = htm_p(2,2);
    Robot(2,3) = htm_p(2,3);
    
    Robot(3,0) = htm_p(3,0);
    Robot(3,1) = htm_p(3,1);
    Robot(3,2) = htm_p(3,2);
    Robot(3,3) = htm_p(3,3);
    
    //cout << Robot << std::endl;
    
    // llenar la vista
    ViewStructure v;
    v.q = config;
    
    v.w = coordinates;
    
    viewList.push_back(v);
  }
}
*/
