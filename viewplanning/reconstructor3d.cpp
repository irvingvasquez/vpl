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


#include "reconstructor3d.h"

Reconstructor3D::Reconstructor3D(RobotSensor* rs, NBVPlanner* p)
{
  initiated = false;
  n_views = 0;
  robot_sensor = rs;
  planner = p;
  waitForUser = true;
  waitForUser = true;
  
  std_accumulated_dist = 0;
  std_dist  = 0;
  std_distance_utility  = 0;
  std_motionplanning_t  = 0;
  std_occ_voxel  = 0;
  std_reconstruction_percent = 0;
  std_positioning_t = 0;
  std_probability = 0;
  std_registration_t = 0;
  std_surface_utility = 0;
  std_total_time = 0;
  std_unk_voxel = 0;
  std_update_t = 0;
  std_utility = 0;
  std_vision_t = 0;
}


bool Reconstructor3D::init()
{
  std::cout << "Starting Next Best View Problem." << std::endl;
    
  std::cout << "Reading initial position." << std::endl;
  std::string fileName;
  fileName.assign(configFolder);
  fileName.append("/p0.vs");
  std::cout << fileName.c_str() << std::endl;
  
  iteration = 0;
  
  if (!vsReadViewList(Plan, fileName)){
    std::cout << "Error: No initial view" << std::endl;
    getchar();
    return false;
  }
  
  initiated = true;
  
  dataFolder.assign(dataFolder);
  configFolder.assign(configFolder);
  //folderName.assign(folder);
  
  scanFileName.clear();
  scanFileName.append(dataFolder);
  scanFileName.append("/");
  //scanFileName.append(problemName);
  scanFileName.append("scan");
  
  originFileName.clear();
  originFileName.append(dataFolder);
  originFileName.append("/");
  //originFileName.append(problemName);
  originFileName.append("scan_origin");
  
  candidateViewsFileName.clear();
  candidateViewsFileName.append(dataFolder);
  candidateViewsFileName.append("/");
  //candidateViewsFileName.append(problemName);
  candidateViewsFileName.append("random_views.vs");
  
  raysFileName.clear();
  raysFileName.append(dataFolder);
  raysFileName.append("/");
  raysFileName.append("rays_0.dat");
  
  //TODO: read from file
  //goalsFileName.assign("/home/irving/projects/OctreeNBVPlanner/build/evaluated_views_mm.vs");
  goalsFileName.clear();
  goalsFileName.append(dataFolder);
  goalsFileName.append("/");
  goalsFileName.append("goals.dat");
  
  reachedGoalsFilename.clear();
  reachedGoalsFilename.append(dataFolder);
  reachedGoalsFilename.append("/reachedGoals.dat");
  
  partialModelFileName.clear();
  partialModelFileName.append(dataFolder);
  partialModelFileName.append("/");
  partialModelFileName.append("partial_model_octree_color.ot");
  
  
  
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("plannerConfig.ini");
  
  dictionary * ini_file;
    
  ini_file = iniparser_load(config_file.c_str());
  if (ini_file ==NULL ) {
      fprintf(stderr, "cannot parse file: %s\n", config_file.c_str());
      return false ;
  }
  //iniparser_dump(ini_file, stderr);
  
  waitForUser = iniparser_getboolean(ini_file, "Reconstructor3D:waitForUser", true);
  updateRobotLocalization = iniparser_getboolean(ini_file, "Reconstructor3D:updateRobotLocalization", false);
  maxIterations = iniparser_getint(ini_file, "Reconstructor3D:maxIterations", 10);
    
  std::cout << "---------- Reconstructor 3D -------" << std::endl;
  std::cout << "Wait for user:" << waitForUser << std::endl;
  std::cout << "Max iterations(stop criteria)" << maxIterations << std::endl;
  std::cout << "-----------------------------------	" << std::endl;
  
  double x,y,z,yaw,pitch,roll;
  
  
  x = iniparser_getdouble(ini_file, "RobotPose:x",0);
  y = iniparser_getdouble(ini_file, "RobotPose:y",0);
  z  = iniparser_getdouble(ini_file, "RobotPose:z",0);
  yaw  = iniparser_getdouble(ini_file, "RobotPose:yaw",0);
  pitch  = iniparser_getdouble(ini_file, "RobotPose:pitch",0);
  roll  = iniparser_getdouble(ini_file, "RobotPose:roll",0);
  mrpt::poses::CPose3D T(x,y,z,yaw,pitch,roll);
  
  robot_sensor->updateRobotLocalization(T);
  
  startLogFile();
  return true;
}


void Reconstructor3D::solveReconstruction()
{
  std::cout << "Solving Reconstruction." << std::endl;
  std::string scan_i_file_name;
  std::string origin_i_file_name;
  //int iteration=0;
  
  if(!initiated)
    return;
  
  ViewStructure pi = Plan.front();
  std::vector<double> q = pi.q;
  robot_sensor->getViewFromState(pi, q);
  char a;
  
  std::vector<double> qi;
  robot_sensor->getCurrentConfiguration(qi);
  std::cout << "Init configuration:" << std::endl;
  printVector(qi); 
  
  do{  
      std::cout << std::endl << std::endl <<"----------------------- View Planning Iteration " << iteration << "----------------" << std::endl;
      
      //if(waitForUser || iteration == 0)
      if(waitForUser)
	if(!vpl::userContinues()){
	  exit(0);
      }
	
      std::cout << "------------------------------- Positioning ---------------" << std::endl;
     
      if(!this->positioning(pi)){
	//waitForUser = true;
      }
      
      currentView = pi;
      scan_i_file_name = scanFileName;
      origin_i_file_name = originFileName;
      
      if(waitForUser)
	if(!vpl::userContinues()){
	  exit(0);
      }
      
      std::cout << "-------------------------------- Scanning ------------------------" << std::endl;
      this->takeScan(scan_i_file_name, origin_i_file_name);
      std::cout << origin_i_file_name.c_str() << std::endl;
      std::cout << scan_i_file_name.c_str() << std::endl;
      
     // if(waitForUser)
     //	if(!vpl::userContinues()){
     //	  exit(0);
     // }
     
      std::cout << "----------------------------- Model update ----------------------" << std::endl;
      this->updateModel(scan_i_file_name, origin_i_file_name);
      
      if(waitForUser)
	if(!vpl::userContinues()){
	  exit(0);
      }
	
      std::cout << "----------------------- Planing Next Best View ------------------" << std::endl;
      pi = this->PlanNextBestView();
      Plan.push_back(pi);
      
      saveToLogFile();
      iteration ++;
      saveData();
  } while(!stopCriteriaSatisfied());
  
  std::cout << "Reconstruction finished" << std::endl;
  this->finishReconstruction();
  
}


void Reconstructor3D::finishReconstruction()
{
  //partialModel->paintOccupiedInCapsule();
  //partialModel->paintOccluded();
  //partialModel->savePartialModel(partialModelFileName);
  //string command("/home/irving/projects/octomap-1.4/bin/octovis ");
//   command.append(partialModelFileName);
//   command.append(" &");
//   
  planner->savePartialModel(partialModelFileName);
  
  planner->finishPlanning();

  std::string plan_file;
  plan_file.clear();
  plan_file.assign(dataFolder);
  plan_file.append("/plan.dat");
  vsSaveViewList(Plan, plan_file);
  
  //string comando;
  //comando.assign("/home/irving/projects/octomap-distribution/bin/octovis ");
  //comando.append("object_pts.dat");
  //comando.append(" &");
  //std::system(comando.c_str());
  //system( /home/irving/projects/Reconstructor3D/build/TestPioneerKinect/partial_model_octree_color.ot &");
  //system("/home/irving/projects/octomap-1.4/bin/octovis /home/irving/projects/Reconstructor3D/build/TestPioneerKinect/scan_1.dat &");
//   sleep(2);
  
  std::cout << "Model saved, bye" << std::endl;
}


bool Reconstructor3D::stopCriteriaSatisfied()
{
  char a;
  
  if(waitForUser){
      
    std::cout << "Continue reconstruction? [y/n]:";
    std::cin >> a;
    if (a == 'n')
      return true;
    else
      return false;
  } else {
    if(this->iteration > maxIterations)
      return true;
  }
  return false;
}


bool Reconstructor3D::takeScan(std::string &scan_name, std::string &origin_name)
{
  std::cout << "Reconstructor: take scan" << std::endl;
  robot_sensor->performScan();
  std::cout << "Reconstructor: save scan" << std::endl;
  robot_sensor->saveLastPointCloudMts(scan_name);
  robot_sensor->saveSensorTrajectoryMts(origin_name);
  
  return true;
}


bool Reconstructor3D::updateModel(std::string scan_name, std::string origin_name)
{  
  std_update_t = planner->updateWithPoinCloud(scan_name, origin_name);
  
  return planner->savePartialModel(partialModelFileName);
}


ViewStructure Reconstructor3D::PlanNextBestView()
{
  ViewStructure v;

  if(!planner->planNBV(v)){
    std::cout << "WARNING: NBV was not found!." << std::endl;
    waitForUser = true;
  }
  
  std_accumulated_dist = planner->std_accu_distance;
  std_dist = planner->std_distance;
  std_distance_utility = planner->std_distance_uf;
  std_motionplanning_t = planner->std_mp_time;
  std_surface_utility = planner->std_surface_uf;
  std_utility = planner->std_utility;
  std_vision_t = planner->std_v_time;
  
  return v;
}


void Reconstructor3D::setNGeneratedViews(int n)
{
  n_views = n;
}


void Reconstructor3D::setDataFolder(std::string folder)
{
  dataFolder = folder;
}

void Reconstructor3D::setConfigFolder(std::string folder)
{
 configFolder = folder;
}



void Reconstructor3D::saveData()
{
  std::ostringstream s;
  s << iteration;
  std::string nuevo_dir = dataFolder;
  nuevo_dir.append("/");
  nuevo_dir.append(s.str());
  
  std::string command;
  
  command.assign("mkdir ");
  command.append(nuevo_dir); 
  std::cout << "command: " <<  command<< std::endl;
  std::system(command.c_str());
  
  command.clear();
  command.assign("cp ");
  command.append(dataFolder);
  command.append("/* ");
  command.append(nuevo_dir);
  std::cout << command << std::endl;
  std::system(command.c_str());
  
  command.clear();
  command.assign("rm ");
  command.append(dataFolder);
  command.append("/scan*");
  std::cout << "command: " <<  command<< std::endl;
  std::system(command.c_str());
}


bool Reconstructor3D::startLogFile()
{
  std::ofstream file;
  std::string file_name(dataFolder);
  file_name.append("/planning.log");
  bool append = false;
  
  if(append){
    file.open(file_name.c_str(),std::ios_base::app);
  }else {
    file.open(file_name.c_str());
  }
  
  if(file.is_open()){
    file << "Iter\t" << "uf\t" << "s_uf\t" << "d_uf\t" << "unk_vxl\t" 
    << "occu_vxl\t" << "per_rec\t" << "dist\t" << "acc_dist\t" 
    << "updt_t\t" << "reg_t\t" << "vis_t\t" << "mp_t\t" 
    << "pos_t\t" << "total_t\t" << "unk_vol" << std::endl ;  
    file.close();
    return true;
  } else {
    std::cout << "Unable to open file " << file_name.c_str() << std::endl;
    return false;
  }
}


void Reconstructor3D::setPartialModel(PartialModelBase* pm)
{
  partialModel = pm;
}


bool Reconstructor3D::saveToLogFile()
{
  std::ofstream file;
  std::string file_name(dataFolder);
  file_name.append("/planning.log");
  bool append = true;
  
  if(append){
    file.open(file_name.c_str(),std::ios_base::app);
  }else {
    file.open(file_name.c_str());
  }
  
  if(file.is_open()){
    //file << "Iter\t" << "uf\t" << "s_uf\t" 
	//    << "d_uf" << "unk_vxl\t" << "occu_vxl\t" << "per_rec\t" 
	// << "dist\t" << "acc_dist\t" << "updt_t\t" << "reg_t\t" << "vis_t\t" << "mp_t\t" << "pos_t\t" << "total_t\t" << std::endl ;  
  
    file << iteration << "\t" 
	  << std_utility << "\t" 
	  << std_surface_utility << "\t" 
	  << std_distance_utility << "\t"
	  << std_unk_voxel << "\t"
	  << std_occ_voxel << "\t"
	  << std_reconstruction_percent << "\t"
	  << std_dist << "\t"
	  << std_accumulated_dist << "\t"
	  << std_update_t << "\t"
	  << std_registration_t << "\t"
	  << std_vision_t << "\t" 
	  << std_motionplanning_t << "\t"
	  << std_positioning_t << "\t"
	  << std_total_time << "\t"
	  << std_unknown_volume << std::endl ;  
    file.close();
    return true;
  } else {
    std::cout << "Unable to open file " << file_name.c_str() << std::endl;
    return false;
  }
}

