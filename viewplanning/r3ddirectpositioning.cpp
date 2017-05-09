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


#include "r3ddirectpositioning.h"

R3DDirectPositioning::R3DDirectPositioning(RobotSensor* rs, NBVPlanner* p): Reconstructor3D(rs, p)
{
    reconstruction_perc_fn.assign("reconstruction_percentage");
}


bool R3DDirectPositioning::init()
{
  Reconstructor3D::init();
  
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("plannerConfig.ini");
  
  mrpt::utils::CConfigFile parser;
  ASSERT_FILE_EXISTS_(config_file);
  parser.setFileName(config_file);
  
  gap_points = parser.read_double("Reconstructor3D", "gap", 0.005);
    
  std::cout << "\n---------- Direct positioning reconstructor 3D -------" << std::endl;
  std::cout << "Gap: " << gap_points << std::endl; 
  std::cout << "-----------------------------------" << std::endl;
}


bool R3DDirectPositioning::positioning(ViewStructure v)
{
    //return Reconstructor3D::positioning(v);
    std::cout << "Moving to: "<< std::endl; 
    std::cout << v;
    std_positioning_t = 0.0;
    
    return robot_sensor->moveToConfiguration(v.q);
}


bool R3DDirectPositioning::updateModel(std::string scan_name, std::string origin_name)
{
  std_update_t = planner->updateWithPoinCloud(scan_name, origin_name);
  
  planner->savePartialModel(partialModelFileName);
  
  // compare percentage of reconstruction
  std::string target(dataFolder);
  target.append("/");
  target.append(tar_pts_fn);
  
  double percentage;
  percentage = PMUtils::compareFilePoints(target, ref_obj_pts_fn, gap_points);

  std::string rpfn(dataFolder);
  rpfn.append("/");
  rpfn.append(reconstruction_perc_fn);
  vpFileReader fm;
  fm.saveData2Text<double>(percentage, rpfn, true);
  
  std_reconstruction_percent = (float) percentage;
  //cout << "rrr" << std_reconstruction_percent << std::endl;
  
  std_unknown_volume = partialModel->getUnknownVolume();
  std::cout << "unknown volume " << std_unknown_volume << std::endl;
  
  return true;
}







Reconstructor3DRobot::Reconstructor3DRobot(RobotSensor* rs, NBVPlanner* p): Reconstructor3D(rs, p)
{

}


bool Reconstructor3DRobot::positioning(ViewStructure v)
{
  std::vector< std::vector<double> > controls;
  std::vector< std::vector<double> > path;
  double delta_t;
  
  std::cout << "Moving to: "<< std::endl; 
  std::cout << v;
  
  if(iteration == 0){
    robot_sensor->moveToConfiguration(v.q);
  } else {
    planner->getSolutionControls(controls, delta_t);
    planner->getSolutionPath(path);
    //cout << "paso 1" << std::endl;
    robot_sensor->setPathToExecute(path);
    robot_sensor->setTrajectory(controls, delta_t);
    robot_sensor->setGoalConfiguration(v.q);
    //cout << "paso 2" << std::endl; 
    if(std_positioning_t = robot_sensor->executeMovement()==-1){
      //fallido
      return false;
    } else {
      std::vector<double> reached_state;
      ViewStructure reached_view;
      robot_sensor->getCurrentConfiguration(reached_state);
      robot_sensor->getViewFromState(reached_view, reached_state);
      std::cout << "Reached configuration:" << std::endl;
      //cout << reached_view;
      
      // Calculate error
      this->partialModel->evaluateView(reached_view);
      float cov_error = reached_view.n_unknown - v.n_unknown;
      std::cout << "Coverage error:" << cov_error;
      float abs_cov_error = (float) abs(reached_view.n_unknown - v.n_unknown);
      std::cout << "\tCoverage error Abs:" << abs_cov_error;
      float ov_plann = (float) v.n_occupied / ((float)v.n_occupied + (float)v.n_unknown);
      std::cout << "\t" << ov_plann;
      float ov_reach = 0;
      if((reached_view.n_occupied + reached_view.n_unknown) != 0){
       ov_reach = (float) reached_view.n_occupied / ((float)reached_view.n_occupied + (float)reached_view.n_unknown);
      }
      std::cout << "\t" << ov_reach;
      float overlap_error = abs(ov_plann - ov_reach);
      std::cout << " \tOverlap error:" << overlap_error << std::endl;
      
      std::cout << reached_view;
      
      //save information
      vpFileReader file_manager;
      file_manager.saveData2Text<float>(cov_error, dataFolder + "/CoverageError", true, '\t');
      file_manager.saveData2Text<float>(abs_cov_error, dataFolder + "/CoverageAbsError", true, '\t');
      file_manager.saveData2Text<float>(overlap_error, dataFolder + "/OverlapError", true, '\t');
    }
  }
}


bool Reconstructor3DRobot::takeScan(std::string& scan_name, std::string& origin_name)
{
  Reconstructor3D::takeScan(scan_name, origin_name);
  
  //TODO remove when registration will be done
  vpFileReader reader;
  std::vector< std::vector<double> > points;
  std::string obj_name = scan_name;
  
  reader.readDoubleCoordinates(scan_name, points);
  obj_name.append(".wrl");
  PMUtils::saveCoordinatesAsVRML(points, obj_name);
  //reader.saveDoubleCoordinatesAsOBJ(obj_name, points);
}


bool Reconstructor3DRobot::updateModel(std::string scan_name, std::string origin_name)
{
  //return Reconstructor3D::updateModel(scan_name, origin_name);
  
  // read this parameters and put them on dat2pcs command
  float dist_seg_base = 1.0;
  float dist_seg_iteration = 0.8;
  
  
  sleep(3);
  std::string ext(".dat");
  size_t start_pos;
  
  std::string scan_pcd_name(scan_name);
  start_pos = scan_pcd_name.find(ext);
  scan_pcd_name.replace(start_pos, ext.length(), ".pcd");
  
  std::string origin_pcd_name(origin_name);
  start_pos = origin_pcd_name.find(ext);
  origin_pcd_name.replace(start_pos, ext.length(), ".pcd");
  
  std::string scan_seg_pcd_name(scan_name);
  start_pos = scan_seg_pcd_name.find(ext);
  scan_seg_pcd_name.replace(start_pos, ext.length(), "_seg.pcd");
  
  std::string scan_reg_pcd_name(scan_name);
  start_pos = scan_reg_pcd_name.find(ext);
  scan_reg_pcd_name.replace(start_pos, ext.length(), "_reg.pcd");
  
  std::string origin_reg_pcd_name(origin_name);
  start_pos = origin_reg_pcd_name.find(ext);
  origin_reg_pcd_name.replace(start_pos, ext.length(), "_reg.pcd");
  
  std::string scan_reg_dat_name(scan_name);
  start_pos = scan_reg_dat_name.find(ext);
  scan_reg_dat_name.replace(start_pos, ext.length(), "_reg.dat");
  
  std::string origin_reg_dat_name(origin_name);
  start_pos = origin_reg_dat_name.find(ext);
  origin_reg_dat_name.replace(start_pos, ext.length(), "_reg.dat");
  
  
  std::string line_command;
  std::string object_pts_name("/home/irving/projects/nbvPlanning-1.1/test/EVA_Kat_horizontal/data/registered_points.pcd");
  
  // save as PCD
  //cout << "Executing commands: " << iteration << std::endl;
  
  if(iteration == 0){
    // TODO Object Segmentation
    // convert to pcd
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/dat2pcd_segment_obj ");
    line_command.append(scan_name);
    line_command.append(" ");
    line_command.append(scan_pcd_name);
    line_command.append(" ");
    line_command.append(scan_seg_pcd_name);
    line_command.append(" 1.0");
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    
    /// convert to PCD the initial map
    std::string initial_map_dat(configFolder);
    initial_map_dat.append("/registrationInitialMap.dat");
    std::string initial_map_pcd(dataFolder);
    initial_map_pcd.append("/registrationInitialMap.pcd");
    
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/dat2pcd ");
    line_command.append(initial_map_dat);
    line_command.append(" ");
    line_command.append(initial_map_pcd);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    
    /// Register the initial map
    // Registration with ICP non linear
    line_command.clear();
    if(waitForUser){
      line_command.append("/home/irving/projects/TestRegistration/build/scan_registration ");
    }else {
      line_command.append("/home/irving/projects/TestRegistration/build/continuos_scan_registration ");
    }
    line_command.append(initial_map_pcd);
    line_command.append(" ");
    line_command.append(scan_seg_pcd_name);
    //line_command.append("/registrationInitialMap.dat");
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    if(waitForUser){
      std::cout << "3DR Is registration sucessful?" << std::endl;
      if (!vpl::userContinues())
	return false;
    }
    
    /// replace accumulated points
    line_command.clear();
    line_command.append("cp registered_1.pcd ");
    line_command.append(object_pts_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    // transform the original full scan
    line_command.clear();
    line_command.append("/home/irving/projects/TestRegistration/build/transform_pcd ");
    line_command.append(scan_pcd_name);
    line_command.append(" transformation.dat ");
    line_command.append(scan_reg_pcd_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    // convert to dat
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/pcd2dat ");
    line_command.append(scan_reg_pcd_name);
    line_command.append(" ");
    line_command.append(scan_reg_dat_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    
    //TODO transform origin of scan
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/dat2pcd ");
    line_command.append(origin_name);
    line_command.append(" ");
    line_command.append(origin_pcd_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    // transform the origin
    line_command.clear();
    line_command.append("/home/irving/projects/TestRegistration/build/transform_pcd ");
    line_command.append(origin_pcd_name);
    line_command.append(" transformation.dat ");
    line_command.append(origin_reg_pcd_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
     // convert to dat
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/pcd2dat ");
    line_command.append(origin_reg_pcd_name);
    line_command.append(" ");
    line_command.append(origin_reg_dat_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
  }
  else {
    
    // TODO Object Segmentation
    // convert to pcd
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/dat2pcd_segment_obj ");
    line_command.append(scan_name);
    line_command.append(" ");
    line_command.append(scan_pcd_name);
    line_command.append(" ");
    line_command.append(scan_seg_pcd_name);
    line_command.append(" 1.0");
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    //std::system("/home/irving/projects/PCLProg/build/dat2pcd /home/irving/projects/nbvPlanning-1.1/test/EVA_Kat_horizontal/data/current_scan.dat /home/irving/projects/nbvPlanning-1.1/test/EVA_Kat_horizontal/data/current_scan.pcd 1.0");
    
    // Registration with ICP non linear
    line_command.clear();
    if(waitForUser){
      line_command.append("/home/irving/projects/TestRegistration/build/scan_registration ");
    }else {
      line_command.append("/home/irving/projects/TestRegistration/build/continuos_scan_registration ");
    }
    line_command.append(object_pts_name);
    line_command.append(" ");
    line_command.append(scan_seg_pcd_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    if(waitForUser){
      std::cout << "3DR Is registration sucessful?" << std::endl;
      if (!vpl::userContinues())
	return false;
    }
    
    // replace accumulated points
    line_command.clear();
    line_command.append("cp registered_1.pcd ");
    line_command.append(object_pts_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    
    // transform the original full scan
    line_command.clear();
    line_command.append("/home/irving/projects/TestRegistration/build/transform_pcd ");
    line_command.append(scan_pcd_name);
    line_command.append(" transformation.dat ");
    line_command.append(scan_reg_pcd_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    
    // convert to dat
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/pcd2dat ");
    line_command.append(scan_reg_pcd_name);
    line_command.append(" ");
    line_command.append(scan_reg_dat_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    
    //TODO transform origin of scan
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/dat2pcd ");
    line_command.append(origin_name);
    line_command.append(" ");
    line_command.append(origin_pcd_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    // transform the origin
    line_command.clear();
    line_command.append("/home/irving/projects/TestRegistration/build/transform_pcd ");
    line_command.append(origin_pcd_name);
    line_command.append(" transformation.dat ");
    line_command.append(origin_reg_pcd_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    // convert to dat
    line_command.clear();
    line_command.append("/home/irving/projects/PCLProg/build/pcd2dat ");
    line_command.append(origin_reg_pcd_name);
    line_command.append(" ");
    line_command.append(origin_reg_dat_name);
    std::cout << line_command << std::endl; 
    std::system(line_command.c_str());
    
    /// Transform the position of the robot given the computed registration matrix
    if(updateRobotLocalization){
      mrpt::poses::CPose3D pose;
      vpl::readTransformationMatrix("transformation.dat", pose);
      robot_sensor->updateRobotLocalization(pose);
    }
  }
    
  //planner->updateWithPoinCloud(scan_name, origin_name);
  
  std_update_t = planner->updateWithPoinCloud(scan_reg_dat_name, origin_reg_dat_name);
  
  planner->savePartialModel(partialModelFileName);
  
  std_unknown_volume = partialModel->getUnknownVolume();
  std::cout << "unknown volume " << std_unknown_volume << std::endl;
  
  std::string comando;
  comando.assign("/home/irving/projects/octomap-distribution/bin/octovis ");
  comando.append(partialModelFileName);
  comando.append(".unk.ot &");
  std::system(comando.c_str());
  
  return true;
}


bool Reconstructor3DExpUtility::positioning(ViewStructure v)
{
  //eturn R3DDirectPositioning::positioning(v);
  std::vector< std::vector<double> > controls;
  std::vector< std::vector<double> > path;
  double delta_t;
  
  std::cout << "Moving to: "<< std::endl; 
  std::cout << v;
  
  if(iteration == 0){
    robot_sensor->moveToConfiguration(v.q);
  } else {
    planner->getSolutionControls(controls, delta_t);
    planner->getSolutionPath(path);
    //cout << "paso 1" << std::endl;
    robot_sensor->setPathToExecute(path);
    robot_sensor->setTrajectory(controls, delta_t);
    robot_sensor->setGoalConfiguration(v.q);
    //cout << "paso 2" << std::endl; 
    if(std_positioning_t = robot_sensor->executeMovement()==-1){
      //fallido
      return false;
    } else {
      std::vector<double> reached_state;
      ViewStructure reached_view;
      robot_sensor->getCurrentConfiguration(reached_state);
      robot_sensor->getViewFromState(reached_view, reached_state);
      std::cout << "Reached configuration:" << std::endl;
      //cout << reached_view;
      
      // Calculate error
      this->partialModel->evaluateView(reached_view);
      float cov_error = reached_view.n_unknown - v.n_unknown;
      std::cout << "Coverage error:" << cov_error;
      float abs_cov_error = (float) abs(reached_view.n_unknown - v.n_unknown);
      std::cout << "\tCoverage error Abs:" << abs_cov_error;
      float ov_plann = (float) v.n_occupied / ((float)v.n_occupied + (float)v.n_unknown);
      std::cout << "\t" << ov_plann;
      float ov_reach = 0;
      if((reached_view.n_occupied + reached_view.n_unknown) != 0){
       ov_reach = (float) reached_view.n_occupied / ((float)reached_view.n_occupied + (float)reached_view.n_unknown);
      }
      std::cout << "\t" << ov_reach;
      float overlap_error = abs(ov_plann - ov_reach);
      std::cout << " \tOverlap error:" << overlap_error << std::endl;
      
      std::cout << reached_view;
      
      //save information
      vpFileReader file_manager;
      file_manager.saveData2Text<float>(cov_error, dataFolder + "/CoverageError", true, '\t');
      file_manager.saveData2Text<float>(abs_cov_error, dataFolder + "/CoverageAbsError", true, '\t');
      file_manager.saveData2Text<float>(overlap_error, dataFolder + "/OverlapError", true, '\t');
    }
  }
  
  return true;
}


Reconstructor3DExpUtility::Reconstructor3DExpUtility(RobotSensor* rs, NBVPlanner* p): R3DDirectPositioning(rs, p)
{

}


/*
void Reconstructor3DRobot::segmentObject(std::string scan_name)
{
  // canceled
  
  vpFileReader reader;
  std::vector < std::vector <double> > points;
  std::vector < std::vector <double> >::iterator it;
  std::vector < double > pt;
  std::vector < std::vector <double> > points_segmented;
  reader.readDoubleCoordinates(scan_name, points);
  
  
  
  for(it = points.begin(); it != points.end(); it++){
    
  }
}*/
