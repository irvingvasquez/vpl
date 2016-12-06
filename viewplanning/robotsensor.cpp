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


#include "robotsensor.h"

RobotSensor::RobotSensor(vpRobot* r, RangeSensor* s)
{
  robotReady = false;
  sensorReady = false;
  pointCloudIdCounter = 0;
  
  robot = r;
  sensor = s;
  
  //TODO paths
}

long int RobotSensor::performScan()
{
  std::vector< mrpt::poses::CPoint3D > points;
  std::vector< mrpt::poses::CPoint3D > origins;
  std::vector< mrpt::poses::CPoint3D > converted_points;
  std::vector< mrpt::poses::CPoint3D > converted_origins;
  
  sensor->getPoints(points);
  //cout << "p: " << points.size() << std::endl;
  convertPointsToRobotReference(points, converted_points);
  
  origins.clear();
  origins.push_back( *(new mrpt::poses::CPoint3D(0,0,0)) );
  convertPointsToRobotReference(origins, converted_origins);
  //cout << "p: " << converted_points.size() << std::endl;
  addPointsToPointCloud(converted_points, converted_origins);
  return converted_points.size();
}


void RobotSensor::setSensor(RangeSensor* rs)
{
  sensor = rs;
}

void RobotSensor::setSensorPose(double x, double y, double z, double yaw, double pitch, double roll)
{
  SensorPose.setFromValues(x, y, z, yaw, pitch, roll);
}


void RobotSensor::addPointsToPointCloud(std::vector< mrpt::poses::CPoint3D >& points, std::vector< mrpt::poses::CPoint3D >& origins, bool clear)
{
  if(clear){
    pointCloudIdCounter++;
    pointCloud.clear();
    //EndEffectorTrajectory.clear();
    SensorTrajectory.clear();
  }
  
  pointCloud.insert(pointCloud.end(), points.begin(), points.end());
  
  SensorTrajectory.insert(SensorTrajectory.end(), origins.begin(), origins.end() );
}


void RobotSensor::getHTMfromRobot(BoostMatrix& htmRobot)
{
  robot->getCurrentHTM(htmRobot);
}

void RobotSensor::getHTMfromSensor(BoostMatrix& htmSensor)
{
  mrpt::math::CMatrixDouble44 SensorTMAux;
  BoostMatrix htmAux(4,4);
  
  /* Establish sensor pose */
  SensorPose.getHomogeneousMatrix(SensorTMAux);
  htmAux(0,0) = SensorTMAux(0,0);
  htmAux(0,1) = SensorTMAux(0,1);
  htmAux(0,2) = SensorTMAux(0,2);
  htmAux(0,3) = SensorTMAux(0,3);
  htmAux(1,0) = SensorTMAux(1,0);
  htmAux(1,1) = SensorTMAux(1,1);
  htmAux(1,2) = SensorTMAux(1,2);
  htmAux(1,3) = SensorTMAux(1,3);
  htmAux(2,0) = SensorTMAux(2,0);
  htmAux(2,1) = SensorTMAux(2,1);
  htmAux(2,2) = SensorTMAux(2,2);
  htmAux(2,3) = SensorTMAux(2,3);
  htmAux(3,0) = SensorTMAux(3,0);
  htmAux(3,1) = SensorTMAux(3,1);
  htmAux(3,2) = SensorTMAux(3,2);
  htmAux(3,3) = SensorTMAux(3,3);
  
  htmSensor = htmAux;
  //cout << "htmSensor: " << std::endl << htmSensor << std::endl;
}

void RobotSensor::convertPointsToRobotReference(std::vector< mrpt::poses::CPoint3D >& points, std::vector< mrpt::poses::CPoint3D >& result)
{
  ///
  BoostMatrix p(4,1);
  BoostMatrix p_robot(4,1);
  BoostMatrix R(4,4);
  BoostMatrix htmRobot;
  BoostMatrix htmSensor;
  this->getHTMfromRobot(htmRobot);
  std::cout << htmRobot << std::endl;
  this->getHTMfromSensor(htmSensor);
  std::cout << htmSensor << std::endl;
  R = boost::numeric::ublas::prod(htmRobot, htmSensor);
  
  mrpt::poses::CPoint3D point_robot;
    
  result.clear();
  p(3,0) = 1;
  for (std::vector<mrpt::poses::CPoint3D>::iterator it=points.begin(); it< points.end(); it++){ 
    p(0,0) = it->m_coords[0];
    p(1,0) = it->m_coords[1];
    p(2,0) = it->m_coords[2];
    
    p_robot = boost::numeric::ublas::prod(R, p);
    
    point_robot.m_coords[0] = p_robot(0,0);
    point_robot.m_coords[1] = p_robot(1,0);
    point_robot.m_coords[2] = p_robot(2,0);
    
    result.push_back(point_robot);
  }
}


bool RobotSensor::saveLastPointCloud(std::string& file_name, double factor)
{
  std::ofstream file;
  std::string aux = file_name;
  //string name;
  
  long int advance;
  int percentage = 0;
  long int saved_points = 0;
  double x, y, z;
  
  std::cout << "Saving last point cloud..." << std::endl;
  
  std::stringstream extension;
  extension << "_" << pointCloudIdCounter << ".dat";

  file_name += extension.str();
  
  advance = pointCloud.size() / 10;
  
  file.open(file_name.data());
  
  std::cout << pointCloud.size() << " points to be saved." << std::endl;
  
  if(file.is_open()){
    std::list<mrpt::poses::CPoint3D>::iterator it;
    
    for(it = pointCloud.begin(); it!= pointCloud.end(); it++){
      file << it->m_coords[0] * factor << "\t" << it->m_coords[1] *factor << "\t" << it->m_coords[2] * factor << "\n";
    }
    
    file.close();
  }
  else {
    std::cout << "Error: unable to save file. " << file_name.c_str() << std::endl;
    return false;
  }
  
  std::cout << "Last point cloud saved. " << file_name << "\n";
}


bool RobotSensor::saveLastPointCloudMts(std::string& file_name)
{
  return saveLastPointCloud(file_name, 1);
}


bool RobotSensor::saveSensorTrajectory(std::string& file_name, double factor)
{
  std::ofstream file;
 
  std::stringstream extension;
  extension << "_" <<  pointCloudIdCounter << ".dat";
  file_name += extension.str();
  
  file.open(file_name.c_str());
  
  std::cout << "Saving sensor trajectory. " << file_name.c_str() <<  std::endl;
  
  if(file.is_open()){
      std::list<mrpt::poses::CPoint3D>::iterator it;
      it = SensorTrajectory.begin();
    
      //file << "# " << file_name << "\n" << "# " << LaserTrajectory.size() << " saved points\n";
      for (uint i=0; i < SensorTrajectory.size(); i++){
	  file << it->m_coords[0] * factor << "\t" << it->m_coords[1] * factor << "\t" << it->m_coords[2] * factor << "\n";
	  it++;
      }
    
      file.close();
  }
    else {
      std::cout << "Error: unable to open file.\n";
      return false;
  }
  
  std::cout << "Saved sensor trajectory. " << file_name << "\n";
  return true;
}


bool RobotSensor::saveSensorTrajectoryMts(std::string& file_name)
{
  return saveSensorTrajectory(file_name, 1);
}


void RobotSensor::computeViewsFromConfigurations(std::list< ViewStructure >& views)
{
   std::list<ViewStructure>::iterator view_it;
  std::vector<int> configuration;
  boost::numeric::ublas::matrix<double> HTM;
  boost::numeric::ublas::matrix<double> htmRobot;
  boost::numeric::ublas::matrix<double> htmSensor;
  
  this->getHTMfromSensor(htmSensor);
  
  for(view_it = views.begin(); view_it!= views.end(); view_it++){
      //configuration = view_it->q;
      htmRobot = view_it->HTM;
      std::cout << htmRobot << std::endl;
      HTM = boost::numeric::ublas::prod(htmRobot, htmSensor);
      std::cout << HTM << std::endl;
      view_it->HTM = HTM;
  }
}


void RobotSensor::getViewsFromComfigurations(std::list< ViewStructure >& views, std::list< std::vector< double > > configurations)
{
  std::list< std::vector<double> >::iterator it_q;
  ViewStructure view;

  boost::numeric::ublas::matrix<double> HTM;
  boost::numeric::ublas::matrix<double> htmRobot;
  boost::numeric::ublas::matrix<double> htmSensor;
  
  views.clear();
  
  this->getHTMfromSensor(htmSensor);
  
  for(it_q = configurations.begin(); it_q!= configurations.end(); it_q++){
      robot->getHTMfromConfiguration(htmRobot, *it_q);
      //cout << htmRobot << std::endl;
      HTM = boost::numeric::ublas::prod(htmRobot, htmSensor);
      //cout << HTM << std::endl;
      view.q = *it_q;
      view.eval = 0;
      view.HTM = HTM;
      views.push_back(view);
  }
}


void RobotSensor::getViewFromState(ViewStructure& view, const std::vector< double > state)
{
  boost::numeric::ublas::matrix<double> HTM;
  boost::numeric::ublas::matrix<double> htmRobot;
  boost::numeric::ublas::matrix<double> htmSensor;
  
  this->getHTMfromSensor(htmSensor);
  
  robot->getHTMfromConfiguration(htmRobot, state);
//  std::cout << htmRobot << std::endl;
  
  HTM = boost::numeric::ublas::prod(htmRobot, htmSensor);
//  std::cout << HTM << std::endl;
  
  view.q = state;
  view.eval = 0;
  view.HTM = HTM;
  
  CMatrixDouble44 Mtx;
  vpl::convertBoost2Eigen(HTM, Mtx);
  mrpt::poses::CPose3D pose(Mtx);
  vpl::Pose2Vector(pose, view.w);
}


void RobotSensor::getRandomState(std::vector< double >& state)
{
  robot->getRandomState(state);
}




bool RobotSensor::init()
{

}

void RobotSensor::setConfigFolder(std::string folder)
{
  configFolder = folder;
}

void RobotSensor::setDataFolder(std::string folder)
{
  dataFolder = folder;
}

bool RobotSensor::moveToConfiguration(std::vector< double > configuration)
{
  return robot->moveToConfiguration(configuration);
}


float RobotSensor::executeMovement()
{
  return robot->executeMovement();
}


// bool RobotSensor::executeTrajectory(std::vector< std::vector< double > > controls, double delta_t, std::vector< double > goal_q)
// {
//   return robot->executeTrajectory(controls, delta_t, goal_q);
// }


void RobotSensor::getCurrentConfiguration(std::vector< double >& configuration)
{
  robot->getCurrentConfiguration(configuration);
}


bool RobotSensor::setGoalConfiguration(std::vector< double > goal_q)
{
  robot->goalConfig = goal_q;
}

bool RobotSensor::setPathToExecute(std::vector< std::vector< double > > path)
{
  robot->pathToExecute = path;
}

bool RobotSensor::setTrajectory(std::vector< std::vector< double > > trajectory, double delta_t)
{
  robot->trajectoryToExecute = trajectory;
  robot->deltaT = delta_t;
}


bool RobotSensor::updateRobotLocalization(mrpt::poses::CPose3D transformation)
{ 
  robot->updateRobotLocalization(transformation);
}

bool RobotSensor::autoOcclusion(std::vector< double > q)
{
  
}

bool RobotSensor::pointsToTheRobot(ViewStructure view, std::vector<double> q)
{
  /**
   * Esto debería hacerse tomando en cuenta la forma del robot
   * Sin embargo por tiempo de implementación solo se verifica si intersecta una esfera que representa el robot
   * 
   */
  
  double altura = 0.75;
  double radio = 0.25;
  
  mrpt::poses::CPoint3D sphere_center(q[0]/1000,q[1]/1000,altura);
  
  mrpt::poses::CPoint3D ray(0,0,1);
  mrpt::poses::CPose3D rotation;
  rotation.setYawPitchRoll(view.w[3],view.w[4],view.w[5]);
  mrpt::poses::CPoint3D raydir = rotation + ray;
  
  mrpt::poses::CPoint3D ray_origin(view.w[0],view.w[1],view.w[2]);
  
  mrpt::poses::CPoint3D L = ray_origin - sphere_center;
  double a = dotProduct(raydir, raydir);
  double b = 2 * dotProduct(raydir, L);
  double c =  dotProduct(L,L) - radio;
  
  double discriminant;
  discriminant = b * b - 4 * a * c;
  
  if(discriminant < 0)
    return false;
  
  return true;
}


double RobotSensor::dotProduct(mrpt::poses::CPoint3D a, mrpt::poses::CPoint3D b)
{
   return a.m_coords[0] * b.m_coords[0] + a.m_coords[1] * b.m_coords[1] + a.m_coords[2] * b.m_coords[2];
}


