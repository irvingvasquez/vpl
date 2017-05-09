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


#include "rangesensor.h"


RangeSensor::RangeSensor():
director_ray(3)
{
  ready = false;
}



bool RangeSensor::saveRays(std::string filename)
{
  vpFileReader writer;
  std::vector< std::vector< double > > rays;
  this->getRays(rays);
  return writer.saveDoubleCoordinates(filename, rays);
}


long int RangeSensor::getRaysForResolution(std::vector< std::vector< double > >& rays, double resolution, double distance)
{
  rays.clear();
  
  long int n_rays = 0;
  
  std::vector<double> ray(3);
  double d;
  int i, j;
  double alpha, beta;
  
  double increment_h_a, increment_v_a;
  
  long int h_points_temp;
  long int v_points_temp;
  
  h_points_temp = h_aperture / (asin(resolution / distance));
  v_points_temp = v_aperture / (asin(resolution / distance));
  
  //compute increments
  increment_h_a = h_aperture / (h_points_temp -1); 
  increment_v_a = v_aperture / (v_points_temp -1);
  
  alpha = -h_aperture/2;
  for(j=0;j<h_points_temp;j++){
    beta = -v_aperture/2;
    for(i=0;i<v_points_temp;i++){
      ray[1] = sin(beta);
      d = cos(beta);
      ray[2] = d * cos(alpha);
      ray[0] = d * sin(alpha);
      
      rays.push_back(ray);
      n_rays ++;
      
      beta = beta + increment_v_a;
      
    }
    alpha = alpha + increment_h_a;
  }
  
  return n_rays;
}


bool RangeSensor::saveRaysForResolution(std::string filename, double resolution, double distance)
{
  vpFileReader writer;
  std::vector< std::vector< double > > rays;
  this->getRaysForResolution(rays, resolution, distance);
  return writer.saveDoubleCoordinates(filename, rays);
}

void RangeSensor::getInfo(std::string &txt)
{
  txt = info;
}

bool RangeSensor::init()
{
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("rangeSensor.ini");
  
  dictionary * ini_file;
    
  ini_file = iniparser_load(config_file.c_str());
  if (ini_file ==NULL ) {
      fprintf(stderr, "cannot parse file: %s\n", config_file.c_str());
      return false ;
  }
    //iniparser_dump(ini_file, stderr);
    
  h_aperture = iniparser_getdouble(ini_file,"rangeSensor:h_aperture", 1);
    
  v_aperture = iniparser_getdouble(ini_file, "rangeSensor:v_aperture", 1);
    
  h_points = iniparser_getint(ini_file,"rangeSensor:h_points", 1);
    
  v_points = iniparser_getint(ini_file, "rangeSensor:v_points", 0.1);
  
  info.clear();
  info.assign(iniparser_getstring(ini_file, "rangeSensor:info", "error"));
  
  director_ray[0] = iniparser_getdouble(ini_file, "directorRay:x", -1);
  director_ray[1]  = iniparser_getdouble(ini_file, "directorRay:y", -1);
  director_ray[2]  = iniparser_getdouble(ini_file, "directorRay:z", -1);
    
  std::cout << "\n---------- Range sensor -------" << std::endl;
  std::cout << "Horizontal aperture: " << h_aperture << std::endl;
  std::cout << "Vertial aperture: " << v_aperture << std::endl;
  std::cout << "horizontal points:" << h_points << std::endl;
  std::cout << "Vertial points:" << v_points << std::endl;
  std::cout << "Director ray: [" << director_ray[0] << ", " << director_ray[1] << ", " << director_ray[2] << "]" << std:: endl;
  
  return true;
}

void RangeSensor::setConfigFolder(std::string folder)
{
  configFolder = folder;
}

void RangeSensor::setDataFolder(std::string folder)
{
  dataFolder = folder;
}




void RangeSensor::getRays(std::vector< std::vector< double > >& rays)
{
  rays.clear();
  
  std::vector<double> ray(3);
  double d;
  int i, j;
  double alpha, beta;
  
  double increment_h_a, increment_v_a;
  
  //compute increments
  increment_h_a = h_aperture / (h_points -1); 
  increment_v_a = v_aperture / (v_points -1);
  
  alpha = -h_aperture/2;
  for(j=0;j<h_points;j++){
    beta = -v_aperture/2;
    for(i=0;i<v_points;i++){
      ray[1] = sin(beta);
      d = cos(beta);
      ray[2] = d * cos(alpha);
      ray[0] = d * sin(alpha);
      
      rays.push_back(ray);
      
      beta = beta + increment_v_a;
      
    }
    alpha = alpha + increment_h_a;
  }
  
  return;
}


void RangeSensor::getDirectorRay(std::vector< double >& ray)
{
  ray = director_ray;
}


void RangeSensor::setCurrentView(ViewStructure v)
{
  currentView = v;
}

