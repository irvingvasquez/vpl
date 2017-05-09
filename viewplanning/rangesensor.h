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


#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include <vector>
#include <string>

#include <mrpt/base/include/mrpt/poses.h>
#include <mrpt/utils.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFile.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <viewstructure.h>
#include <pmutils.h>

using namespace PMUtils;

/**
 * Defines a Range Sensor. 
 * It abstracs the communication with the device.
 * Returns a set of poins in the 3D space as a read.
 * All units must be in mts and rads;
 */

class RangeSensor
{

public:
RangeSensor();

  virtual bool init();

  /** 
   * Reads the sensor and get 3D points in mm
   */
  virtual long int getPoints(std::vector<mrpt::poses::CPoint3D> &points)=0;
  
  /**
   * Returns the set of rays which define the sensor
   */
  virtual void getRays(std::vector< std::vector<double> > &rays);
  
  
  bool saveRays(std::string filename);
  
  /**
   * Computes a set of minimun rays for the voxel resolution especified
   * resolution must have the same units than distance
   */ 
  virtual long int getRaysForResolution(std::vector< std::vector<double> > &rays, double resolution, double distance);
  
  /**
   * Saves a "ideal" subset of rays depending on the resolution of the octree and the distance from the sensor
   * resolution must have the same units than distance
   */
  bool saveRaysForResolution(std::string filename, double resolution, double distance);
  
  
  /**
   * Returns information about the sensor
   */
  void getInfo(std::string &txt);
  
  void setConfigFolder(std::string folder);
  
  void setDataFolder(std::string folder);
  
  void getDirectorRay(std::vector<double> &ray);
  
  void setCurrentView(ViewStructure v);
  
protected:
  double h_aperture;
  double v_aperture;
  
  int h_points;
  int v_points;
  
  std::string info;
  bool ready;
  
  std::string configFolder;
  std::string dataFolder;
  
  std::vector<double> director_ray;
 
  /// Only required for simulated sensors
  ViewStructure currentView;
  
private:
  //vector< std::vector<double> > rays;
  
};

#endif // RANGESENSOR_H
