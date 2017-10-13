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


#ifndef RANGESIMULATOROCTREE_H
#define RANGESIMULATOROCTREE_H

#include "rangesimulatorbase.h"
#include <octomap/ColorOcTree.h>
#include <boost/numeric/ublas/matrix.hpp>

//typedef boost::numeric::ublas::matrix<double> boost::numeric::ublas::matrix<double>;
/** 
 * Initialization:
 * 
 * setConfigFolder(config_folder);
 * setDataFolder(data_folder); 
 * readRays(rays); 
 * init(); 
 * loadModel(model);
*/

class RangeSimulatorOctree : public RangeSimulatorBase
{

public:
RangeSimulatorOctree();

virtual bool init();

/**
 * Load a 3D model from a file.
 * The file must be a wrl file.
 */
virtual bool loadModel(std::string file);


virtual bool takeAndSaveScan( ViewStructure v, std::string file_name );

virtual bool takeAndSaveScan( boost::numeric::ublas::matrix<double> htm, std::string &scan_name, std::string &origin_name);

virtual bool save(std::string file);

/**
  * Reads the rays (directions) thought by the sensor
  * They are stored at rays with the notation [x, y, z, 1]^T
  * The 1 is added to perform matrix multiplications by a HTM
  */ 
virtual long int readRays(std::string file_address);


/**
   * Performs a ray tracing to gather how much voxels ware touched. It uses the translation of the HTM like origin of the ray tracing.
   * Returns false if the origin is not free.
   * @param m [in] Homogenous transformation matrix. It will be applied to the sensor model
   */
bool rayTracingPointCloud(boost::numeric::ublas::matrix<double> m, octomap::Pointcloud &cloud);


void insertFreeSpace(double x1, double y1, double z1, double x2, double y2, double z2);

protected:
  /// Sensor information
  std::vector< boost::numeric::ublas::matrix<double> > rays;
  
  /// Octree representation
  octomap::ColorOcTree *model;
  float maxLenght;
  
  float voxelResolution;
  bool freeSpace;
  
  float x_free_1; 
  float x_free_2; 
  float y_free_1;
  float y_free_2;
  float z_free_1;
  float z_free_2; 
  
  //float cx,cy,cz;
  
  octomap::pose6d scene_transformation;
  float scene_scale;

  void computeRayFromPointToPoint(boost::numeric::ublas::matrix<double> pointA, boost::numeric::ublas::matrix<double> pointB,  double & i, double &j, double &k);
  
  bool isRawFormat;
  bool measureUnknown; // If true, the the touched unknown voxels will be considered as measures, otherwise only occupied voxels will be measured
  
};

#endif // RANGESIMULATOROCTREE_H
