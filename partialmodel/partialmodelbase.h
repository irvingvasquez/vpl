/*
 * 
 * 
Partial Model Library
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


#ifndef PARTIALMODELBASE_H
#define PARTIALMODELBASE_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <list>
#include <string>
#include <ctime>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <math.h>
//#include <iniparser.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFile.h>

#include "vpfilereader.h"
#include "viewstructure.h"
#include "pmutils.h"



using namespace octomap;
typedef boost::numeric::ublas::matrix<double> BoostMatrix;

/**
 * The partial model stores information about the scene and provides functions that are needed to evaluate a candidate sensor view. 
 * This class implements the base for a partial model,
 * several functions related with the partial model are provided, for example update with a point cloud.
 * This implementation stores a set of candidate views and can assign a evaluation to each candidate view according to a certain metric.
 * Derived classes will implement the evaluations. 
 * This class implements the evaluation of the candidate views and the model update.
 * Units: meters and rads
 */


class PartialModelBase
{
public:

PartialModelBase();
  
  virtual float updateWithScan(std::string file_name_scan, std::string file_name_origin)=0;
  
  virtual void evaluateCandidateViews()=0;
  
  void evaluateCandidateViews(ViewList &views);
  
  virtual int evaluateView(ViewStructure &v)=0;
  
  virtual bool stopCriteriaReached()=0;
  
  virtual bool savePartialModel(std::string file_name)=0;
  
  virtual bool loadPartialModel(std::string file_name)=0;
  
  /**
   * Returns an aproximation of the unknown volume;
   */
  virtual double getUnknownVolume()=0;
  
  /**
  * Reads the rays (directions) thought by the sensor
  * They are stored at rays with the notation [x, y, z, 1]^T
  * The 1 is added to perform matrix multiplications by a HTM
  */ 
  virtual long int readRays(std::string file_address)=0;
  
  
  virtual bool init();
  
  /**
   * Saves the evaluated views, 
   * HTM is saved in meters
   * (how was readed)
   */
  bool saveEvaluatedViews(std::string file_name);
  
  
  /**
   * Reads candidate views from a given file
   */
  bool readCandidateViews(std::string file_name);
  
  
  /**
   * selects only the n early candidate views
   * rest is deleted
   * TM is saved in meters 
   * (how was readed)
   */ 
  bool saveOnlyNViews(int n, std::string file_name);
  
  
  /**
   * Orders the views according to their evaluation
   * From high to low
   */
  void sortCandidateViews();
  
  
  /**
   * Establishes the limits for the capsule where the object is.
   */
  void setObjectCapsule(double x1, double y1, double z1, double x2, double y2, double z2);
  
  
  /**
  * Establishes the limits for the Reconstruction Scene.
  */
  void setScene(double x1, double y1, double z1, double x2, double y2, double z2);
  
  
//   void setPaintPartialModel(bool value);
  
  void setConfigFolder(std::string folder);
  
  void setDataFolder(std::string folder);
  
  virtual void saveEvaluations()=0;
  
  
  /**
   * Saves the occupied and uknown voxels inside the object bouding box as a list of triangles.
   */
  virtual void saveObjectAsRawT(std::string file_name)=0;
  
  
  /**
   * Saves the occupied and uknown voxels inside the object bouding box as a list of triangles.
   */
  virtual void saveObjectAsObst(std::string file_name)=0;
  
  /**
  *
  */
  virtual bool saveUnknownVolumeAsObst(std::string file_name)=0;
  
  /**
   * 
   */
  virtual bool saveUnknownVolumeAsRawT(std::string file_name)=0;
  
  /**
   * Saves the occupied  and unknown voxels as Obs
   */
  virtual bool saveObstacle(std::string file_name)=0;  
  
  virtual bool saveVisibleUnknown(std::string file_name_vertex, std::string file_name_normal)=0;
  
  virtual bool saveFrontierUnknown(std::string file_name_vertex, std::string file_name_normal)=0;
  
  virtual void getOccupiedTriangles(vpTriangleList &tris) = 0;
  
  virtual void getUnknownTriangles(vpTriangleList &tris) = 0;
  
  /**
   * Gets the object bounding box parameters
   */
  void getOBB(double &x1, double &y1, double &z1, double &x2, double &y2, double &z2);
  
  bool poitsToTheObject(ViewStructure &v);
  
  /**
   * Accumulated points from the reconstructions
   */
  std::string object_points_filename;
  
protected:
  
  bool pointsToASphere(ViewStructure &v, double center_x, double center_y, double center_z, double radius);
  
  /// Object accumulated point cloud
  Pointcloud objectPointCloud;
  
  point3d objectSphereCenter;
  double objectSphereRadius;
  double objRadius2;
  point3d directorRay;
  
  std::string configFolder;
  std::string dataFolder;
  std::string evaluationsFile;
  
  /// robot bounding box size
  //double rbb_size; 
  
  std::list<ViewStructure> candidateViews;
  
  /// Sensor information
  std::vector< boost::numeric::ublas::matrix<double> > rays;
  // minimun depth of view. Distance in meters
  float minDOV;
  // maximun depth of view. Distance in meters
  float maxDOV;
  
  /**
   * Checks if a hit point is inside the depth of view
  */
  bool isInDOV(octomap::point3d origin, octomap::point3d point);
  
  
  octomap::point3d *scanOrigin;
  octomap::Pointcloud *scanCloudOrigins;
  octomap::Pointcloud *scanCloud;
  
  // Object Capsule variables
  double x_cap_1, y_cap_1, z_cap_1;
  double x_cap_2, y_cap_2, z_cap_2;
  
  // Scene capsule
  double x_sce_1, y_sce_1, z_sce_1;
  double x_sce_2, y_sce_2, z_sce_2;
  
  double objectResolution;
  
  /**
   * Checks if a point is inside the object capsule
   */
  bool isInCapsule(octomap::point3d point);
  
  
  bool isInScene(octomap::point3d point);
  
  
   /**
   * Registration contraints
   */
  virtual bool registrationLowConstraint(long int n_occupied);
  
  /**
  * Reads a scan
  * The file is an ASCII Format with plain data
  */
  bool readPointCloudFromDAT(std::string file_name, octomap::Pointcloud &cloud);
  
  
  void computeRayFromOriginToPoint(double x, double y, double z, double & i, double &j, double &k);
  
  void computeRayFromPointToPoint(BoostMatrix pointA, BoostMatrix pointB,  double & i, double &j, double &k);
  
  
  //------------- colors --------------------
  octomap::ColorOcTreeNode::Color colorTouchedUnkmark;
  octomap::ColorOcTreeNode::Color colorUnmark;
  octomap::ColorOcTreeNode::Color colorTouchedOccupied;
  octomap::ColorOcTreeNode::Color colorOccupied;
  
  octomap::ColorOcTreeNode::Color colorOrigin;
  octomap::ColorOcTreeNode::Color colorRed;
  octomap::ColorOcTreeNode::Color colorBlue;
  octomap::ColorOcTreeNode::Color colorYellow;
  octomap::ColorOcTreeNode::Color colorCian;
  octomap::ColorOcTreeNode::Color colorOrange;
  octomap::ColorOcTreeNode::Color colorGray;
  
  /**
   * Stadistical variables. Used for compare reconstructions:
   */
  std::vector<long int> unknownVoxelsInOBBx;
  
  /*
   * this should be specified in case that the sensor point to another direction for example the kinect reference frame is not the usual
   */
  std::vector<double> sensorReferenceFramePose;
  
private:
  
  /**
   * Verifica si un rayo ray dir con origen rayorigin intersecta el objeto encapsulado en una esfera
   * los parámetros de la esfera se leen desde el archivo de configuración
   */
  inline bool rayIntersectObjectSphere(const point3d raydir, const point3d rayorig);
  
  /**
   * Verifica si un rayo intersecta una esfera
   */
  inline bool rayIntersectSphere(const point3d raydir, const point3d rayorig, const point3d sphere_center, double radius);
};

#endif // PARTIALMODELBASE_H
