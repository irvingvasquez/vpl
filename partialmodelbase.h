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


#ifndef PARTIALMODELBASE_H
#define PARTIALMODELBASE_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <list>
#include <string>
#include <ctime>

#include <math.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <iniparser.h>

#include "pmdefinitions.h"
#include "vpfilereader.h"
#include "viewstructure.h"

using namespace std;
using namespace octomap;

/**
 * A partial model stores the information about the object.
 * Class that represents the partial model.
 * This class implements the evaluation of the candidate views and the model update.
 * Units mts and rads
 */

class PartialModelBase
{
public:

PartialModelBase();
  
  virtual float updateWithScan(string file_name_scan, string file_name_origin)=0;
  
  virtual void evaluateCandidateViews()=0;
  
  virtual int evaluateView(ViewStructure &v)=0;
  
  virtual bool stopCriteriaReached()=0;
  
  virtual bool savePartialModel(string file_name)=0;
  
  virtual bool loadPartialModel(string file_name)=0;
  
  /**
   * Returns an aproximation of the unknown volume;
   */
  virtual double getUnknownVolume()=0;
  
  /**
  * Reads the rays (directions) thought by the sensor
  * They are stored at rays with the notation [x, y, z, 1]^T
  * The 1 is added to perform matrix multiplications by a HTM
  */ 
  virtual long int readRays(string file_address)=0;
  
  virtual bool init();
  
  /**
   * Saves the evaluated views, 
   * HTM is saved in mts
   * (how was readed)
   */
  bool saveEvaluatedViews(string file_name);
  
  /**
   * Reads candidate views from a given file
   */
  bool readCandidateViews(string file_name);
  
  /**
   * selects only the n early candidate views
   * rest is deleted
   * TM is saved in mts 
   * (how was readed)
   */ 
  bool saveOnlyNViews(int n, string file_name);
  
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
  
  void setConfigFolder(string folder);
  
  void setDataFolder(string folder);
  
  virtual void saveEvaluations()=0;
  
  
  /**
   * Saves the occupied and uknown voxels inside the object bouding box as a list of triangles.
   */
  virtual void saveObjectAsRawT(string file_name)=0;
  
  
  /**
   * Saves the occupied and uknown voxels inside the object bouding box as a list of triangles.
   */
  virtual void saveObjectAsObst(string file_name)=0;
  
  /**
  *
  */
  virtual bool saveUnknownVolumeAsObst(string file_name)=0;
  
  /**
   * 
   */
  virtual bool saveUnknownVolumeAsRawT(string file_name)=0;
  
  /**
   * Saves the occupied  and unknown voxels as Obs
   */
  virtual bool saveObstacle(string file_name)=0;
  
  virtual void getOccupiedTriangles(vpTriangleList &tris) = 0;
  
  virtual void getUnknownTriangles(vpTriangleList &tris) = 0;
  
  /**
   * Gets the object bounding box parameters
   */
  void getOBB(double &x1, double &y1, double &z1, double &x2, double &y2, double &z2);
  
  bool poitsToTheObject(ViewStructure &v);
  
  bool pointsToASphere(ViewStructure &v, double center_x, double center_y, double center_z, double radius);
  
  /**
   * Accumulated points from the reconstructions
   */
  string object_points_filename;
  
  
  /// Stadistical variables
  // float std_utility;
  // float std_surface_u;
  // float std_unk_voxel;
  // float std_occ_voxel;
  // float std_updt_t;
  // float std_evaluation_t;
  
  
protected:
  
  /// Object accumulated point cloud
  Pointcloud objectPointCloud;
  
  point3d objectSphereCenter;
  double objectSphereRadius;
  double objRadius2;
  point3d directorRay;
  
  string configFolder;
  string dataFolder;
  string evaluationsFile;
  
  /// robot bounding box size
  //double rbb_size; 
  
  list<ViewStructure> candidateViews;
  
  /// Sensor information
  vector< boost::numeric::ublas::matrix<double> > rays;
  // minimun depth of view. Distance in mts
  float minDOV;
  // maximun depth of view. Distance in mts
  float maxDOV;
  
  /**
   * Checks if a hit point is inside the depth of view
  */
  bool isInDOV(octomap::point3d origin, octomap::point3d point);
  
  //bool paintPartialModel;
  
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
  bool readPointCloudFromDAT(string file_name, octomap::Pointcloud &cloud);
  
  
  void computeRayFromOriginToPoint(double x, double y, double z, double & i, double &j, double &k);
  
  void computeRayFromPointToPoint(BoostMatrix pointA, BoostMatrix pointB,  double & i, double &j, double &k);
  
  
  //------------- utils --------------------
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
  vector<long int> unknownVoxelsInOBBx;
  
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
