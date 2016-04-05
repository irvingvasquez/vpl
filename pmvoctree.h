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


#ifndef PMVOCTREE_H
#define PMVOCTREE_H

#include "pmvolumetric.h"
#include "volumetricutilityfunction.h"
#include "coctreevpl.h"


/**
 * Octree class for view planning in 3D object Reconstruction.
 * 
 * Initialization:
 * 	- Set config folder
 * 	- Set data folder 
 * 	- init()
 * 	- readRays();
 * 	- set utility function
*/

class PMVOctree :  public PMVolumetric
{

public:
PMVOctree();

  virtual float updateWithScan(string file_name_scan, string file_name_origin);
  
  virtual bool savePartialModel(string file_name);
  
  virtual bool loadPartialModel(string file_name);
  
  virtual void evaluateCandidateViews();
  
  virtual int evaluateView(ViewStructure &v);
  
  /**
  * Reads the rays (directions) thought by the sensor
  * They are stored at rays with the notation [x, y, z, 1]^T
  * The 1 is added to perform matrix multiplications by a HTM
  */ 
  virtual long int readRays(string file_address);
  

   
  //virtual void paintOccupiedInCapsule();
  
  /**
   * Paints in blue occupied voxels that are inside the object capsule.
   * Paints in orange unknown voxels
   */
  long int paintVoxels(COctreeVPL *octree);
  
  virtual bool init();
  
  /**
   * Inserts a cube of free space given by two xtreme points
   */
  void insertFreeSpace(double x1, double y1, double z1, double x2, double y2, double z2);  
  
//   void saveEvaluations();
  
  void setUtilityFunction(VolumetricUtilityFunction *uf);
  
  virtual void saveObjectAsRawT(string file_name);

  virtual void saveObjectAsObst(string file_name);
  
  virtual bool saveUnknownVolumeAsObst(string file_name);
  
  virtual bool saveUnknownVolumeAsRawT(string file_name);
  
  /**
   * Saves the occupied  and unknown voxels as Obs
   */
  virtual bool saveObstacle(string file_name);
  
  virtual void getOccupiedTriangles(vpTriangleList &tris);
  
  virtual void getUnknownTriangles(vpTriangleList &tris);
  
virtual double getUnknownVolume();

protected:
  
  /// Octree representation
  //octomap::ColorOcTree *map;
  COctreeVPL *map;
  
  
  /**
   * Performs a ray tracing to gather how much voxels ware touched. It uses the translation of the HTM like origin of the ray tracing.
   * Returns false if the origin is not free.
   * @param m [in] Homogenous transformation matrix. It will be applied to the sensor model
   * @param n_occupied [out]
   * @param n_unmark [out]
   */
  bool rayTracingHTM(boost::numeric::ublas::matrix<double> m, EvaluationResult &result);
  
};

#endif // PMVOCTREE_H
