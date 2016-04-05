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


#ifndef PMVOLUMETRIC_H
#define PMVOLUMETRIC_H

#include "vptriangle.h"

#include "partialmodelbase.h"
#include "evaluationresultvector.h"
#include "vufobjectfilters.h"


/**
 * Set of possible voxels in the volumentric representation
 */
enum _VOXEL_TYPES {
  /// (Used for ray tracing control) If the ray is throw from a occupied position invalid mut be returned
  INVALID, 
  
  /// A voxel that contains a surface point of the object
  OCCUPIED, 
  
  /// A voxel that contains surface from the environment
  OCCUPIED_SCENE,
  
  /// An unknow space
  UNMARK,
  
  /// Unknown space in the environment
  UNMARK_SCENE,
  
  /// the rays does not toch any voxel
  RAY_LOST
};


/**
 * Represents the object by dividing the space into cells.
 * Each cell is called voxel.
 * There are several types of voxels.
 */

class PMVolumetric : public PartialModelBase
{
public:
  
// pmVolumetric();
 PMVolumetric();
 
  /**
   * Paints in blue occupied voxels that are inside the object capsule
   */
  //virtual void paintOccupiedInCapsule()=0;
  
  virtual bool stopCriteriaReached();
  
  
  virtual bool init();
    
  /**
   * Checks if a point is in collition
   * parameters in mts
   */
  virtual bool collisionFree(float x, float y, float z);
  
  virtual void saveEvaluations();
 
  /**
   * If the number of unmark voxels is lesst than n then the stop criteria is reached
   */
  void setMinimunNumberOfUnmarkVoxels(int n);
  
  
protected:
  vector<point3d> touched_voxels;
  
  ///Object Bounding Box
  octomap::point3d ObjectBBxMin;
  ///Object Bounding Box
  octomap::point3d ObjectBBxMax;
  
  ///Scene Bounding Box
  octomap::point3d SceneBBxMin;
  ///Object Bounding Box
  octomap::point3d SceneBBxMax;
  
  // Max range for inserting scans
  double maxRange;
  
  // Evaluaciones para guardar
  EvaluationResultVector evals;
  
  /// resolution of the voxel
  float voxelResolution;  
  
  /// specifies is there is a free space
  bool freeSpace;
  
  /// weight for selection either exploration or surface.
  float weight;
  
  /// Free space variables
  float x_free_1, y_free_1, z_free_1;
  float x_free_2, y_free_2, z_free_2;
   
  /// low collition cheking
  float collisionGap;
  
  /** 
   * Stop criteria threshold
   * Minimun number of unmark voxels for the next bext view
   */
  int minUnknown;
  
  int minOverlap;
  
  bool thereIsUnmarkVoxelsInNBV;
  
  bool newInformationAcquired;
  
  bool stopCriteria;
  
  VolumetricUtilityFunction *utilityFunction;
  
  /**
   * Returns the list of eight vertices.
   */
  inline void getVoxelVertices(point3d_list &vertices, const point3d center, const double resolution);
  
  /**
   * Returns the lisf of triangles from a voxel
   */
  void getTrianglesOfVoxel(vpTriangleList &tris, const point3d center, const double resolution);
  
  
  //void getTrianglesOfVoxels(vpTriangleList &tris)
  
};

#endif // PMVOLUMETRIC_H
