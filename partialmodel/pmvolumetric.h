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


#ifndef PMVOLUMETRIC_H
#define PMVOLUMETRIC_H

#include "vptriangle.h"
#include "partialmodelbase.h"
#include "evaluationresult.h"

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
  
//pmVolumetric();
PMVolumetric();
virtual ~PMVolumetric(); 

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
  
  virtual bool saveVisibleUnknown(std::string file_name_vertex, std::string file_name_normal)=0;
  
  virtual bool insertUnknownSurface(Pointcloud pc)=0;
  
//   virtual void getOBBVoxelGrid(std::vector< std::vector< std::vector<double> > > &voxels)=0;
  
protected:
  std::vector<point3d> touched_voxels;
  
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
  //EvaluationResultVector evals;
  
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
  
  //VolumetricUtilityFunction *utilityFunction;
  
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
