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


#ifndef COCTREEVPL_H
#define COCTREEVPL_H

#include <list>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include "vputils.h"

using namespace octomap;
using namespace std;

/**
 *  Just used for returning values of the ray tracing
 */
enum VoxelTypes
{
  /// Voxel occupied
  VXL_OCCUPIED, 
  /// Voxel Unknown
  VXL_UNKNOWN, 
  ///
  VXL_FREE,
  /// This is unknown voxel, but which as been already touched by a ray in a ray tracing
  VXL_UNKNOWN_TOUCHED,
  ///
  VXL_OCCUPIED_TOUCHED,
  ///
  VXL_UNKNOWN_NOT_CREATED
};


class COctreeVPL: public octomap::ColorOcTree
{
public:
  COctreeVPL(double resolution);
  
/**
 * Cast a ray for hierarchical ray tracing
 * Traces a ray at a given octree depth
 * This particular implementation paints the spanned voxels
 * @param end Return the last point
 */
//   virtual bool castRay2(const point3d& origin, const point3d& directionP, point3d& end,
//                  bool ignoreUnknownCells=false, double maxRange=-1.0, int atDeepth = 0) const;
		
  
  virtual int castRayVPLHierarchical(const point3d& origin, const point3d& directionP, point3d& end,
                 bool ignoreUnknownCells=false, double maxRange=-1.0, int atDeepth = 0);	 
		 
  /**
   * Traces a Ray in the octree and returns the type of voxel found
   */
  virtual int castRayVPL(const point3d& origin, const point3d& directionP, point3d& end,
                 bool ignoreUnknownCells=false, double maxRange=-1.0);
  
  /**
   * Traces a Ray and returns the information gain of a ray
   */
  virtual double castRayIG(const point3d& origin, const point3d& directionP, point3d& end,
                 bool ignoreUnknownCells=false, double maxRange=-1.0);
  
 
  /**
   * This should be used after a ray tracing
   */
  void cleanTouchedVoxels();
		 
  /// 
  void setUnknownThres(double prob){unk_prob_thres_log = logodds(prob); }
  
  /// queries whether a node is occupied according to the tree's parameter for "occupancy"
  bool isNodeUnknown(const OcTreeNode* occupancyNode) const{
      return (occupancyNode->getLogOdds() < this->occ_prob_thres_log && occupancyNode->getLogOdds() >= this->unk_prob_thres_log);
  }
  
  
  /// queries whether a node is occupied according to the tree's parameter for "occupancy"
  inline bool isNodeUnknown(const OcTreeNode& occupancyNode) const{
      return (occupancyNode.getLogOdds() < this->occ_prob_thres_log && occupancyNode.getLogOdds() >= this->unk_prob_thres_log);
  }
  
  
  /**
   * Returns centers of all Uknown voxels, here all voxels that are inside the unknown probability range are used, 
   * plus leafs that do NOT exist (but could) in the set bounding box
   * Upper and lower bounding points must be set before using this function
   */
  void getUnknownCentersAll(point3d_list& node_centers);
  
  
  /**
   * Returns centers of all Uknown voxels, here all voxels that are inside the unknown probability range are used, 
   * plus leafs that do NOT exist (but could) in the set bounding box
   * Upper and lower bounding points must be set before using this function
  */
  void getUnknownVoxels(point3d_list& node_centers, std::vector<double> &sizes);
  
  
  octomap::ColorOcTreeNode::Color colorTouchedUnknown;
  octomap::ColorOcTreeNode::Color colorUnknown;
  octomap::ColorOcTreeNode::Color colorTouchedOccupied;
  octomap::ColorOcTreeNode::Color colorOccupied;
  
  
  
  
protected:
  
  float unk_prob_thres_log;
  
  std::list< ColorOcTreeNode* > touchedNodes;
  //std::list< ColorOcTreeNode* > doubleTouchedNodes;
};

#endif // COCTREEVPL_H
