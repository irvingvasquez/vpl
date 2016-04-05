/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

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
  virtual bool castRay2(const point3d& origin, const point3d& directionP, point3d& end,
                 bool ignoreUnknownCells=false, double maxRange=-1.0, int atDeepth = 0) const;
		
  
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
