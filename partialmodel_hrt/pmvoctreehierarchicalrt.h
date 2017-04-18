/*
    Copyright 2013 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef PMVOCTREEHIERARCHICALRT_H
#define PMVOCTREEHIERARCHICALRT_H


#include <sstream>
#include <iomanip>
#include <time.h>

#include <vector>
#include <list>
#include <string>

#include "pmvoctree.h"
#include "pmraynode.h"
#include "coctreevpl.h"

#include "volumetricutilityfunction.h"

typedef std::vector< boost::numeric::ublas::matrix<double> > RaysVector;

typedef std::list< std::list<pmRayNode>::pointer > RayNodePtr_List;

/**
 * Represents a multiresolution octree
 * It includes a tree of octrees and a tree of rays to perform ray tracing
 * 
 * Initialization:
 * 1. setConfigFolder(config_folder);
 * 2. readRays(file_rays);
 * 3. init();
 * 
 * Usage:
 * 1. readCandidateViews(cviews_file);
 * 2. evaluateCandidateViews();
 * 3. saveEvaluatedViews();
 */


class PMVOctreeHierarchicalRT : public PMVOctree
{
public:
  PMVOctreeHierarchicalRT();
  
  virtual bool init();
  
  virtual long int readRays(std::string file_address);
    
  virtual void evaluateCandidateViews();
  
  void setAbstractionLevel(int level);
  
  virtual int evaluateView(ViewStructure &v);
  
protected:
  /// level of abstraction for the multiresolution
  int abstractionLevel; 
  
  // ----------------- Rays Tree members and functions ------------------------------
  // all must have the prefix rt
  
  bool rtHasRoot;
  
  /// Tree of rays. Structure used to perform rays tracing
  std::list<pmRayNode> rtRaysTree;
  
  /// Pointer to the root of the rays tree
  std::list<pmRayNode>::pointer rtRootPtr;
  
 
  /// Control members
  bool hasAbstractionLevel;
  bool hasResolution;
  bool isReady();
  
  inline bool rtIsLeaf(rtPointerToElementType element_ptr);
  
  
  /**
   * @param m [in] Homogenous transformation matrix. It will be applied to the sensor model
   */ 
  bool hierarchicalRayTracing(BoostMatrix m,
			      std::list<pmRayNode>::pointer root_ray_ptr, 
			      int depth,
			      EvaluationResult &result);
  
  
  /** 
   * used by hierarchicalRayTracing
  */
  bool recursiveHRayTracing( BoostMatrix m,
			     std::list<pmRayNode>::pointer root_ray_ptr, 
			     int depth,
			     EvaluationResult &result); 
  
  
//   /**
//    * Traces a ray in one map of the multiresolution octree
//    */  
//   int traceRayInMultiresOctree(BoostMatrix m,
// 			       rtPointerToElementType ray_node,
// 			       moPointerToElement octree_node);

  
  /**
    * Builds the rays tree structure based on the abstraction level resolution and the rays for each resolution
    * file_rays must name the first resolution, for example rays_kinect_0.dat 
    * It is very important that the file ends with "_0.dat"
    * Then the function automatically will search for rays_kinect_1.dat
    */
  virtual void rtGenerateRaysTree(std::string file_rays);
  
  /**
   * Traverse the rays tree and shows info about each node
   */
  void rtTraverseWithInfo();
  
  /**
  * Adds a set of rays R as leafs. USED BY rtGenerateRaysTree
  * Clears the tree
  * Returns a list of pointers to the created leafs
  */
  void rtAddLeafsToTree( RaysVector &R, std::list< std::list<pmRayNode>::pointer > &leafsPtrList);
  
  /**
  * Adds a set of ray R as nodes in the tree. USED BY rtGenerateRaysTree
  * Returns a list of pointers to the created leafs
  */
  void rtAddNodesToTree( RaysVector &R,  RayNodePtr_List &node_ptr_list);
  
  void rtAddRoot(RayNodePtr_List &node_ptr_list);
  
  /**
    * Links a set of Children with a set of parents
    * Each child is assigned to the nearest parent. The distance is the angle between vectors. 
    */
  void rtLinkChildrenWithParents(RayNodePtr_List children, RayNodePtr_List parents);
  
  /**
   * Finds the nearest ray from a set of rays.
   * Returns a pointer to the finded ray.
    * We are assuming that the rays are normalized.
    */
  std::list<pmRayNode>::pointer rtGetNearestRay(std::list<pmRayNode>::pointer node_ptr, RayNodePtr_List node_ptr_list);
  
  bool rtReadRays(std::string file_name, std::vector< boost::numeric::ublas::matrix<double> > &R);  
  
  /**
   * Gets the name of a rays file for a given abstraction level 
   */
  std::string rtGetNameForRaysFile(std::string filename_cero, int level);
  
  
//   int castRayAtAbstractionLevel(BoostMatrix m,
// 			       rtPointerToElementType ray_node,
// 			       int a_l);
  
  int castRayAtDepth(BoostMatrix m, 
		rtPointerToElementType ray_node,
		int depth);
  
  
  /**
   * Verify is point is in the object capsule dilated by resolution
   */
  bool isInCapsule(point3d point, float resolution);
  
  // ////////////////////////////////////////////////////////////////////////////////////////////
};

#endif // PMVOCTREEHIERARCHICALRT_H
