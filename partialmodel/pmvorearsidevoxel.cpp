/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "pmvorearsidevoxel.h"


COctreeRearSideVoxel::COctreeRearSideVoxel(double resolution): COctreeVPL(resolution)
{
 // ok
  octomap::ColorOcTreeNode::Color colorPink(255,0,255);
  colorRearSide = colorPink;
  
}

int COctreeRearSideVoxel::castRayVPL(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknownCells, double maxRange)
{
 /// ----------  see OcTreeBase::computeRayKeys  -----------
    int lastTouchedVoxel = VXL_UNDEFINED;
    bool unknownPassed = false;
    point3d unknownEnd;
    ColorOcTreeNode* lastNode;
    
    // Initialization phase -------------------------------------------------------
    OcTreeKey current_key;
    if ( !coordToKeyChecked(origin, current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }

    ColorOcTreeNode* startingNode = this->search(current_key);
    if (startingNode){
      if (this->isNodeOccupied(startingNode)){
        // Occupied node found at origin 
        // (need to convert from key, since origin does not need to be a voxel center)
        end = this->keyToCoord(current_key);
        return true;
      }
    } else if(!ignoreUnknownCells){
      OCTOMAP_ERROR_STR("Origin node at " << origin << " for raycasting not found, does the node exist?");
      end = this->keyToCoord(current_key);
      return false;
    }

    point3d direction = directionP.normalized();
    bool max_range_set = (maxRange > 0.0);

    int step[3]; 
    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      // compute step direction
      if (direction(i) > 0.0) step[i] =  1;
      else if (direction(i) < 0.0)   step[i] = -1;
      else step[i] = 0;

      // compute tMax, tDelta
      if (step[i] != 0) {
        // corner point of voxel (in direction of ray)
        double voxelBorder = this->keyToCoord(current_key[i]);
        voxelBorder += double(step[i] * this->resolution * 0.5);

        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = this->resolution / fabs( direction(i) );
      }
      else {
        tMax[i] =  std::numeric_limits<double>::max();
        tDelta[i] = std::numeric_limits<double>::max();
      }
    }

    if (step[0] == 0 && step[1] == 0 && step[2] == 0){
    	OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
    	return false;
    }

    // for speedup:
    double maxrange_sq = maxRange *maxRange;

    // Incremental phase  --------------------------------------------------------- 

    bool done = false;

    while (!done) {
      unsigned int dim;

      // find minimum tMax:
      if (tMax[0] < tMax[1]){
        if (tMax[0] < tMax[2]) dim = 0;
        else                   dim = 2;
      }
      else {
        if (tMax[1] < tMax[2]) dim = 1;
        else                   dim = 2;
      }

      // check for overflow:
      if ((step[dim] < 0 && current_key[dim] == 0)
    		  || (step[dim] > 0 && current_key[dim] == 2* this->tree_max_val-1))
      {
        OCTOMAP_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
        // return border point nevertheless:
        end = this->keyToCoord(current_key);
        return false;
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];


      // generate world coords from key
      end = this->keyToCoord(current_key);

      // check for maxrange:
      if (max_range_set){
        double dist_from_origin_sq(0.0);
        for (unsigned int j = 0; j < 3; j++) {
          dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
        }
        if (dist_from_origin_sq > maxrange_sq)
          return false;
      }
      
      ///  castRayVPL  !!!!!
      ColorOcTreeNode* currentNode = this->search(current_key);
      if (currentNode){
        if (this->isNodeOccupied(currentNode)) {
	  // Isler cuenta para cada rayo, asi que no es necesario verificar si ya fue tocado ese voxel.
	  if(lastTouchedVoxel == VXL_UNKNOWN){
	    //lastNode->setColor(colorRearSide);
	    //currentNode->setColor(colorTouchedOccupied);
	    // agregarlo a la lista
	    //touchedNodes.push_back(lastNode);
	    //touchedNodes.push_back(currentNode);
	    return VXL_REAR_SIDE;
	  } else {
	    //currentNode->setColor(colorTouchedOccupied);
	    //touchedNodes.push_back(currentNode);
	    return VXL_OCCUPIED;
	  }
        } else {
	  if (this->isNodeUnknown(currentNode)) {
	    // Como estamos contando voxeles rear side, si es unknown también pasará de largo el rayo, pero anotaremos que tipo fue el que pasamos.
	    lastTouchedVoxel = VXL_UNKNOWN;
	    if(!unknownPassed){
	      unknownPassed = true;
	      unknownEnd = end;
	    }
	  } else {
	    lastTouchedVoxel = VXL_FREE;
	  }
	}
        // otherwise: node is free and valid, raycasting continues
        lastNode = currentNode;
      } else if (!ignoreUnknownCells){ // no node found, this usually means we are in "unknown" areas
        //OCTOMAP_WARNING_STR("Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end);
	if(unknownPassed){
	  end = unknownEnd;
	  return VXL_UNKNOWN;
	} else {
	  return VXL_UNKNOWN_NOT_CREATED;
	}
      }
    } // end while

    return VXL_FREE;
}



PMVORearSideVoxel::PMVORearSideVoxel()
{
 // ok
}



int PMVORearSideVoxel::evaluateView(ViewStructure& v)
{
  if(!poitsToTheObject(v)){
    //cout << "Sorry no points :(" << std::endl;
    return UNFEASIBLE_VIEW;
  }
  
  EvaluationResult result;
  
  bool valid_result = rayTracingHTM(v.HTM, result);
  
  /// Evaluate the result of the raytracing
  if(valid_result){
      v.n_unknown = result.n_unknown;
      v.n_occupied = result.n_occupied;
      v.n_occplane = result.n_unknown;

      // check for registration constraint
      if(registrationConstraint(result)){
	// evaluation of the view by counting the occplane voxels (unknown voxels of the surface)
	v.eval = (float) result.n_rear_side;
	return FEASIBLE_VIEW;
      } else {
	v.eval = 0.0;
	return UNFEASIBLE_VIEW;
      }
  } else 
  { 
    v.eval = 0.0;
    return UNFEASIBLE_VIEW;
  }
}


bool PMVORearSideVoxel::init()
{
  PMVolumetric::init();
  
  std::cout << "---------------- Octree -------------------" << std::endl;
  
  //map = new octomap::ColorOcTree(voxelResolution);
  //map = new COctreeVPL(voxelResolution);
  map = new COctreeRearSideVoxel(voxelResolution);
  
  map->colorOccupied = colorOccupied;
  map->colorUnknown = colorUnmark;
  
  map->colorTouchedOccupied = colorTouchedOccupied;
  map->colorTouchedUnknown = colorTouchedUnkmark;
  
  map->setBBXMax(ObjectBBxMax);
  map->setBBXMin(ObjectBBxMin);
  
  
  // If freeSpace then free voxels are inserted
  if(freeSpace)
    insertFreeSpace(x_free_1,y_free_1,z_free_1,x_free_2,y_free_2,z_free_2);
  
  // The boundries of the object capsule is dilated by voxel resolution
  x_cap_1 -= 2 * voxelResolution;
  y_cap_1 -= 2 * voxelResolution;
  z_cap_1 -= 2 * voxelResolution;
  
  x_cap_2 += 2* voxelResolution;
  y_cap_2 += 2* voxelResolution;
  z_cap_2 += 2* voxelResolution;
  
  std::cout << "-------------------------------------------" << std::endl;
}


bool PMVORearSideVoxel::rayTracingHTM(boost::numeric::ublas::matrix< double > m, EvaluationResult& result)
{
  // Funcion revisada! ok!
  double i_ray,j_ray,k_ray;
  
  std::vector< boost::numeric::ublas::matrix<double> >::iterator it;
  BoostMatrix rotated_ray;
  BoostMatrix cero_origin(4,1);
  cero_origin(0,0) = 0;
  cero_origin(1,0) = 0;
  cero_origin(2,0) = 0;
  cero_origin(3,0) = 1;
  BoostMatrix rotated_origin;
  BoostMatrix ray;
  
  octomap::point3d *direction;
  octomap::point3d touched_position;
  
  octomap::ColorOcTreeNode *touched_node;
  octomap::ColorOcTreeNode *origin_node;
  
  //compute origin
  rotated_origin = boost::numeric::ublas::prod(m, cero_origin);
  octomap::point3d origin(rotated_origin(0,0), rotated_origin(1,0), rotated_origin(2,0));
  //cout << "origin: " << std::endl << origin << std::endl;
  
  // check for collision
  origin_node = map->search(origin);
  if(origin_node == NULL){
    //cout << "Origin not found. It could be in a unknown part" << std::endl;
    //cout << origin.x() << " " << origin.y() << " " << origin.z() << std::endl;
    return false;
  }
  
  for(it = rays.begin(); it!= rays.end(); it++){
     /// The ray is rotated and traslated by the rotation matrix
     ray = *it;
     rotated_ray = boost::numeric::ublas::prod(m, ray);
     //cout << "rotated_ray: " << rotated_ray(0,0) << " " <<  rotated_ray(1,0) << " " << rotated_ray(2,0) << std::endl;
     
     //compute direction from position to the rotated ray. This is necesary because the rotated ray was also trasladated by the rotation matrix
     computeRayFromPointToPoint(rotated_origin , rotated_ray, i_ray, j_ray, k_ray);
     
     direction = new octomap::point3d(i_ray, j_ray, k_ray);
     //cout << "direction: " << direction->x() << " " << direction->y() << " " << direction->z() << std::endl;
     
     // if the casted ray returns true a occupied voxel was hit
     int answer = map->castRayVPL(origin, *direction, touched_position); 
     if (answer == VXL_OCCUPIED){
       if(isInDOV(origin,touched_position)) {
	  if(isInCapsule(touched_position)){
	    result.n_occupied ++;
	  } else {
	    result.n_occupied_scene ++;
	  }
       } else {
	 result.n_lost ++;
       }
     } else if (answer == VXL_UNKNOWN || answer == VXL_UNKNOWN_NOT_CREATED){
       if(isInDOV(origin, touched_position)){
	  if(isInCapsule(touched_position)){
	    result.n_unknown ++; 
	  } else if(isInScene(touched_position)){
	    result.n_unknown_scene++;
	  } else {
	    result.n_lost++;
	  }
       } else {
	 result.n_lost ++;
       }
     } else {
       if(answer == VXL_REAR_SIDE){
	 result.n_rear_side++;
	 // tambien incrementamos los unknown por que para alcanzar un rearside pasó por un uknown
	 result.n_unknown++;
       } else{
	 result.n_lost ++;
       }
     }
     
     delete direction;
  }
//   std::cout << "RT. Occ:" << result.n_occupied << " Occ_sce:" << result.n_occupied_scene 
//  		    << " Unk:" << result.n_unknown <<  " Unk_sce:" << result.n_unknown_scene << " RearSide:" << result.n_rear_side  
//  		    << " lost:" << result.n_lost << std::endl;
// 
//   // WARNING
//   if(result.n_rear_side != 0){
//     map->write("octree_painted.ot");
//     std::getchar();
//   }
  
  map->cleanTouchedVoxels();
  return true;
}

float PMVORearSideVoxel::updateWithScan(std::__cxx11::string file_name_scan, std::__cxx11::string file_name_origin)
{
  float r = PMVOctree::updateWithScan(file_name_scan, file_name_origin);
  map->expand();
  return r;
}
