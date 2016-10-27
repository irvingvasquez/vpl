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


#include "coctreevpl.h"



COctreeVPL::COctreeVPL(double resolution): ColorOcTree(resolution)
{
  this->setOccupancyThres(0.52);
  this->setUnknownThres(0.48);
  
  octomap::ColorOcTreeNode::Color colorRed(255,0,0);
  octomap::ColorOcTreeNode::Color colorBlue(0,0,255);
  octomap::ColorOcTreeNode::Color colorYellow(255,255,0);
  octomap::ColorOcTreeNode::Color colorCian(0,255,255);
  octomap::ColorOcTreeNode::Color colorOrange(255,165,0);
  
  colorOccupied = colorBlue;
  colorUnknown = colorOrange;
  colorTouchedOccupied = colorCian;
  colorTouchedUnknown = colorRed;
}



double COctreeVPL::castRayIG(const point3d& origin, const point3d& directionP, point3d& end,bool ignoreUnknownCells, double maxRange)
{
    double ig_sum = 0;
  
    OcTreeKey current_key;
    if ( !coordToKeyChecked(origin, current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return 0;
    }

    ColorOcTreeNode* startingNode = this->search(current_key);
    if (startingNode){
      if (this->isNodeOccupied(startingNode)){
        // Occupied node found at origin 
        // (need to convert from key, since origin does not need to be a voxel center)
        end = this->keyToCoord(current_key);
        return 0;
      }
    } else if(!ignoreUnknownCells){
      OCTOMAP_ERROR_STR("Origin node at " << origin << " for raycasting not found, does the node exist?");
      end = this->keyToCoord(current_key);
      return 0;
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
    	return 0;
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
        return 0;
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
          return 0;
      }
      
      ///  castRay Information Gain !!!!!
      ColorOcTreeNode* currentNode = this->search(current_key);
      if (currentNode){
        if (this->isNodeOccupied(currentNode)) {
	  // verificar si el nodo ya fue tocado
	  return ig_sum;
          
        } else if (this->isNodeUnknown(currentNode)) {
	  if(currentNode->getColor() == colorTouchedUnknown){
	      // el nodo ya fue tocado
	  }else {
	     if(inBBX(end)){
	       
	      currentNode->setColor(colorTouchedUnknown);
	      touchedNodes.push_back(currentNode);
	      double p = currentNode->getOccupancy();
	       
	      ig_sum = ig_sum + (-p * log(p) - (1-p) * log(1-p));
	      //cout << "p " << p << " IG " << ig_sum <<std::endl;
	     }
	  }
	}
	 // otherwise: node is free and valid, raycasting continues
      } else if (!ignoreUnknownCells){ // no node found, this usually means we are in "unknown" areas
        //OCTOMAP_WARNING_STR("Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end);
        return ig_sum;
      }
    } // end while

    return ig_sum;
}



int COctreeVPL::castRayVPL(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknownCells, double maxRange)
{
      /// ----------  see OcTreeBase::computeRayKeys  -----------

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
	  // verificar si el nodo ya fue tocado
	  if(currentNode->getColor() == colorTouchedOccupied){
	      // el nodo ya fue tocado
	      //cout << "voxel repetido" <<std::endl;
	      return VXL_OCCUPIED_TOUCHED;
	  } else {
	      // el nodo no ha sido tocado
	      currentNode->setColor(colorTouchedOccupied);
	      // agregarlo a la lista
	      touchedNodes.push_back(currentNode);
	      return VXL_OCCUPIED;
	  }
          
        } else if (this->isNodeUnknown(currentNode)) {
	  if(currentNode->getColor() == colorTouchedUnknown){
	      // el nodo ya fue tocado
	      //cout << "unknown repetido" <<std::endl;
	      return VXL_UNKNOWN_TOUCHED;
	  }else {
	      currentNode->setColor(colorTouchedUnknown);
	      touchedNodes.push_back(currentNode);
	      return VXL_UNKNOWN;
	  }
	}
        // otherwise: node is free and valid, raycasting continues
      } else if (!ignoreUnknownCells){ // no node found, this usually means we are in "unknown" areas
        //OCTOMAP_WARNING_STR("Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end);
        return VXL_UNKNOWN_NOT_CREATED;
      }
    } // end while

    return VXL_FREE;
}



void COctreeVPL::cleanTouchedVoxels()
{
  std::list< ColorOcTreeNode* >::iterator it;
  
  //cout << touchedNodes.size() << " different touched voxels" <<std::endl;
  
  for(it= touchedNodes.begin(); it!= touchedNodes.end(); it++){
    //(*it)->setColor(255,255,0);
    if(isNodeUnknown(*it)){
      (*it)->setColor(colorUnknown);
    } else if(isNodeOccupied(*it)) {
      (*it)->setColor(colorOccupied);
    }
  }
  
  touchedNodes.clear();
}


/*
bool COctreeVPL::castRay2(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknownCells, double maxRange, int atDeepth) const
{
    /// ----------  see OcTreeBase::computeRayKeys  -----------

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

      ColorOcTreeNode* currentNode = this->search(current_key);
      if (currentNode){
        if (this->isNodeOccupied(currentNode)) {
          done = true;
          break;
        } else if (this->isNodeUnknown(currentNode)) {
	  return false;
	}
        // otherwise: node is free and valid, raycasting continues
      } else if (!ignoreUnknownCells){ // no node found, this usually means we are in "unknown" areas
        //OCTOMAP_WARNING_STR("Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end);
        return false;
      }
    } // end while

    return true;
}*/



/*
bool COctreeVPL::castRay2(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknownCells, double maxRange, int atDeepth) const
{
  
    std::vector< OcTreeKey> llaves;
    
    // ----------  see OcTreeBase::computeRayKeys  -----------
    assert(atDeepth <= tree_depth);

    if (atDeepth == 0)
      atDeepth = tree_depth;

    int diff_deepth = tree_depth - atDeepth;

    ///Initialization phase -------------------------------------------------------
    // Genera una llave para el nivel mas bajo
    OcTreeKey current_key;
    if ( !coordToKeyChecked(origin, current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }
    
    // verifica que el voxel inicial sea libre
    ColorOcTreeNode *startingNode = this->search(current_key, atDeepth);
    if (startingNode){
      if (this->isNodeOccupied(startingNode)){
        // Occupied node found at origin 
        // (need to convert from key, since origin does not need to be a voxel center)
        end = this->keyToCoord(current_key, atDeepth);
        return true;
      }
    } else if(!ignoreUnknownCells){
      OCTOMAP_ERROR_STR("Origin node at " << origin << " for raycasting not found, does the node exist?");
      end = this->keyToCoord(current_key, atDeepth);
      return false;
    }

    /// determinar la direccion
    point3d direction = directionP.normalized();
    bool max_range_set = (maxRange > 0.0);

    int step[3]; 
    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      //compute step direction
      // el step depende de que resolucion utilicemos
      if (direction(i) > 0.0) step[i] = (int) 1 * pow(2,diff_deepth);
      else if (direction(i) < 0.0)   step[i] = -1 * pow(2,diff_deepth);
      else step[i] = 0;

      //compute tMax, tDelta
      if (step[i] != 0) {
        //corner point of voxel (in direction of ray)
        double voxelBorder = this->keyToCoord(current_key[i], atDeepth);
        voxelBorder += double(step[i] * this->resolution * 0.5);

        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = (this->resolution * pow(2,diff_deepth)) / fabs( direction(i) );
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

    //for speedup:
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
        end = this->keyToCoord(current_key, atDeepth);
        return false;
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];


      // generate world coords from key
      end = this->keyToCoord(current_key, atDeepth);
      std::cout << end << std::endl;;
      
      point3d p1 = this->keyToCoord(current_key, atDeepth-1);
      std::cout << p1 << std::endl;;
      
      point3d p2 = this->keyToCoord(current_key, atDeepth-2);
      std::cout << p2 << std::endl;;
      
      point3d p3 = this->keyToCoord(current_key, atDeepth-3);
      std::cout << p3 << std::endl;;
      
      std::cout << tree_depth << std::endl;
      

      // check for maxrange:
      if (max_range_set){
        double dist_from_origin_sq(0.0);
        for (unsigned int j = 0; j < 3; j++) {
          dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
        }
        if (dist_from_origin_sq > maxrange_sq)
          return false;
      }

      ColorOcTreeNode* currentNode = this->search(current_key, atDeepth);
      if (currentNode){
        if (this->isNodeOccupied(currentNode)) {
          done = true;
          break;
        } else {
	  currentNode->setLogOdds(0);
	  llaves.push_back(current_key);
	  currentNode->setColor(255,20,147);
	}
        
      //  otherwise: node is free and valid, raycasting continues
      } else if (!ignoreUnknownCells){ // no node found, this usually means we are in "unknown" areas
        OCTOMAP_WARNING_STR("Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end);
        std::cout << llaves.size() << " spans" << std::endl;
    
	std::vector<OcTreeKey>::iterator node_it;
	int i=0;
	for(node_it = llaves.begin();node_it!= llaves.end(); node_it++){
	  ColorOcTreeNode* currentNode = this->search(*node_it, atDeepth);
	  if (currentNode){
	      currentNode->setLogOdds(0);
	      currentNode->setColor(255-i*5,0,i*5);
	      for(int j = 0; j<8 ; j++){
		if(currentNode->childExists(j)){
		  currentNode->deleteChild(j);
		}
	      }
	    }
	    i++;
	}
	
	this->write("to_show.ot");
	
	    for(node_it = llaves.begin();node_it!= llaves.end(); node_it++){
	  ColorOcTreeNode* currentNode = this->search(*node_it, atDeepth);
	  if (currentNode){
	      currentNode->setLogOdds(-1);
	      currentNode->setColor(255,20,147);
	      for(int j = 0; j<8 ; j++){
		if(currentNode->childExists(j)){
		  currentNode->deleteChild(j);
		}
	      }
	    }
	}
    
        return false;
      }
    } // end while

    std::cout << llaves.size() << " spans" << std::endl;
    
    std::vector<OcTreeKey>::iterator node_it;
    for(node_it = llaves.begin();node_it!= llaves.end(); node_it++){
      ColorOcTreeNode* currentNode = this->search(*node_it, atDeepth);
       if (currentNode){
	  currentNode->setLogOdds(0);
	  currentNode->setColor(255,20,147);
	  for(int j = 0; j<8 ; j++){
	    if(currentNode->childExists(j)){
	      currentNode->deleteChild(j);
	    }
	  }
	}
    }
    
    this->write("to_show.ot");
    
        for(node_it = llaves.begin();node_it!= llaves.end(); node_it++){
      ColorOcTreeNode* currentNode = this->search(*node_it, atDeepth);
       if (currentNode){
	  currentNode->setLogOdds(-1);
	  currentNode->setColor(255,20,147);
	  for(int j = 0; j<8 ; j++){
	    if(currentNode->childExists(j)){
	      currentNode->deleteChild(j);
	    }
	  }
	}
    }
    
    return true;
}
*/



int COctreeVPL::castRayVPLHierarchical(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknownCells, double maxRange, int atDeepth) 
{
    int return_value = VXL_FREE;
    
    // ----------  see OcTreeBase::computeRayKeys  -----------
    assert(atDeepth <= tree_depth);

    if (atDeepth == 0)
      atDeepth = tree_depth;

    int diff_deepth = tree_depth - atDeepth;

    ///Initialization phase -------------------------------------------------------
    // Genera una llave para el nivel mas bajo
    OcTreeKey current_key;
    if ( !coordToKeyChecked(origin, current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }
    
    // verifica que el voxel inicial sea libre
    ColorOcTreeNode *startingNode = this->search(current_key, atDeepth);
    if (startingNode){
      if (this->isNodeOccupied(startingNode)){
        // Occupied node found at origin 
        // (need to convert from key, since origin does not need to be a voxel center)
        end = this->keyToCoord(current_key, atDeepth);
        return true;
      }
    } else if(!ignoreUnknownCells){
      OCTOMAP_ERROR_STR("Origin node at " << origin << " for raycasting not found, does the node exist?");
      end = this->keyToCoord(current_key, atDeepth);
      return false;
    }

    /// determinar la direccion
    point3d direction = directionP.normalized();
    bool max_range_set = (maxRange > 0.0);

    int step[3]; 
    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      //compute step direction
      // el step depende de que resolucion utilicemos
      if (direction(i) > 0.0) step[i] = (int) 1 * pow(2,diff_deepth);
      else if (direction(i) < 0.0)   step[i] = -1 * pow(2,diff_deepth);
      else step[i] = 0;

      //compute tMax, tDelta
      if (step[i] != 0) {
        //corner point of voxel (in direction of ray)
        double voxelBorder = this->keyToCoord(current_key[i], atDeepth);
        voxelBorder += double(step[i] * this->resolution * 0.5);

        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = (this->resolution * pow(2,diff_deepth)) / fabs( direction(i) );
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

    //for speedup:
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
        end = this->keyToCoord(current_key, atDeepth);
        return false;
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];


      // generate world coords from key
      end = this->keyToCoord(current_key, atDeepth);      

      // check for maxrange:
      if (max_range_set){
        double dist_from_origin_sq(0.0);
        for (unsigned int j = 0; j < 3; j++) {
          dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
        }
        if (dist_from_origin_sq > maxrange_sq)
          return false;
      }

      ColorOcTreeNode* currentNode = this->search(current_key, atDeepth);
      if (currentNode){
	if (this->isNodeOccupied(currentNode)) {
	  // verificar si el nodo ya fue tocado
	  if(currentNode->getColor() == colorTouchedOccupied){
	      // el nodo ya fue tocado
	      //cout << "voxel repetido" <<std::endl;
	      return_value = VXL_OCCUPIED_TOUCHED;
	      done = true;
	  } else {
	      // el nodo no ha sido tocado
	      currentNode->setColor(colorTouchedOccupied);
	      // agregarlo a la lista
	      touchedNodes.push_back(currentNode);
	      return_value = VXL_OCCUPIED;
	      done = true;
	  }
          
        } else if (this->isNodeUnknown(currentNode)) {
	  if(currentNode->getColor() == colorTouchedUnknown){
	      // el nodo ya fue tocado
	      //cout << "unknown repetido" <<std::endl;
	      return_value = VXL_UNKNOWN_TOUCHED;
	      done = true;
	  }else {
	      currentNode->setColor(colorTouchedUnknown);
	      touchedNodes.push_back(currentNode);
	      return_value = VXL_UNKNOWN;
	      done = true;
	  }
	}
	// else {
	// otherwise: node is free and valid, raycasting continues
	// }
      
      } else if (!ignoreUnknownCells){ // no node found, this usually means we are in "unknown" areas
        
	//OCTOMAP_WARNING_STR("Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end);
        return_value = VXL_UNKNOWN_NOT_CREATED;
	done = true;
      }
    } // end while
    
    return return_value;
}




///// This particular implementation paints the spanned voxels
/*
int COctreeVPL::castRayVPLHierarchical(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknownCells, double maxRange, int atDeepth) 
{
    std::vector< OcTreeKey> llaves;
    int return_value = VXL_FREE;
    
    // ----------  see OcTreeBase::computeRayKeys  -----------
    assert(atDeepth <= tree_depth);

    if (atDeepth == 0)
      atDeepth = tree_depth;

    int diff_deepth = tree_depth - atDeepth;

    ///Initialization phase -------------------------------------------------------
    // Genera una llave para el nivel mas bajo
    OcTreeKey current_key;
    if ( !coordToKeyChecked(origin, current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }
    
    // verifica que el voxel inicial sea libre
    ColorOcTreeNode *startingNode = this->search(current_key, atDeepth);
    if (startingNode){
      if (this->isNodeOccupied(startingNode)){
        // Occupied node found at origin 
        // (need to convert from key, since origin does not need to be a voxel center)
        end = this->keyToCoord(current_key, atDeepth);
        return true;
      }
    } else if(!ignoreUnknownCells){
      OCTOMAP_ERROR_STR("Origin node at " << origin << " for raycasting not found, does the node exist?");
      end = this->keyToCoord(current_key, atDeepth);
      return false;
    }

    /// determinar la direccion
    point3d direction = directionP.normalized();
    bool max_range_set = (maxRange > 0.0);

    int step[3]; 
    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      //compute step direction
      // el step depende de que resolucion utilicemos
      if (direction(i) > 0.0) step[i] = (int) 1 * pow(2,diff_deepth);
      else if (direction(i) < 0.0)   step[i] = -1 * pow(2,diff_deepth);
      else step[i] = 0;

      //compute tMax, tDelta
      if (step[i] != 0) {
        //corner point of voxel (in direction of ray)
        double voxelBorder = this->keyToCoord(current_key[i], atDeepth);
        voxelBorder += double(step[i] * this->resolution * 0.5);

        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = (this->resolution * pow(2,diff_deepth)) / fabs( direction(i) );
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

    //for speedup:
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
        end = this->keyToCoord(current_key, atDeepth);
        return false;
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];


      // generate world coords from key
      end = this->keyToCoord(current_key, atDeepth);
//       std::cout << end << std::endl;;
//       
//       point3d p1 = this->keyToCoord(current_key, atDeepth-1);
//       std::cout << p1 << std::endl;;
//       
//       point3d p2 = this->keyToCoord(current_key, atDeepth-2);
//       std::cout << p2 << std::endl;;
//       
//       point3d p3 = this->keyToCoord(current_key, atDeepth-3);
//       std::cout << p3 << std::endl;;
//       
//       std::cout << tree_depth << std::endl;
      

      // check for maxrange:
      if (max_range_set){
        double dist_from_origin_sq(0.0);
        for (unsigned int j = 0; j < 3; j++) {
          dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
        }
        if (dist_from_origin_sq > maxrange_sq)
          return false;
      }

      ColorOcTreeNode* currentNode = this->search(current_key, atDeepth);
      if (currentNode){
	if (this->isNodeOccupied(currentNode)) {
	  // verificar si el nodo ya fue tocado
	  if(currentNode->getColor() == colorTouchedOccupied){
	      // el nodo ya fue tocado
	      //cout << "voxel repetido" <<std::endl;
	      return_value = VXL_OCCUPIED_TOUCHED;
	      done = true;
	  } else {
	      // el nodo no ha sido tocado
	      currentNode->setColor(colorTouchedOccupied);
	      // agregarlo a la lista
	      touchedNodes.push_back(currentNode);
	      return_value = VXL_OCCUPIED;
	      done = true;
	  }
          
        } else if (this->isNodeUnknown(currentNode)) {
	  if(currentNode->getColor() == colorTouchedUnknown){
	      // el nodo ya fue tocado
	      //cout << "unknown repetido" <<std::endl;
	      return_value = VXL_UNKNOWN_TOUCHED;
	      done = true;
	  }else {
	      currentNode->setColor(colorTouchedUnknown);
	      touchedNodes.push_back(currentNode);
	      return_value = VXL_UNKNOWN;
	      done = true;
	  }
	} else {
	  // guardar para intar remover despues
	  currentNode->setLogOdds(0);
	  llaves.push_back(current_key);
	  currentNode->setColor(255,20,147);
	}
        
      //  otherwise: node is free and valid, raycasting continues
      } else if (!ignoreUnknownCells){ // no node found, this usually means we are in "unknown" areas
        
	OCTOMAP_WARNING_STR("Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end);
        
	std::cout << llaves.size() << " spans" << std::endl;
    
	std::vector<OcTreeKey>::iterator node_it;
	int i=0;
	for(node_it = llaves.begin();node_it!= llaves.end(); node_it++){
	  ColorOcTreeNode* currentNode = this->search(*node_it, atDeepth);
	  if (currentNode){
	      currentNode->setLogOdds(0);
	      currentNode->setColor(255-i*5,0,i*5);
	      for(int j = 0; j<8 ; j++){
		if(currentNode->childExists(j)){
		  currentNode->deleteChild(j);
		}
	      }
	    }
	    i++;
	}
	
	this->write("to_show.ot");
	
	for(node_it = llaves.begin();node_it!= llaves.end(); node_it++){
	  ColorOcTreeNode* currentNode = this->search(*node_it, atDeepth);
	  if (currentNode){
	      currentNode->setLogOdds(-1);
	      currentNode->setColor(255,20,147);
	      for(int j = 0; j<8 ; j++){
		if(currentNode->childExists(j)){
		  currentNode->deleteChild(j);
		}
	      }
	  }
	}
    
        return_value = VXL_UNKNOWN_NOT_CREATED;
	done = true;
      }
    } // end while

    std::cout << llaves.size() << " spans" << std::endl;
    
    std::vector<OcTreeKey>::iterator node_it;
    for(node_it = llaves.begin();node_it!= llaves.end(); node_it++){
      ColorOcTreeNode* currentNode = this->search(*node_it, atDeepth);
       if (currentNode){
	  currentNode->setLogOdds(0);
	  currentNode->setColor(255,20,147);
	  for(int j = 0; j<8 ; j++){
	    if(currentNode->childExists(j)){
	      currentNode->deleteChild(j);
	    }
	  }
	}
    }
    
    this->write("to_show.ot");
    
        for(node_it = llaves.begin();node_it!= llaves.end(); node_it++){
      ColorOcTreeNode* currentNode = this->search(*node_it, atDeepth);
       if (currentNode){
	  currentNode->setLogOdds(-1);
	  currentNode->setColor(255,20,147);
	  for(int j = 0; j<8 ; j++){
	    if(currentNode->childExists(j)){
	      currentNode->deleteChild(j);
	    }
	  }
	}
    }
    
    return return_value;
}
*/

void COctreeVPL::getUnknownVoxels(point3d_list& node_centers, std::vector< double >& sizes)
{
  int max_depth = this->tree_depth;

  for( COctreeVPL::leaf_iterator it = this->begin_leafs(max_depth), end=this->end_leafs(); it!= end; ++it)
  {
      if(this->isNodeUnknown(*it)){
	point3d p = it.getCoordinate();
	if(this->inBBX(p)){
	  node_centers.push_back(p);
	  sizes.push_back(it.getSize());
	}
      }
  }
}



void COctreeVPL::getUnknownCentersAll(point3d_list& node_centers)
{
  this->getUnknownLeafCenters(node_centers, this->bbx_min, this->bbx_max);
  
  int max_depth = this->tree_depth;

  for( COctreeVPL::leaf_iterator it = this->begin(max_depth), end=this->end(); it!= end; ++it)
  {
      if(this->isNodeUnknown(*it)){
	point3d p = it.getCoordinate();
	if(this->inBBX(p)){
	  node_centers.push_back(p);
	}
      }
  }
 

 
 
  
   
//   std::list<OcTreeVolume> list_v;
//   
//   std::list<OcTreeVolume>::const_iterator it;
//   
//   this->getFreespace(list_v);
//   
//   for(it = list_v.begin(); it != list_v.end(); it++){
//     point3d p = it->first;
//     ColorOcTreeNode *node = this->search(p);
//     if(isNodeUnknown(node)){
//       node_centers.push_back(p);
//     }
//   }
}

