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


#include "pmvoctreeig.h"

PMVOctreeIG::PMVOctreeIG()
{

}


int PMVOctreeIG::evaluateView(ViewStructure& v)
{
  if(!poitsToTheObject(v)){
    //cout << "Sorry no points :(" << std::endl;
    return UNFEASIBLE_VIEW;
  }
  
  EvaluationResult result,countingIG;
  double frustumIG;
  
  // first, perform a ray tracing to calculate feasibility
  bool valid_result = rayTracingHTM(v.HTM, result);
  
  // Information Gain calculation
  frustumIG = rayTracingHTMIG(v.HTM, countingIG);
  
  if(valid_result){
    v.n_unknown = result.n_unknown;
    v.n_occupied = result.n_occupied;
    v.n_occplane = result.n_unknown;
      
    // See at least an unknowm voxel?
//    if(result.n_unknown == 0 && result.n_unknown_scene ==0){
//      v.eval= 0.0;
//      return UNFEASIBLE_VIEW;
//    } else {	
	// check for registration constraint
	if(registrationConstraint(result)){
	    //view evaluation with frustum information gain
	    v.eval = frustumIG;
	    return FEASIBLE_VIEW;
	} else {
	   v.eval = 0.0;
	   return UNFEASIBLE_VIEW;
	}
   //}
  } 
  
  return UNFEASIBLE_VIEW;
}


double PMVOctreeIG::rayTracingHTMIG(boost::numeric::ublas::matrix< double > m, EvaluationResult& result)
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
  
  //std::cout << "Nodes before expansion:" <<  map->getNumLeafNodes() << std::endl;
  //map->expandUnknownVoxels();
  //map->updateInnerOccupancy();
  //map->expand();
  //std::cout << "Nodes after expansion:" << map->getNumLeafNodes() << std::endl;
  
  double ig_sum = 0;
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
     ig_sum = ig_sum + map->castRayIG(origin, *direction, touched_position);
     
     delete direction;
  }
  result.evaluation = ig_sum;
  
//  std::cout << "RT. Occ:" << result.n_occupied << " Occ_sce:" << result.n_occupied_scene 
// 		    << " Unk:" << result.n_unknown <<  " Unk_sce:" << result.n_unknown_scene 
// 		    << " lost:" << result.n_lost << std::endl;

  //map->write("octree_painted.ot");
  map->cleanTouchedVoxels();
  return ig_sum;
}


float PMVOctreeIG::updateWithScan(std::string file_name_scan, std::string file_name_origin)
{
  float r = PMVOctree::updateWithScan(file_name_scan, file_name_origin);
  map->expand();
  return r;
}
