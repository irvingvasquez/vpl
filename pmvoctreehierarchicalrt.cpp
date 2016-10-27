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


#include "pmvoctreehierarchicalrt.h"

PMVOctreeHierarchicalRT::PMVOctreeHierarchicalRT():PMVOctree()
{
  abstractionLevel = 0;
}

bool PMVOctreeHierarchicalRT::init()
{
  PMVOctree::init();
  
  std::cout << "---------------- Octree HTR -------------------" << std::endl;
  
  string config_file(configFolder);
  config_file.append("/");
  config_file.append("partialModelConfig.ini");
  
  dictionary * ini_file;
    
  ini_file = iniparser_load(config_file.c_str());
  if (ini_file ==NULL ) {
     fprintf(stderr, "cannot parse file: %s\n", config_file.c_str());
     return false ;
  }
  //  iniparser_dump(ini_file, stderr);
  abstractionLevel = iniparser_getint(ini_file,"multiresOctree:abstractionLevel", 1);
  
  hasAbstractionLevel = true;
    
  std::cout << "-------------------------------------------" << std::endl;
}

long int PMVOctreeHierarchicalRT::readRays(std::string file_address)
{
   PMVOctree::readRays(file_address);
   
   rtGenerateRaysTree(file_address);
}

void PMVOctreeHierarchicalRT::rtGenerateRaysTree(std::string file_rays)
{
  int l=0;
  RaysVector R_l;
  RayNodePtr_List children_ptr_list;
  RayNodePtr_List parent_ptr_list;
  string name_for_level;
  
  std::cout << "Generating rays tree" << std::endl;
  std::cout << "Abstraction level: " << l << std::endl;
  
  rtReadRays(file_rays, R_l);
  rtAddLeafsToTree(R_l, children_ptr_list);
  l++;
  
  
  while(l <= abstractionLevel){
    std::cout << "	Abstraction level: " << l << std::endl;
    // generate nodes
    std::cout << "	Generate nodes." << std::endl;
    name_for_level = rtGetNameForRaysFile(file_rays, l);
    rtReadRays(name_for_level, R_l);
    rtAddNodesToTree(R_l,parent_ptr_list);
    
    //link nodes
    std::cout << "	-Link nodes." << std::endl;
    rtLinkChildrenWithParents(children_ptr_list, parent_ptr_list);
    
    //update references
    children_ptr_list.clear();
    children_ptr_list.insert(children_ptr_list.begin(), parent_ptr_list.begin(), parent_ptr_list.end());
    l++;
    
  }
  
  //add root
  rtAddRoot(children_ptr_list);
  rtHasRoot = true;
  
 // rtTraverseWithInfo();
  std::cout << "Rays tree generated" << std::endl;
}



void PMVOctreeHierarchicalRT::setAbstractionLevel(int level)
{
  if(level < 0){
    std::cout << "ERROR: Abstraction level must be higher than 0" << std::endl;
    exit(0);
  }
  
  abstractionLevel = level;
  hasAbstractionLevel = true;
}


bool PMVOctreeHierarchicalRT::isReady()
{
  if(!hasAbstractionLevel)
    return false;
  
  if(!hasResolution)
    return false;
  
  return true;
}


void PMVOctreeHierarchicalRT::rtAddLeafsToTree(RaysVector& R, std::list< pmRayNode* >& leafsPtrList)
{
  rtRaysTree.clear();
  leafsPtrList.clear();
  std::list<pmRayNode>::pointer node_ptr;
  
  std::vector< boost::numeric::ublas::matrix< double > >::iterator it_rays;
  for(it_rays = R.begin(); it_rays!=R.end(); it_rays++){
    pmRayNode node(*it_rays);  
    node.nLeafs = 1; // it is 1 because byitself is a leaf if there is no more links
    rtRaysTree.push_back(node);
    node_ptr = &rtRaysTree.back();
    leafsPtrList.push_back(node_ptr);
  }
}


void PMVOctreeHierarchicalRT::rtAddNodesToTree(RaysVector& R, RayNodePtr_List& node_ptr_list)
{
   node_ptr_list.clear();
  std::list<pmRayNode>::pointer node_ptr;
  
  RaysVector::iterator it_rays;
  
  for(it_rays = R.begin(); it_rays!=R.end(); it_rays++){
    pmRayNode node(*it_rays);
    rtRaysTree.push_front(node);
    node_ptr = &rtRaysTree.front();
    node_ptr_list.push_back(node_ptr);
  }
}

void PMVOctreeHierarchicalRT::rtAddRoot(RayNodePtr_List& node_ptr_list)
{
  pmRayNode root(0,0,0);
  std::list<pmRayNode>::pointer root_ptr;
  rtRaysTree.push_front(root);
  root_ptr = &rtRaysTree.front();
  
  RayNodePtr_List::iterator node_it;
  for(node_it = node_ptr_list.begin(); node_it != node_ptr_list.end(); node_it++){
    root_ptr->children_count ++;
    root_ptr->childrenPtrList.push_back( (*node_it) );
    root_ptr->nLeafs = root_ptr->nLeafs + (*node_it)->nLeafs;
    (*node_it)->parent = root_ptr;
  }
  
  rtRootPtr = root_ptr;
}


string PMVOctreeHierarchicalRT::rtGetNameForRaysFile(std::string filename_cero, int level)
{
  string name;
  string extension;
  string name_for_level;
  
  size_t found;
  found = filename_cero.find_last_of("0.dat");
  name.assign(filename_cero.substr(0,found-4));
  extension.assign(".dat");
  
  ostringstream oss;
  oss << name << level << extension;
  name_for_level.assign(oss.str());
  
  return name_for_level;
}


pmRayNode* PMVOctreeHierarchicalRT::rtGetNearestRay(pmRayNode* node_ptr, RayNodePtr_List node_ptr_list)
{
  //cout << "Get nearest ray." << std::endl;
  RayNodePtr_List::iterator list_it;
  //cout << "Node: " << node_ptr->ray.x() << " " << node_ptr->ray.y() << " " << node_ptr->ray.z() << std::endl;
  
  pmRayNode* nn = NULL;
  double distance ;
  //double angle;
  double min_dist= 32000;
  for(list_it = node_ptr_list.begin(); list_it!= node_ptr_list.end(); list_it++ ){
    // the distance iw the angle between vectors
    distance = node_ptr->angleTo(*(*list_it));
    if(distance < 0)
      distance = distance * (-1);
    
    //cout << "Parent: " << (*list_it)->ray.x() << " " << (*list_it)->ray.y() << " " << (*list_it)->ray.z() << "\t" << "d: " << distance <<std::endl;
    if(distance < min_dist)
    {
      nn = *list_it;
      min_dist = distance;
    }
  }
  
  //cout << "Selected parent: " << nn->ray.x() << " " << nn->ray.y() << " " << nn->ray.z() << std::endl << std::endl;
  return nn;
}


void PMVOctreeHierarchicalRT::rtLinkChildrenWithParents(RayNodePtr_List children, RayNodePtr_List parents)
{
  RayNodePtr_List::iterator children_it;
  RayNodePtr_List::iterator parents_it;
  std::list<pmRayNode>::pointer np; // nearest parent
  int child_number = 0;
  
  for(children_it = children.begin(); children_it != children.end(); children_it ++){
    //cout << "child: " << child_number << std::endl;
    child_number++;
    
    np = rtGetNearestRay(*children_it, parents);
    
    //link
    (*children_it)->parent = np;
    np->childrenPtrList.push_back((*children_it));
    np->children_count++;
    np->nLeafs = np->nLeafs + (*children_it)->nLeafs;

  }
}


bool PMVOctreeHierarchicalRT::rtIsLeaf(rtPointerToElementType element_ptr)
{
  if(element_ptr->childrenPtrList.size() == 0)
    return true;
  else
    return false;
}


bool PMVOctreeHierarchicalRT::rtReadRays(std::string file_name, std::vector< boost::numeric::ublas::matrix< double > >& R)
{
  double x_r, y_r, z_r;
  long int n_rays;
  long int readed_rays = 0;
  R.clear();
  boost::numeric::ublas::matrix<double> ray(4,1);
  
  std::cout << "Reading file: " << file_name << std::endl;
  
  ifstream file(file_name.c_str());
  if(file.is_open()){
    //file >> n_rays;
    file >> x_r;
    
    while(file.good()){
        
	file >> y_r;
	file >> z_r;
	ray(0,0) = x_r;
	ray(1,0) = y_r;
	ray(2,0) = z_r;
	ray(3,0) = 1;
	
	R.push_back(ray);
	
	readed_rays ++;
	
	file >> x_r;
    }
    file.close();
  } else {
    std::cout << "Unable to open file " << file_name.c_str() << std::endl;
    getchar();
    return false;
  }
  
  //cout << "done." << std::endl;
  std::cout << "Readed rays: "<< readed_rays << std::endl;
  return true;
}


void PMVOctreeHierarchicalRT::rtTraverseWithInfo()
{
   std::list<pmRayNode>::iterator node_it;
  
  int i;
  int counter = 0;
   
  std::cout << "-------------------------------------\n" << std::endl;
  std::cout << "	rtRaysTree\n";
  std::cout << "-------------------------------------\n" << std::endl;
  
  for(node_it = rtRaysTree.begin(); node_it != rtRaysTree.end(); node_it++){
    std::cout.precision(3);
    std::cout << fixed << node_it->ray.x() << " " << fixed << node_it->ray.y() << " " << fixed << node_it->ray.z() << "\t";
    std::cout << "Children count: " << node_it->children_count << "\t";
    std::cout << "Leafs: " << node_it->nLeafs << std::endl;
    counter ++;
    
    if (counter == 100){
      std::cout << "press any key to continue..." << std::endl;
      getchar();
      counter = 0;
    }
  }
  std::cout << "-------------------------------------" << std::endl;
}



int PMVOctreeHierarchicalRT::evaluateView(ViewStructure& v)
{   
  if(!poitsToTheObject(v)){
    //cout << "Sorry no points :(" << std::endl;
    return UNFEASIBLE_VIEW;
  }
  
  bool valid_result;
  EvaluationResult result;
    
  if(!rtHasRoot){
    std::cout << "RaysTree has not been created." << std::endl;
    return UNFEASIBLE_VIEW;
  }
  
  if(rtRootPtr->children_count == 0){
    std::cout << "RaysTree Root has no rays." << std::endl;
    return UNFEASIBLE_VIEW;
  }
  
  int depth = map->getTreeDepth() - abstractionLevel;
  
  result.clear();
  valid_result = hierarchicalRayTracing(v.HTM, this->rtRootPtr, depth, result);
    
  if(valid_result){           
      /// Evaluate the result of the raytracing
      v.n_unmark = result.n_unmark;
      v.n_occupied = result.n_occupied;
      
      if(v.n_unmark < minUnknown){
	return UNFEASIBLE_VIEW;
      }
      
      if( this->utilityFunction->evaluate(result) == FEASIBLE_VIEW){
	v.eval = result.evaluation;
	return FEASIBLE_VIEW;
      }
  }
  
  return UNFEASIBLE_VIEW;
}



void PMVOctreeHierarchicalRT::evaluateCandidateViews()
{
  std::list<ViewStructure>::iterator it_v;
  int removed_views=0;
  clock_t start;
  double diff;
  bool valid_result;
  EvaluationResult result;
  int i =0;
  
  std::cout << "Evaluation candidate views with HRT octree." << std::endl;
  std::cout << "Rays to be traced:" << rtRootPtr->nLeafs << std::endl;
  
  if(!rtHasRoot){
    std::cout << "RaysTree has not been created." << std::endl;
    return;
  }
  
  if(rtRootPtr->children_count == 0){
    std::cout << "RaysTree Root has no rays." << std::endl;
    return;
  }
  
  stopCriteria = true;
  evals.clear();
  int depth = map->getTreeDepth() - abstractionLevel;
  it_v = candidateViews.begin();
  while(it_v != candidateViews.end()){
    // TODO collision checking
    i++; //display

    result.clear();
    //printVector(it_v->q);
    
    start = clock();
    
    valid_result = hierarchicalRayTracing(it_v->HTM, this->rtRootPtr, depth, result);
    diff = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;
    
    result.computation_time = diff;
    evals.push_back(result);
    
    if(valid_result){       
      
      /// Evaluate the result of the raytracing
      if( this->utilityFunction->evaluate(result) == UNFEASIBLE_VIEW){
	it_v = candidateViews.erase(it_v);
	cout << "Unfeasible view" << std::endl;
	removed_views ++;
      } else {
	if(result.n_unmark > minUnknown)
	  stopCriteria = false;
	
	cout << "Evaluation " << i << ": " << result.evaluation << std::endl;
	it_v->eval = result.evaluation;
	it_v ++;
      }
    }
    else {
      
      it_v = candidateViews.erase(it_v);
      removed_views ++;
    }
  }
  
  std::cout << "Removed views: " << removed_views << std::endl;
}




// bool PMVOctreeHierarchicalRT::hierarchicalRayTracing(BoostMatrix m, pmRayNode* root_ray_ptr, int depth, EvaluationResult& result)
// {  
//   rtChildrenPtrListType::iterator r_it;
//   
//   // Para cada hijo del nodo raiz...
//   for(r_it = root_ray_ptr->childrenPtrList.begin(); r_it!= root_ray_ptr->childrenPtrList.end(); r_it++){
//     // rotarlo por la matriz de la vista
//     switch( castRayAtDepth(m, *r_it, depth)){
//       case OCCUPIED:
// 	if(depth == map->getTreeDepth() || rtIsLeaf(*r_it)){
// 	  result.n_occupied ++;
// 	}
// 	else {
// 	  EvaluationResult aux_result;
// 	  recursiveHRayTracing(m, *r_it, depth+1, aux_result);
// 	  //cout << "After recursive: occ: " << aux_n_occupied << " un:" << aux_n_unmark << " un_sc:" << aux_n_unmark_scene << std::endl; 
// 	  result.addVoxelAmouts(aux_result);
// 	}
// 	break;
// 	
//       case OCCUPIED_SCENE:
// 	result.n_occupied_scene += (*r_it)->nLeafs;
// 	break;
// 	
//       case UNMARK:
// 	result.n_unmark += (*r_it)->nLeafs;
// 	break;
//       
//       case UNMARK_SCENE:
// 	result.n_unmark_scene += (*r_it)->nLeafs;
// 	break;
// 	
//       case RAY_LOST:
// 	result.n_lost += (*r_it)->nLeafs;
// 	break;
// 	
//       case INVALID:
// 	return false;
// 	//break;
//     }
//   }
//   
// //   std::cout << "RT. Occ:" << result.n_occupied << " Occ_sce:" << result.n_occupied_scene 
// //    		    << " Unk:" << result.n_unmark <<  " Unk_sce:" << result.n_unmark_scene 
// //    		    << " lost:" << result.n_lost << std::endl;
// 
//   map->write("painted.ot");
//   //getchar();
//   
//   map->cleanTouchedVoxels();
// 
//   return true;
// }



bool PMVOctreeHierarchicalRT::hierarchicalRayTracing(BoostMatrix m, pmRayNode* root_ray_ptr, int depth, EvaluationResult& result)
{  
  rtChildrenPtrListType::iterator r_it;
  
  // Para cada hijo del nodo raiz...
  for(r_it = root_ray_ptr->childrenPtrList.begin(); r_it!= root_ray_ptr->childrenPtrList.end(); r_it++){
    // rotarlo por la matriz de la vista
    int answer = castRayAtDepth(m, *r_it, depth);
    
    if(answer == VXL_OCCUPIED || answer == VXL_OCCUPIED_TOUCHED){
	if(depth == map->getTreeDepth() || rtIsLeaf(*r_it)){
	  if(answer == VXL_OCCUPIED)
	    result.n_occupied ++;
	} else {
	  EvaluationResult aux_result;
	  recursiveHRayTracing(m, *r_it, depth+1, aux_result);
	  //cout << "After recursive: occ: " << aux_n_occupied << " un:" << aux_n_unmark << " un_sc:" << aux_n_unmark_scene << std::endl; 
	  result.addVoxelAmouts(aux_result);
	}
    } else {
      if (answer == VXL_UNKNOWN || answer == VXL_UNKNOWN_TOUCHED){
	if(depth == map->getTreeDepth() || rtIsLeaf(*r_it)){
	  if(answer == VXL_UNKNOWN)
	    result.n_unmark ++;
	}
	else {
	  EvaluationResult aux_result;
	  recursiveHRayTracing(m, *r_it, depth+1, aux_result);
	  //cout << "After recursive: occ: " << aux_n_occupied << " un:" << aux_n_unmark << " un_sc:" << aux_n_unmark_scene << std::endl; 
	  result.addVoxelAmouts(aux_result);
	}
      }
    }
  }
    
  /*  switch( castRayAtDepth(m, *r_it, depth)){
      case OCCUPIED:
	if(depth == map->getTreeDepth() || rtIsLeaf(*r_it)){
	  result.n_occupied ++;
	}
	else {
	  EvaluationResult aux_result;
	  recursiveHRayTracing(m, *r_it, depth+1, aux_result);
	  //cout << "After recursive: occ: " << aux_n_occupied << " un:" << aux_n_unmark << " un_sc:" << aux_n_unmark_scene << std::endl; 
	  result.addVoxelAmouts(aux_result);
	}
	break;
	
      case OCCUPIED_SCENE:
	result.n_occupied_scene += (*r_it)->nLeafs;
	break;
	
      case UNMARK:
	if(depth == map->getTreeDepth() || rtIsLeaf(*r_it)){
	  result.n_unmark ++;
	}
	else {
	  EvaluationResult aux_result;
	  recursiveHRayTracing(m, *r_it, depth+1, aux_result);
	  //cout << "After recursive: occ: " << aux_n_occupied << " un:" << aux_n_unmark << " un_sc:" << aux_n_unmark_scene << std::endl; 
	  result.addVoxelAmouts(aux_result);
	}
	break;
      
      case UNMARK_SCENE:
	result.n_unmark_scene += (*r_it)->nLeafs;
	break;
	
      case RAY_LOST:
	result.n_lost += (*r_it)->nLeafs;
	break;
	
      case INVALID:
	return false;
	//break;
    }
  }
 */ 
//   std::cout << "RT. Occ:" << result.n_occupied << " Occ_sce:" << result.n_occupied_scene 
//    		    << " Unk:" << result.n_unmark <<  " Unk_sce:" << result.n_unmark_scene 
//    		    << " lost:" << result.n_lost << std::endl;

  //map->write("painted.ot");
  //getchar();
  
  map->cleanTouchedVoxels();

  return true;
}



bool PMVOctreeHierarchicalRT::recursiveHRayTracing(BoostMatrix m, pmRayNode* root_ray_ptr, int depth, EvaluationResult& result)
{
  rtChildrenPtrListType::iterator r_it;
  
  for(r_it = root_ray_ptr->childrenPtrList.begin(); r_it!= root_ray_ptr->childrenPtrList.end(); r_it++){
    // rotarlo por la matriz de la vista
    int answer = castRayAtDepth(m, *r_it, depth);
    
    if(answer == VXL_OCCUPIED || answer == VXL_OCCUPIED_TOUCHED){
	if(depth == map->getTreeDepth() || rtIsLeaf(*r_it)){
	  if(answer == VXL_OCCUPIED)
	    result.n_occupied ++;
	} else {
	  EvaluationResult aux_result;
	  recursiveHRayTracing(m, *r_it, depth+1, aux_result);
	  //cout << "After recursive: occ: " << aux_n_occupied << " un:" << aux_n_unmark << " un_sc:" << aux_n_unmark_scene << std::endl; 
	  result.addVoxelAmouts(aux_result);
	}
    } else {
      if (answer == VXL_UNKNOWN || answer == VXL_UNKNOWN_TOUCHED){
	if(depth == map->getTreeDepth() || rtIsLeaf(*r_it)){
	  if(answer == VXL_UNKNOWN)
	    result.n_unmark ++;
	}
	else {
	  EvaluationResult aux_result;
	  recursiveHRayTracing(m, *r_it, depth+1, aux_result);
	  //cout << "After recursive: occ: " << aux_n_occupied << " un:" << aux_n_unmark << " un_sc:" << aux_n_unmark_scene << std::endl; 
	  result.addVoxelAmouts(aux_result);
	}
      }
    }
  }
  
  return true;
}



bool PMVOctreeHierarchicalRT::isInCapsule(point3d point, float resolution)
{
  if( (point.x() >=  (x_cap_1-resolution)) && (point.x() <= (x_cap_2+resolution)) )
    if (point.y() >=  (y_cap_1-resolution) && point.y() <= (y_cap_2+resolution) )
      if ( point.z() >= (z_cap_1-resolution) && point.z() <= (z_cap_2+resolution) )
	return true;
  
//  std::cout << point.x() << " " << point.y() << " " << point.z() << std::endl;
//  std::cout << "false" << std::endl;
  return false;
}


int PMVOctreeHierarchicalRT::castRayAtDepth(BoostMatrix m, rtPointerToElementType ray_node, int depth)
{
  try {
    //Dynamic casting
    COctreeVPL * octree_hrt;
    octree_hrt = dynamic_cast<COctreeVPL*>(map);
    if (octree_hrt==0) std::cout << "Null pointer on type-cast" << std::endl;
  
    int diff_depth = octree_hrt->getTreeDepth() - depth;
    double resolution_at_depth = octree_hrt->getResolution() * pow(2,diff_depth);
    int return_value;  
    
    double i_ray,j_ray,k_ray;
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
    origin_node =  octree_hrt->search(origin);
    if(origin_node == NULL){
      //cout << "Origin not found. It could be in a unknown part. " << std::endl;
      //cout << origin.x() << " " << origin.y() << " " << origin.z() << std::endl;
      return INVALID;
    }

    /// The ray is rotated and traslated by the rotation matrix
    ray = ray_node->ray_matrix;
    rotated_ray = boost::numeric::ublas::prod(m, ray);
    //cout << "rotated_ray: " << rotated_ray(0,0) << " " <<  rotated_ray(1,0) << " " << rotated_ray(2,0) << std::endl;
    
    //compute direction from position to the rotated ray. This is necesary because the rotated ray was also trasladated by the rotation matrix
    computeRayFromPointToPoint(rotated_origin , rotated_ray, i_ray, j_ray, k_ray);

    direction = new octomap::point3d(i_ray, j_ray, k_ray);
    //cout << "direction: " << direction->x() << " " << direction->y() << " " << direction->z() << std::endl;
    
    // if the casted ray returns true a occupied voxel was hit
    // if (map->castRay(origin, *direction, touched_position, false, -1.0)){
    int answer = octree_hrt->castRayVPLHierarchical(origin, *direction, touched_position, false, -1.0, depth);
    if (answer == VXL_OCCUPIED || answer == VXL_OCCUPIED_TOUCHED){
	if(isInDOV(origin,touched_position)) {
	  if(isInCapsule(touched_position, resolution_at_depth)){	  
	    return_value = answer;
	  } else {
	    return_value = OCCUPIED_SCENE;
	  }
	} else {
	  return_value = RAY_LOST;
	}
    } else if(answer == VXL_UNKNOWN || answer == VXL_UNKNOWN_TOUCHED){
	if(isInDOV(origin, touched_position)){
	  if(isInCapsule(touched_position, resolution_at_depth)){
	    return_value = answer;
	  } else {
	    return_value = UNMARK_SCENE;
	  }
	} else {
	  return_value = RAY_LOST;
	}
    } else {
      return_value = RAY_LOST;
    }
    
    delete direction;
    return return_value;
  } 
    catch (exception& e) {cout << "Exception: " << e.what(); getchar(); return INVALID;
  }
}

/*

int PMVOctreeHierarchicalRT::castRayAtAbstractionLevel(BoostMatrix m, rtPointerToElementType ray_node, int a_l)
{
  int return_value;
  
  //float resolution_temp = voxelResolution * pow();
  
  double i_ray,j_ray,k_ray;
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
  //cout << "rotated origin: " << std::endl << rotated_origin << std::endl;
  
  /// Collision Checking
//   if(!collisionFree(origin.x(),origin.y(),origin.z())){
//     return INVALID;
//   }
//   
//   // check for collision
   origin_node =  map->search(origin);
//   
  if(origin_node == NULL){
     //cout << "Origin not found. It could be in a unknown part. " << std::endl;
     //cout << origin.x() << " " << origin.y() << " " << origin.z() << std::endl;
     return INVALID;
  }
  
  // get ray
  ray = ray_node->ray_matrix;
  // The ray is rotated and traslated by the rotation matrix
  rotated_ray = boost::numeric::ublas::prod(m, ray);
  
  //BoostMatrix rotated_end;
  //rotated_end = maxDOV * ray;
  //rotated_end = boost::numeric::ublas::prod(m, rotated_end);
  //point3d end_point(rotated_end(0,0), rotated_end(1,0), rotated_end(2,0));
  
  //cout << "end point" << end_point.x() << " " << end_point.y() << " " <<  end_point.z() << std::endl;
  //cout << "rotated_ray: " << rotated_ray(0,0) << " " <<  rotated_ray(1,0) << " " << rotated_ray(2,0) << std::endl;
  
    //compute direction from position to the rotated ray. This is necesary because the rotated ray was also trasladated by the rotation matrix
  computeRayFromPointToPoint(rotated_origin , rotated_ray, i_ray, j_ray, k_ray);

  
  direction = new octomap::point3d(i_ray, j_ray, k_ray);
  //cout << "direction: " << direction->x() << " " << direction->y() << " " << direction->z() << std::endl;
  delete direction;
  
  point3d scaled_dir(maxDOV*i_ray, maxDOV*j_ray,maxDOV*k_ray);
  point3d end_point = origin + scaled_dir;
  //cout << "end point" << end_point.x() << " " << end_point.y() << " " <<  end_point.z() << std::endl;
  
  //getchar();
  // if the castRay returns true a occupied voxel was hit
  std::vector<point3d> voxels;
  
  float res_temp = voxelResolution * pow(2,a_l);
  int deepth_goal = map->getTreeDepth() - a_l;
  
  map->setResolution(res_temp);
  if (map->computeRay(origin, end_point, voxels)){
    map->setResolution(voxelResolution);
    
    std::vector<point3d>::iterator it_voxel;
    for(it_voxel= voxels.begin(); it_voxel!= voxels.end(); it_voxel++){      
      ColorOcTreeNode *node;
      node= map->search(*it_voxel, deepth_goal);
      
      if(node == NULL){
	if(isInCapsule(*it_voxel, res_temp)){
	  //cout << "Unmark" << std::endl;
	  
	  return UNMARK;
	} else { 
	  if(isInScene(*it_voxel)){
	    //cout << "Unmark Scene" << std::endl;
	    return UNMARK_SCENE;
	  } else {
	    return RAY_LOST;
	  }
	}
      }
      
      if(node->getOccupancy()>0.5){
	if(isInCapsule(*it_voxel, res_temp)){	  
	  //cout << "Occupied" << std::endl;
	  return OCCUPIED;
	} else {
	  if(isInScene(*it_voxel)){
	    //cout << "Occupied scene" << std::endl;
	    return OCCUPIED_SCENE;
	  } else {
	    return RAY_LOST;
	  }
	}
      } 
      
      if(node->getOccupancy() == 0.5) {
	if(isInCapsule(*it_voxel, res_temp)){
	  //cout << "Unmark" << std::endl;
	  return UNMARK;
	} else{
	  if(isInScene(*it_voxel)){
	    //cout << "Unmark Scene" << std::endl;
	    return UNMARK_SCENE;
	  } else {
	    return RAY_LOST;
	  }
	}
      }
      
      // si no fue ninguno de los anteriores entonces es free y continua el rayo
    }
    return RAY_LOST;
  }
  map->setResolution(voxelResolution);
  std::cout << "Aqui hay un problema el punto inicial o final esta fuera del rango del octree!";
  getchar();
 
 
  return (RAY_LOST);
}*/
/*
bool PMVOctreeHierarchicalRT::recursiveHRayTracing(BoostMatrix m, pmRayNode* root_ray_ptr, int depth, EvaluationResult& result)
{
  // we assume that the nodes are not leafsPtrList
  result.clear();
  
  rtChildrenPtrListType::iterator r_it;
  //cout << "children rays: " << root_ray_ptr->children_count << std::endl;;
  for(r_it = root_ray_ptr->childrenPtrList.begin(); r_it!= root_ray_ptr->childrenPtrList.end(); r_it++){
      switch(castRayAtDepth(m,*r_it, depth)){
	case OCCUPIED:
	  if(depth == map->getTreeDepth() || rtIsLeaf(*r_it)){
	    result.n_occupied ++;
	  }
	  else {
	    EvaluationResult aux_result;
	    recursiveHRayTracing(m, *r_it, depth+1, aux_result);
	    result.addVoxelAmouts(aux_result);
	  }
	  break;
	  
	case OCCUPIED_SCENE:
	  result.n_occupied_scene += (*r_it)->nLeafs;
	  break;
	  
	case UNMARK:
	  result.n_unmark += (*r_it)->nLeafs;
	  break;
	
	case UNMARK_SCENE:
	  result.n_unmark_scene += (*r_it)->nLeafs;
	  break;
	  
	case INVALID:
	  break;
	  
	case RAY_LOST:
	  result.n_lost += (*r_it)->nLeafs;
	  break;
      }
  }
 // std::cout << "Occupied: " << n_occupied << " Unmark: " << n_unmark <<  " Unmark scene: " << n_unmark_scene <<  " occupied_scene:" << n_occupied_scene << " lost:" << ray_lost << std::endl;

  return true;
}*/
