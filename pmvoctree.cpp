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


#include "pmvoctree.h"


PMVOctree::PMVOctree() : PMVolumetric()
{

}

bool PMVOctree::init()
{  
  PMVolumetric::init();
  
  std::cout << "---------------- Octree -------------------" << std::endl;
  
  //map = new octomap::ColorOcTree(voxelResolution);
  map = new COctreeVPL(voxelResolution);
  
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


void PMVOctree::insertFreeSpace(double x1, double y1, double z1, double x2, double y2, double z2)
{
  double x,y,z; 
  //TODO read this values:
  float start_prob = 0.5;
  
  if(x1>x2)
    return;
  
  if(y1>y2)
    return;
  
  if(z1>z2)
    return;
  
  std::cout << "Inserting free space.." << std::endl;
  
  float resolution;
  int nx, ny, nz;
  float ox,oy,oz;
  float p1x, p1y, p1z;
  float p2x, p2y, p2z;
  
  resolution = this->voxelResolution;
  //resolution = multi_octree_it->getResolution();
  
  // compute the origin of the voxels
  ox = resolution / 2;
  oy = ox;
  oz = ox;
  
  // how many displacements are need to reach the first point
  nx = (int) floor(x1/resolution);
  ny = (int) floor(y1/resolution);
  nz = (int) floor(z1/resolution);
  
  // compute the center of the first point
  p1x = ox + nx * resolution;
  p1y = oy + ny * resolution;
  p1z = oz + nz * resolution;
  
  // how many displacements are need to reach the second point
  nx = (int) floor(x2/resolution);
  ny = (int) floor(y2/resolution);
  nz = (int) floor(z2/resolution);
  
  // compute the center of the second point
  p2x = ox + nx * resolution;
  p2y = oy + ny * resolution;
  p2z = oz + nz * resolution;
  
  // update all the voxels between center points   
  x = p1x;
  while(x <= p2x){
    y = p1y;
    while(y <= p2y){
      z = p1z;
      while(z <= p2z){
	octomap::point3d endpoint ((float) x, (float) y, (float) z);
	
// 	if(!isInCapsule(endpoint)){
// 	  octomap::ColorOcTreeNode* n = map->updateNode(endpoint, false);
// 	  n->setColor(colorYellow);
// 	} 
// 	
	
	if(isInCapsule(endpoint)){
	  octomap::ColorOcTreeNode* n = map->updateNode(endpoint, false);
	  //n->setValue(0);
	  n->setLogOdds(logodds(start_prob));
	  n->setColor(colorOrange);
	} else {
	  octomap::ColorOcTreeNode* n = map->updateNode(endpoint, false);
	  //n->setLogOdds(logodds(0.0));
	  //n->setValue(1);
	  //n->setColor(colorCian);
	}
	
	z = z + resolution;
      } 
      y = y + resolution;
    }
    x = x + resolution;
  }
  
  // insert unknown voxels
//   point3d_list centers;
//   point3d_list::iterator it;
//   map->getUnknownLeafCenters(centers, SceneBBxMin, SceneBBxMax);
//   
//   //TODO read this values:
//   float unknown_prob = 0.5;
//   
//   for(it= centers.begin(); it != centers.end(); it++){
//     // marcar como unknown
//     if(isInCapsule(*it)){
//       octomap::ColorOcTreeNode* n = map->updateNode(*it, false);
//       n->setValue(unknown_prob);
//       n->setLogOdds(logodds(unknown_prob));
//       n->setColor(colorOrange);
//     } else {
//       octomap::ColorOcTreeNode* n = map->updateNode(*it, false);
//       n->setColor(colorYellow);
//     }
//   }
  
  map->updateInnerOccupancy();
  map->prune();
}



// void PMVOctree::paintOccupiedInCapsule()
// {
//   octomap::ColorOcTree::leaf_iterator it= map->begin_leafs();
//   while(it != map->end_leafs()){
//     if(it->getOccupancy() > 0.5){ 
//       if(isInCapsule(it.getCoordinate())){
// 	(*it).setColor(colorBlue);
//       }
//     }
//     it ++;
//   }
//   
//   map->updateInnerOccupancy();
// }



long int PMVOctree::paintVoxels(COctreeVPL* octree)
{
  octomap::ColorOcTree::leaf_iterator it= octree->begin_leafs();
  while(it != octree->end_leafs()){
    if( octree->isNodeOccupied(&(*it)) ){ 
      if(isInCapsule(it.getCoordinate())){
	(*it).setColor(colorBlue);
      }
      else {
	(*it).setColor(colorGray);
      }
    } else if(octree->isNodeUnknown(&(*it))){
      if(isInCapsule(it.getCoordinate())){
	(*it).setLogOdds(logodds(0.5));
	(*it).setColor(colorOrange);
      } else{
	(*it).setColor(colorGray);
      }
    }
    it ++;
  }
  
  octree->updateInnerOccupancy();

  return 0;
}



bool PMVOctree::rayTracingHTM(boost::numeric::ublas::matrix< double > m, EvaluationResult& result)
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
	    result.n_unmark ++; 
	  } else if(isInScene(touched_position)){
	    result.n_unmark_scene++;
	  } else {
	    result.n_lost++;
	  }
       } else {
	 result.n_lost ++;
       }
     }else {
	 result.n_lost ++;
     }
     
     delete direction;
  }
//  std::cout << "RT. Occ:" << result.n_occupied << " Occ_sce:" << result.n_occupied_scene 
// 		    << " Unk:" << result.n_unmark <<  " Unk_sce:" << result.n_unmark_scene 
// 		    << " lost:" << result.n_lost << std::endl;

  //map->write("octree_painted.ot");
  map->cleanTouchedVoxels();
  return true;
}


long int PMVOctree::readRays(std::string file_address)
{
  rays.clear();
  
  std::cout << "Reading rays...." << std::endl;
  double x_r, y_r, z_r;
  long int n_rays;
  long int readed_rays = 0;
  
  boost::numeric::ublas::matrix<double> ray(4,1);
  
  std::ifstream file(file_address.c_str());
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
	
	rays.push_back(ray);
	
	readed_rays ++;
	
	file >> x_r;
    }
    file.close();
  } else {
    std::cout << "Unable to open file " << file_address.c_str() << std::endl;
    return -1;
  }
  
  //cout << "done." << std::endl;
  std::cout << "Readed rays: "<< readed_rays << std::endl;
  
  return readed_rays;
}


bool PMVOctree::savePartialModel(std::string file_name)
{ 
  vpFileReader reader;
  
  if (map->size() == 0){
    std::cout << "Octree empty" << std::endl;
    return false;
  }
  
  std::cout << "Writing to " << file_name.c_str() << std::endl;
  
  if(!map->write(file_name.c_str())){
    return false;
  }
  
  //paintOccupiedInCapsule();
  
  long int n_unk;
  std::string file_uknown(file_name);
  file_uknown.append(".unk.ot");
  COctreeVPL temp_octree(*(this->map));
  n_unk = paintVoxels(&temp_octree);
  if(!temp_octree.write(file_uknown.c_str())){
    return false;
  }
  
  // save accumulated points
  std::string file_object_points(dataFolder);
  file_object_points.append("/object_accumulated_points.wrl");
  objectPointCloud.writeVrml(file_object_points);
  
  
  Pointcloud::iterator it;
  std::vector< std::vector<double> > vec_pc;
  for (it = objectPointCloud.begin(); it!= objectPointCloud.end(); it++){
    std::vector<double> vec(3);
    vec[0] = it->x();
    vec[1] = it->y();
    vec[2] = it->z();
    vec_pc.push_back(vec);
  }
  
  std::string file_pc(dataFolder);
  file_pc.append("/");
  file_pc.append(object_points_filename);
  reader.saveDoubleCoordinates(file_pc, vec_pc);
  
  unknownVoxelsInOBBx.push_back(n_unk);;
  std::string file_n_uknown(dataFolder);
  file_n_uknown.append("/unknown_voxels_OBBx");
  reader.saveVector<long int>(unknownVoxelsInOBBx, file_n_uknown);
  
  return true;
}


bool PMVOctree::loadPartialModel(std::string file_name)
{
  if(map!= NULL)
    delete map;
 
  AbstractOcTree *tree = tree->read(file_name);
  map = dynamic_cast<COctreeVPL*>(tree);
  
  /*
  std::cout << "Reading file: " << file_name << std::endl; 
  AbstractOcTree* readTreeAbstract;
  readTreeAbstract = AbstractOcTree::read(file_name);
  //EXPECT_TRUE(readTreeAbstract);
  std::cout << "Readed: " << readTreeAbstract->getTreeType() << std::endl;
  //EXPECT_EQ(colorTree.getTreeType(),  readTreeAbstract->getTreeType());
  ColorOcTree* readColorTree = dynamic_cast<ColorOcTree*>(readTreeAbstract);
 
  //readColorTree->write("leido.ot");
  // Convert 
  
  map = new COctreeVPL(voxelResolution);
  
  ColorOcTree::iterator it;
  map-> ;
  */
  
  return true;
}


float PMVOctree::updateWithScan(std::string file_name_scan, std::string file_name_origin)
{
  std::cout << "Updating octree with scan." << std::endl;
  
  if (!readPointCloudFromDAT(file_name_scan, *scanCloud))
    return -1;
  if (!readPointCloudFromDAT(file_name_origin, *scanCloudOrigins))
    return -1;
  
  // TODO: Validate that there is a origin and a point cloud
  
  octomap::Pointcloud::iterator it_origin;
  it_origin = scanCloudOrigins->begin(); //We will take the first point of scanCloudOrigins as origin
  
  double diff;
  clock_t start = clock();
  
  map->insertPointCloud(*scanCloud, *it_origin, maxRange);
  map->updateInnerOccupancy();
  map->prune();
  
  clock_t ends = clock();
  diff = (double) (ends - start) / CLOCKS_PER_SEC;

  printf("Octree update took %.2lf seconds to run.\f", diff );
  
  scanCloud->crop(ObjectBBxMin, ObjectBBxMax);
  
  objectPointCloud.push_back(scanCloud);
  
  scanCloud->clear();
  
  return (float) diff;
}


int PMVOctree::evaluateView(ViewStructure& v)
{
  if(!poitsToTheObject(v)){
    //cout << "Sorry no points :(" << std::endl;
    return UNFEASIBLE_VIEW;
  }
  
  EvaluationResult result;
  
  bool valid_result = rayTracingHTM(v.HTM, result);
  
  if(valid_result){
    /// Evaluate the result of the raytracing
    if( this->utilityFunction->evaluate(result) == FEASIBLE_VIEW){
	v.eval = result.evaluation;
	v.n_unmark = result.n_unmark;
	v.n_occupied = result.n_occupied;
	return FEASIBLE_VIEW;
    }
  }
  
  return UNFEASIBLE_VIEW;
}



void PMVOctree::evaluateCandidateViews()
{
  std::list<ViewStructure>::iterator it_v;
  int removed_views=0;
  clock_t start;
  double diff;
  bool valid_result;
  EvaluationResult result;
  int i =0;
  
  std::cout << "Evaluating candidate views with octree." << std::endl;
  
  
  if(rays.size() ==0){
    std::cout << "rays have not been readed" << std::endl;
    return;
  }
  
  stopCriteria = true;
  evals.clear();
  
  it_v = candidateViews.begin();
  while(it_v != candidateViews.end()){
    i++; //display
    
    result.clear();
    //printVector(it_v->q);
    
    start = clock();
    
    valid_result = rayTracingHTM(it_v->HTM, result);
    diff = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;
    
    result.computation_time = diff;
    evals.push_back(result);
    
    if(valid_result){       
      
      /// Evaluate the result of the raytracing
      if( this->utilityFunction->evaluate(result) == UNFEASIBLE_VIEW){
	it_v = candidateViews.erase(it_v);
	std::cout << "Unfeasible view" << std::endl;
	removed_views ++;
      } else {
	if(result.n_unmark > minUnknown)
	  stopCriteria = false;
	
	std::cout << "Evaluation " << i << ": " << result.evaluation << std::endl;
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


void PMVOctree::setUtilityFunction(VolumetricUtilityFunction* uf)
{
  utilityFunction = uf;
}


void PMVOctree::saveObjectAsRawT(std::string file_name)
{
  //point3d_list occ_centers;
  //map->getOccupiedLeafsBBX(occ_centers, ObjectBBxMin, ObjectBBxMax);
  vpTriangleList triangles;
  getOccupiedTriangles(triangles);
  triangles.saveToFile(file_name);
}


void PMVOctree::saveObjectAsObst(std::string file_name)
{
  vpTriangleList triangles;
  getOccupiedTriangles(triangles);
  triangles.saveToMSLTriangle(file_name);
}

bool PMVOctree::saveUnknownVolumeAsObst(std::string file_name)
{
  vpTriangleList triangles;
  getUnknownTriangles(triangles);
  triangles.saveToMSLTriangle(file_name);
}


bool PMVOctree::saveUnknownVolumeAsRawT(std::string file_name)
{
  vpTriangleList triangles;
  getUnknownTriangles(triangles);
  triangles.saveToFile(file_name);
}



bool PMVOctree::saveObstacle(std::string file_name)
{
  vpTriangleList triangles;
  
  vpTriangleList tr_occ;
  getOccupiedTriangles(tr_occ);
  
  vpTriangleList tr_unk;
  getUnknownTriangles(tr_unk);
  
  triangles.insert(triangles.end(), tr_occ.begin(), tr_occ.end());
  triangles.insert(triangles.end(), tr_unk.begin(), tr_unk.end());
  
  triangles.saveToMSLTriangle(file_name);
}


void PMVOctree::getOccupiedTriangles(vpTriangleList& tris)
{
  //point3d_list occ_centers;
  map->setBBXMin(ObjectBBxMin);
  map->setBBXMax(ObjectBBxMax);
  map->useBBXLimit(true);
  vpTriangleList tris_occupied;
  
  tris.clear();
  
  octomap::ColorOcTree::leaf_iterator it= map->begin_leafs();
  while(it != map->end_leafs()){
    if(it->getOccupancy() > 0.5){ 
      if(isInCapsule(it.getCoordinate())){
	getTrianglesOfVoxel(tris_occupied, it.getCoordinate(), map->getResolution());
	tris.insert(tris.end(), tris_occupied.begin(), tris_occupied.end());
      }
    }
    it ++;
  }
  
  map->useBBXLimit(false);
}

void PMVOctree::getUnknownTriangles(vpTriangleList& tris)
{
  point3d_list unk_centers;
  point3d_list::iterator it;
  vpTriangleList tris_unknown;
  
  tris.clear();
  
  map->getUnknownCentersAll(unk_centers);
  
  for(it=unk_centers.begin(); it!=unk_centers.end(); it++){
    getTrianglesOfVoxel(tris_unknown, *it, map->getResolution());
    tris.insert(tris.end(), tris_unknown.begin(), tris_unknown.end());
  }
    
}


double PMVOctree::getUnknownVolume()
{
  point3d_list centers;
  std::vector<double> sizes;
  std::vector<double>::iterator it;
  
  map->getUnknownVoxels(centers, sizes);
  double vol = 0;
  double accumulated_volume = 0;
  
  for(it = sizes.begin(); it != sizes.end(); it++){
    //cout << "voxel size:" << *it << std::endl;
    vol = pow(*it,3);
    accumulated_volume = accumulated_volume + vol;
  }
  
  return accumulated_volume;
}



