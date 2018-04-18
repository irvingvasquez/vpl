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

PMVOctree::~PMVOctree()
{
  //std::cout << "~PMVOctree" << std::endl;
  delete map;
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
	(*it).setLogOdds(logodds(0.5)); // WARNING ?
	(*it).setColor(colorYellow);
      } else{
	(*it).setColor(colorGray);
      }
    }
    it ++;
  }
  
  octree->updateInnerOccupancy();

  return 0;
}

long int PMVOctree::paintVoxels()
{
  return paintVoxels(this->map);
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
	    result.n_unknown ++; 
	  } else if(isInScene(touched_position)){
	    result.n_unknown_scene++;
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
//   std::cout << "RT. Occ:" << result.n_occupied << " Occ_sce:" << result.n_occupied_scene 
//  		    << " Unk:" << result.n_unknown <<  " Unk_sce:" << result.n_unknown_scene 
//  		    << " lost:" << result.n_lost << std::endl;

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
    std::cout << "Unable to read file " << file_address.c_str() << std::endl;
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
 
  if(!map->write(file_name.c_str())){
    return false;
  }
  
  std::cout << "Octomap: " << file_name.c_str() << " saved\n";
    
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
  
  std::cout << "Object VRML: " << file_object_points.c_str() << " saved\n";
  
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
  
  std::cout << "Object point cloud: " << file_pc.c_str() << " saved\n";
  
  unknownVoxelsInOBBx.push_back(n_unk);;
  std::string file_n_uknown(dataFolder);
  file_n_uknown.append("/unknown_voxels_OBBx");
  reader.saveVector<long int>(unknownVoxelsInOBBx, file_n_uknown);
  
  std::cout << "Unknown voxels in BBx amouts was saved\n";
  
  return true;
}


bool PMVOctree::loadPartialModel(std::string file_name)
{
  return false;
}


bool PMVOctree::insertUnknownSurface(Pointcloud pc)
{
  Pointcloud::iterator it;
  float start_prob = 0.5;
  
  for(it = pc.begin(); it!= pc.end(); it++)
  {
    // update all the voxels between center points   
    octomap::point3d endpoint = *it;
    octomap::ColorOcTreeNode* n = map->updateNode(endpoint, false);
    //n->setValue(0);
    n->setLogOdds(logodds(start_prob));
    n->setColor(colorOrange);
  }
}


// void PMVOctree::getOBBVoxelGrid(std::vector< double >& voxels, std::vector< int >& dim)
// {
//   double x,y,z; 
//   float start_prob = 0.5;
//   float prob;
//   dim.resize(3);
//   
//   float x1 = ObjectBBxMin.x();
//   float y1 = ObjectBBxMin.y();
//   float z1 = ObjectBBxMin.z();
//   
//   float x2 = ObjectBBxMax.x();
//   float y2 = ObjectBBxMax.y();
//   float z2 = ObjectBBxMax.z();
//   
//   std::cout << "Inserting free space.." << std::endl;
//   
//   float resolution;
//   int nx, ny, nz;
//   float ox,oy,oz;
//   float p1x, p1y, p1z;
//   float p2x, p2y, p2z;
//   
//   resolution = this->voxelResolution;
//   //resolution = multi_octree_it->getResolution();
//   
//   // compute the origin of the voxels
//   ox = resolution / 2;
//   oy = ox;
//   oz = ox;
//   
//   // how many displacements are need to reach the first point
//   nx = (int) floor(x1/resolution);
//   ny = (int) floor(y1/resolution);
//   nz = (int) floor(z1/resolution);
//   
//   // compute the center of the first point
//   p1x = ox + nx * resolution;
//   p1y = oy + ny * resolution;
//   p1z = oz + nz * resolution;
//   
//   // how many displacements are need to reach the second point
//   nx = (int) floor(x2/resolution);
//   ny = (int) floor(y2/resolution);
//   nz = (int) floor(z2/resolution);
//   
//   // compute the center of the second point
//   p2x = ox + nx * resolution;
//   p2y = oy + ny * resolution;
//   p2z = oz + nz * resolution;
//   
//   //calculate the dimension
//   int dimx = 0;
//   int dimy = 0;
//   int dimz = 0;
//   
//   // update all the voxels between center points   
//   x = p1x;
//   while(x <= p2x){
//     y = p1y;
//     while(y <= p2y){
//       z = p1z;
//       while(z <= p2z){
// 	octomap::point3d endpoint ((float) x, (float) y, (float) z);
// 	OcTreeKey key;
// 	map->coordToKeyChecked(endpoint, key);
// 	ColorOcTreeNode* node = map->search(key);
// 	prob = node->getOccupancy();
// 	
// 	voxels.push_back((double)prob);
// 	dimz ++;
// 	
// 	z = z + resolution;
//       } 
//       dimy ++;
//       y = y + resolution;
//     }
//     dimx ++;
//     x = x + resolution;
//   }
// 
//   dim[0]=dimx;
//   dim[1]=dimy;
//   dim[2]=dimz;
// }



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
  map->expand();
  
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
    //std::cout << "Sorry no points :(" << std::endl;
    return UNFEASIBLE_VIEW;
  }
  
  EvaluationResult result;
  
  bool valid_result = rayTracingHTM(v.HTM, result);
  
  /// Evaluate the result of the raytracing
  if(valid_result){
      v.n_unknown = result.n_unknown;
      v.n_occupied = result.n_occupied;
      v.n_occplane = result.n_unknown;
      
      // See at least an unknowm voxel?
      if(result.n_unknown == 0 && result.n_unknown_scene ==0){
	    v.eval = 0.0;
	    return UNFEASIBLE_VIEW;
      } else {	
	    // check for registration constraint
	    if(registrationConstraint(result)){
	      // evaluation of the view by counting the occplane voxels (unknown voxels of the surface)
	      v.eval = (float) result.n_unknown;
	      return FEASIBLE_VIEW;
	    } else {
	      v.eval = 0.0;
	      return UNFEASIBLE_VIEW;
	    }
      }
  } else 
  { 
    //std::cout << " no valid :S" << std::endl;
    //v.eval = 0.0;
    return UNFEASIBLE_VIEW;
  }
}




bool PMVOctree::registrationConstraint(EvaluationResult r)
{
  float overlap;
  overlap = (float) r.n_occupied /(r.n_occupied + r.n_unknown);
  overlap = overlap * 100;
  
  //std::cout << r.n_occupied << " overlap: " << overlap << std::endl;
  
  if(overlap >= minOverlap)
    return true;
  else 
    return false;
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


bool PMVOctree::saveVisibleUnknown(std::string file_name_vertex, std::string file_name_normal)
{
  point3d_list vertices;
  point3d_list normals;
  point3d_list::iterator itv;
  point3d_list::iterator itn;
  std::vector<double> point(3);
  std::vector< std::vector<double> > data;
  vpFileReader fr;
  
  map->getVisibleUnknownVoxels(vertices, normals);
  itv = vertices.begin();
  itn = normals.begin();
  
  // convert to file format
  while(itv != vertices.end()){
    point[0] = itv->x();
    point[1] = itv->y();
    point[2] = itv->z();
    
    data.push_back(point);
    itv ++;
  }
  
  
  fr.saveDoubleCoordinates(file_name_vertex, data);
  data.clear();
  
  // convert to file format
  while(itn != normals.end()){
    point[0] = itn->x();
    point[1] = itn->y();
    point[2] = itn->z();
    
    data.push_back(point);
    itn ++;
  }
  fr.saveDoubleCoordinates(file_name_normal, data);
  
  return true;
}



bool PMVOctree::saveFrontierUnknown(std::string file_name_vertex, std::string file_name_normal)
{
  point3d_list vertices;
  point3d_list normals;
  point3d_list::iterator itv;
  point3d_list::iterator itn;
  std::vector<double> point(3);
  std::vector< std::vector<double> > data;
  vpFileReader fr;
  
  
  map->getFrontierUnknownVoxels(vertices, normals);
  itv = vertices.begin();
  itn = normals.begin();
  
  // convert to file format
  while(itv != vertices.end()){
    point[0] = itv->x();
    point[1] = itv->y();
    point[2] = itv->z();
    
    data.push_back(point);
    itv ++;
  }
  
  fr.saveDoubleCoordinates(file_name_vertex, data);
  data.clear();
  
  // convert to file format
  while(itn != normals.end()){
    point[0] = itn->x();
    point[1] = itn->y();
    point[2] = itn->z();
    
    data.push_back(point);
    itn ++;
  }
  fr.saveDoubleCoordinates(file_name_normal, data);
  
  return true;
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
  map->expand();
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
