/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

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


#include "rangesimulatoroctree.h"

RangeSimulatorOctree::RangeSimulatorOctree():RangeSimulatorBase()
{
  
}

bool RangeSimulatorOctree::init()
{
  //std::cout << "--- Simulator with octree ---" << std::endl;
  RangeSimulatorBase::init();
  
  std::string config_file(configFolder);
   config_file.append("/");
   config_file.append("rangeSimulator.ini");
  
   
   INIReader reader(config_file.c_str());
  
   if (reader.ParseError() < 0) {
	  std::cout << "Can't load " << config_file.c_str() << "\n";
	  return false;
   }
   
    
   voxelResolution = reader.GetReal("volumetric", "resolution", -1);
   maxLenght = reader.GetReal("volumetric", "maxLenght", 0); 
   measureUnknown = false;
   freeSpace = reader.GetBoolean("volumetric", "freeSpace", false);
   
   float x,y,z,yaw,pitch,roll;
   
   x  = reader.GetReal( "transformation", "x", 0);
   y = reader.GetReal( "transformation", "y", 0);
   z = reader.GetReal( "transformation", "z", 0);
   yaw = reader.GetReal( "transformation", "yaw", 0);
   pitch = reader.GetReal( "transformation", "pitch", 0);
   roll = reader.GetReal( "transformation", "roll", 0);
   scene_scale = reader.GetReal( "transformation", "scale", 1);
   
   octomap::pose6d poseT(x,y,z,yaw,pitch,roll);
   scene_transformation = poseT;
   
   if(freeSpace){
      x_free_1 = reader.GetReal("freeSpaceCoord", "x1", -1);
      x_free_2 = reader.GetReal("freeSpaceCoord", "x2", -1);
      
      y_free_1 = reader.GetReal("freeSpaceCoord", "y1", -1);
      y_free_2 = reader.GetReal("freeSpaceCoord", "y2", -1);
      
      z_free_1 = reader.GetReal("freeSpaceCoord", "z1", -1);
      z_free_2 = reader.GetReal("freeSpaceCoord", "z2", -1);
   }
   
   model = new octomap::ColorOcTree(voxelResolution);
   
   
   //model->prune();
      
   std::string model_f;
   model_f.assign( reader.Get("scene", "file", "none.wrl") );
   modelFile.clear();
   modelFile.assign(configFolder);
   modelFile.append("/");
   modelFile.append(model_f);
   
   isRawFormat = reader.GetBoolean("scene", "rawFormat", true);
   
   std::string rays_f;
   rays_f.assign( reader.Get("sensor", "raysFile", "sensor.dat") );
   sensorFile.clear();
   sensorFile.assign(configFolder);
   sensorFile.append("/");
   sensorFile.append(rays_f);
   
   
   std::cout << "--------------  Range simulator Configuration ----------------" << std::endl;
   std::cout << "Resolution: " << voxelResolution << std::endl;
   std::cout << "Maximum lenght: " << maxLenght << std::endl;
   std::cout << "File (file):" << modelFile << std::endl;
   std::cout << "Raw format (rawFormat):" << isRawFormat << std::endl;
   std::cout << "Rays file:" << sensorFile << std::endl;
   std::cout << "Scene Transformation:" << std::endl;
   std::cout << "(freeSpace): " << freeSpace << std::endl;
   loadModel(modelFile);
   readRays(sensorFile);
   if(freeSpace){
     insertFreeSpace(x_free_1, y_free_1, z_free_1, x_free_2, y_free_2, z_free_2);
   }
   std::cout << "-------------------------------------------------------------"<< std::endl;
}



bool RangeSimulatorOctree::loadModel(std::string file)
{
  //return RangeSimulatorBase::loadModel(file);
  vpFileReader reader;
  octomap::point3d lowerBound;
  octomap::point3d upperBound;
  octomap::Pointcloud cloud;
  float lx,ly,lz;
  float x,y,z;
  
  std::vector< std::vector<double> > model_points;
  std::vector< std::vector<double> >::iterator it;
  std::vector<double>::iterator it_v;
  
  if(isRawFormat){
    reader.readDoubleCoordinates(file,model_points);
  }else {
    reader.readDoubleCoordinatesFromWRL(file, model_points);
  }
  
  for (it = model_points.begin(); it != model_points.end(); it++){
    octomap::point3d p((float)(*it)[0], (float)(*it)[1], (float)(*it)[2]);
    cloud.push_back(p);
  }

//   cloud.calcBBX(lowerBound, upperBound);
//   
//   lx=upperBound.x()-lowerBound.x();
//   ly=upperBound.y()-lowerBound.y();
//   lz=upperBound.z()-lowerBound.z();
//   
//   x = lowerBound.x() + (lx)/2;
//   y = lowerBound.y() + (ly)/2;
//   z = lowerBound.z() + (lz)/2;
//   
//   float lmax = 0;
//   if (abs(lx) > lmax)
//     lmax = lx;
//   
//   if(abs(ly) > lmax)
//     lmax = ly;
//   
//   if(abs(lz) > lmax)
//     lmax = lz;
  
  //float factor;
  //factor = maxLenght / lmax;
    
  std::cout << "Center position of the model: " << x << " " << y << " " << z << std::endl;
  std::cout << "Scale factor: " << scene_scale << std::endl;
  
  ///Scale
  octomap::Pointcloud::iterator cl_it;
  for (cl_it = cloud.begin(); cl_it != cloud.end(); cl_it ++){
    (*cl_it) =  (*cl_it) * scene_scale;
  }
  
  // Recompute values
//   cloud.calcBBX(lowerBound, upperBound);
//   lx=upperBound.x()-lowerBound.x();
//   ly=upperBound.y()-lowerBound.y();
//   lz=upperBound.z()-lowerBound.z();
//   
//   x = lowerBound.x() + (lx)/2;
//   y = lowerBound.y() + (ly)/2;
//   z = lowerBound.z() + (lz)/2;
  
//   std::cout << "After scalation center position of the model: " << x << " " << y << " " << z << std::endl;
  
  /// Rotate and Translate
//   octomap::pose6d original_pose(x,y,z,0,0,0);
//   octomap::pose6d transf;
//   transf = scene_transformation * original_pose.inv();
  //cloud.transformAbsolute(transf);
  cloud.transformAbsolute(scene_transformation);
  
  /// copy to octree 
  for (cl_it = cloud.begin(); cl_it != cloud.end(); cl_it ++){
    octomap::ColorOcTreeNode* n = model->updateNode((*cl_it), true); 
    n->setColor(255,255,0); // set color to red
  }
  
  model->updateInnerOccupancy();
}


bool RangeSimulatorOctree::save(std::string file)
{
  //return RangeSimulatorBase::save(file);
  model->write(file);// writeBinary(file.c_str());
}


void RangeSimulatorOctree::insertFreeSpace(double x1, double y1, double z1, double x2, double y2, double z2)
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

	  octomap::ColorOcTreeNode* n = model->updateNode(endpoint, false);
	
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
  
  model->updateInnerOccupancy();
  model->prune();
}

long int RangeSimulatorOctree::readRays(std::string file_address)
{
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


bool RangeSimulatorOctree::rayTracingPointCloud(boost::numeric::ublas::matrix< double > m, octomap::Pointcloud& cloud)
{
  double i_ray,j_ray,k_ray;
  
  //  long int n_occupied_scene = 0;
  long int ray_lost = 0;
  
  std::vector< boost::numeric::ublas::matrix<double> >::iterator it;
  boost::numeric::ublas::matrix<double> rotated_ray;
  boost::numeric::ublas::matrix<double> cero_origin(4,1);
  cero_origin(0,0) = 0;
  cero_origin(1,0) = 0;
  cero_origin(2,0) = 0;
  cero_origin(3,0) = 1;
  boost::numeric::ublas::matrix<double> rotated_origin;
  boost::numeric::ublas::matrix<double> ray;
  
  octomap::point3d *direction;
  octomap::point3d touched_position;
  
  octomap::ColorOcTreeNode *touched_node;
  octomap::ColorOcTreeNode *origin_node;
  
  //cout << "m: " << std::endl << m << std::endl;
  //cout << "origin: " << std::endl << cero_origin << std::endl;
  rotated_origin = boost::numeric::ublas::prod(m, cero_origin);
  octomap::point3d origin((float)rotated_origin(0,0), (float)rotated_origin(1,0), (float)rotated_origin(2,0));
  //cout << "rotated origin: " << std::endl << rotated_origin << std::endl;
  
  origin_node = model->search(origin);
  if(origin_node == NULL){
    std::cout << "Origin not found. It could be in a unknown part" << std::endl;
    std::cout << origin.x() << " " << origin.y() << " " << origin.z() << std::endl;
    return false;
  }
  
  cloud.clear();
  
  for(it = rays.begin(); it!= rays.end(); it++){
     // rotate by m
     ray = *it;
     /// The ray is rotated and traslated by the rotation matrix
     rotated_ray = boost::numeric::ublas::prod(m, ray);
     
     //cout << "rotated_ray: " << rotated_ray(0,0) << " " <<  rotated_ray(1,0) << " " << rotated_ray(2,0) << std::endl;
     
     //compute direction from position to the rotated ray. This is necesary because the rotated ray was also trasladated by the rotation matrix
     computeRayFromPointToPoint(rotated_origin , rotated_ray, i_ray, j_ray, k_ray);
     
     direction = new octomap::point3d((float)i_ray, (float)j_ray, (float)k_ray);
     
     //cout << "direction: " << direction->x() << " " << direction->y() << " " << direction->z() << std::endl;
     // getchar();
     
     model->castRay(origin, *direction, touched_position);
     
     // if the cast rays returns true a occupied voxel was hit
    /* if (map->castRay(origin, *direction, touched_position)){
     } else{
     }
    */ 
     cloud.push_back(touched_position);
     
     delete direction;
  }

  return true;
}


bool RangeSimulatorOctree::takeAndSaveScan(boost::numeric::ublas::matrix< double > htm, std::string &scan_name, std::string &origin_name)
{
  std::cout << "Taking range image." << std::endl;
  //std::stringstream extension;
  //extension << "_" <<  scanIdCounter << ".dat";
  //scan_name += extension.str();
  //origin_name += extension.str();
  
  octomap::Pointcloud cloud;
  octomap::Pointcloud::iterator it;
    
  rayTracingPointCloud(htm, cloud);
  
  std::ofstream myfile (scan_name.c_str());
  std::cout << "Saving file: " << scan_name.c_str() << std::endl;
  if (!myfile.is_open())
  {
    std::cout << "Unable to open file" << scan_name.c_str() << std::endl;
    return false;
  }
  for(it = cloud.begin(); it!= cloud.end(); it++){
      myfile << it->x() << " " << it->y() << " " << it->z() << std::endl;
  }
  myfile.close();
  
  boost::numeric::ublas::matrix<double> cero_origin(4,1);
  cero_origin(0,0) = 0;
  cero_origin(1,0) = 0;
  cero_origin(2,0) = 0;
  cero_origin(3,0) = 1;
  boost::numeric::ublas::matrix<double> rotated_origin;
  rotated_origin = boost::numeric::ublas::prod(htm, cero_origin);
  
  std::ofstream myfileOrigin (origin_name.c_str());
  std::cout << "Saving file: " << origin_name.c_str() << std::endl;
  if (!myfileOrigin.is_open())
  {
    std::cout << "Unable to open file"  << origin_name.c_str() << std::endl;
    return false;
  }
  myfileOrigin << rotated_origin(0,0) << " " << rotated_origin(1,0) << " " << rotated_origin(2,0) << std::endl;
  myfileOrigin.close();
  
  scanIdCounter ++;

  return true;
}



void RangeSimulatorOctree::computeRayFromPointToPoint(boost::numeric::ublas::matrix<double> pointA, boost::numeric::ublas::matrix<double> pointB, double& i, double& j, double& k)
{
  i = pointB(0,0) - pointA(0,0);
  j = pointB(1,0) - pointA(1,0);
  k = pointB(2,0) - pointA(2,0);
}


bool RangeSimulatorOctree::takeAndSaveScan(ViewStructure v, std::string file_name)
{
  octomap::Pointcloud cloud;
  octomap::Pointcloud::iterator it;
    
  rayTracingPointCloud(v.HTM, cloud);
      
  std::ofstream myfile (file_name.c_str());
  if (!myfile.is_open())
  {
    std::cout << "Unable to open file:"  << file_name.c_str() << std::endl;
    return false;
  }
  
  for(it = cloud.begin(); it!= cloud.end(); it++){
      myfile << it->x() << " " << it->y() << " " << it->z() << std::endl;
  }
    
  myfile.close();
  return true;
}


