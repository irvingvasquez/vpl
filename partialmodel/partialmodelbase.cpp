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


#include "partialmodelbase.h"


PartialModelBase::PartialModelBase():
colorOrigin(255,255,0),
colorRed(255,0,0),
colorBlue(0,0,255),
colorYellow(255,255,0),
colorCian(0,255,255),
colorOrange(255,165,0),
colorGray(200,200,200)
{
  scanOrigin = new octomap::point3d (0,0,0);
  scanCloudOrigins = new octomap::Pointcloud();
  scanCloud = new octomap::Pointcloud();
  
  colorOccupied = colorBlue;
  colorUnmark = colorOrange;
  colorTouchedOccupied = colorCian;
  colorTouchedUnkmark = colorRed;
  
  object_points_filename.assign("object_pts.dat");
}


bool PartialModelBase::init()
{
  float x1, x2, y1, y2, z1, z2;
  double x,y,z;
 
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("partialModelConfig.ini");
  
  //mrpt::utils::CConfigFile parser;
  //ASSERT_FILE_EXISTS_(config_file);
  //parser.setFileName(config_file);
    
  INIReader reader(config_file.c_str());
  
  if (reader.ParseError() < 0) {
        std::cout << "Can't load " << config_file.c_str() << "\n";
        return false;
  }
  
  std::cout << "\n------------- Partial Model Configuration ------------" << std::endl;
  
  x = reader.GetReal("directorRay", "x", 0);
  y = reader.GetReal("directorRay", "y", 0);
  z = reader.GetReal("directorRay", "z", 0);
  minDOV = reader.GetReal("sensor", "minDOV", 0);
  maxDOV = reader.GetReal("sensor", "maxDOV", 50);
  point3d dr(x,y,z);
  directorRay = dr;
 
  std::cout << "Sensor:" << std::endl;
  std::cout << "  Director ray: " << directorRay << std::endl;
  std::cout << "  Minimun DOV: " << minDOV << std::endl;
  std::cout << "  Maximun DOV: " << maxDOV << std::endl;
  
  
  x1 = reader.GetReal("objectCapsule", "x1", 0);  
  x2 = reader.GetReal("objectCapsule", "x2", 0);  
  y1 = reader.GetReal("objectCapsule", "y1", 0);  
  y2 = reader.GetReal("objectCapsule", "y2", 0);
  z1 = reader.GetReal("objectCapsule", "z1", 0);
  z2 = reader.GetReal("objectCapsule", "z2", 0);
  
  setObjectCapsule(x1,y1,z1,x2,y2,z2);
  point3d p_end(x2,y2,z2);
  std::cout << "Object Capsule: [" << x1 << ", " << y1 << ", " << z1 << "][" << x2 << ", " << y2 << ", " << z2 << "]" << std::endl;
  
  x = (x1+x2)/2;
  y = (y1+y2)/2;
  z = (z1+z2)/2;
  point3d p(x,y,z);
  objectSphereCenter = p;
  std::cout << "Capsule Center: " << objectSphereCenter << std::endl;;
  
  point3d pr = p_end - p;
  objectSphereRadius =  pr.norm(); //reader.GetReal("objectCenter", "radio", 1.0);
  objRadius2 = objectSphereRadius;// * objectSphereRadius;
  std::cout <<  "Capsule radius:" << objectSphereRadius << std::endl;
  
  x1 = reader.GetReal("sceneCapsule", "x1", 0);
  x2 = reader.GetReal("sceneCapsule", "x2", 0);
  y1 = reader.GetReal("sceneCapsule", "y1", 0);
  y2 = reader.GetReal("sceneCapsule", "y2", 0);
  z1 = reader.GetReal("sceneCapsule", "z1", 0);
  z2 = reader.GetReal("sceneCapsule", "z2", 0);
  
  setScene(x1,y1,z1,x2,y2,z2);
  std::cout << "Escene: [" << x1 << ", " << y1 << ", " << z1 << "][" << x2 << ", " << y2 << ", " << z2 << "]" << std::endl;
  
  evaluationsFile.clear();
  evaluationsFile.assign(dataFolder);
  evaluationsFile.append("/partial_model_evaluations.dat");

  //rbb_size = reader.GetReal(ini_file, "partialModel:rbb_size", 0);
  
  
  
  
//   x = reader.GetReal("sensorPose", "x", 0);
//   y = reader.GetReal("sensorPose", "y", 0);
//   z = reader.GetReal("sensorPose", "z", 0);
//   
//   double yaw, pitch, roll;
//   yaw = reader.GetReal("sensorPose", "yaw", 0);
//   pitch = reader.GetReal("sensorPose", "pitch", 0);
//   roll = reader.GetReal("sensorPose", "roll", 0);
//   sensorReferenceFramePose.resize(6);
//   sensorReferenceFramePose[0] = x;
//   sensorReferenceFramePose[1] = y;
//   sensorReferenceFramePose[2] = z;
//   sensorReferenceFramePose[3] = yaw;
//   sensorReferenceFramePose[4] = pitch;
//   sensorReferenceFramePose[5] = roll;
  
//  std::cout << "Sensor pose: "; PMUtils::printVector(sensorReferenceFramePose); 
 
  
  std::cout << "-------------------------------------" << std::endl;
  return true;
}



void PartialModelBase::setObjectCapsule(double x1, double y1, double z1, double x2, double y2, double z2)
{
 //TODO: validate data
  
  x_cap_1 = x1;
  y_cap_1 = y1;
  z_cap_1 = z1;
  
  x_cap_2 = x2;
  y_cap_2 = y2;
  z_cap_2 = z2;
}


bool PartialModelBase::saveEvaluatedViews(std::string file_name)
{
  return vsSaveViewList(candidateViews, file_name);
}


bool PartialModelBase::saveOnlyNViews(int n, std::string file_name)
{
  std::list<ViewStructure> secondList;
  getNViewsFromList(candidateViews, n, secondList);
  return vsSaveViewList(secondList, file_name);
}


bool PartialModelBase::readCandidateViews(std::string file_name)
{
  candidateViews.clear();
  return vsReadViewList(candidateViews, file_name);
}


void PartialModelBase::setScene(double x1, double y1, double z1, double x2, double y2, double z2)
{
 //TODO: validate data
  
  x_sce_1 = x1;
  y_sce_1 = y1;
  z_sce_1 = z1;
  
  x_sce_2 = x2;
  y_sce_2 = y2;
  z_sce_2 = z2;
}

// void PartialModelBase::setPaintPartialModel(bool value)
// {
//   paintPartialModel = value;
// }


bool PartialModelBase::isInCapsule(point3d point)
{
  if( (point.x() >=  x_cap_1) && (point.x() <= x_cap_2) )
    if (point.y() >= y_cap_1 && point.y() <= y_cap_2 )
      if ( point.z() >= z_cap_1 && point.z() <= z_cap_2 )
	return true;
  
//  std::cout << point.x() << " " << point.y() << " " << point.z() << std::endl;
//  std::cout << "false" << std::endl;
  return false;
}


bool PartialModelBase::isInScene(point3d point)
{
 if( (point.x() >=  x_sce_1) && (point.x() <= x_sce_2) )
    if (point.y() >= y_sce_1 && point.y() <= y_sce_2 )
      if ( point.z() >= z_sce_1 && point.z() <= z_sce_2 )
	return true;
      
  return false;
}




// bool PartialModelBase::readScanFromPCD(std::string file_name, octomap::Pointcloud &cloud)
// {
// //   double x_p, y_p, z_p;
// //   long int readed_points = 0;
// //  // octomap::point3d *point_on_surface;
// //   
// //   pcl::PointCloud<pcl::PointXYZ> points;
// //   pcl::io::loadPCDFile(file_name.c_str(), points);
// //   pcl::PointCloud<pcl::PointXYZ>::iterator it;
// //   
// //   cloud.clear();
// //   
// //   for(it = points.begin(); it != points.end(); it++){
// //     	//point_on_surface = new octomap::point3d(x_p, y_p, z_p);
// //     	if( isnan(it->x) || isnan(it->y) || isnan(it->z) ){
// // 	    std::cout << it->x << " " << it->y << " " << it->z << std::endl;
// // 	} 
// // 	else {
// // 	  cloud.push_back(new octomap::point3d(it->x, it->y, it->z));
// // 	  readed_points ++;
// // 	}
// // 	//delete point_on_surface;
// //   }
// //   
// //   std::cout << "done." << std::endl;
// //   std::cout << "Readed points: "<< readed_points << std::endl;
// //   
// // //   if(!readPointCloudFromFile(origin_file, *scanCloudOrigins))
// // //     return false;
// // //   
//   return true;
// }


bool PartialModelBase::readPointCloudFromDAT(std::string file_name, Pointcloud& cloud)
{
 double x_p, y_p, z_p;
  long int n_points;
  long int readed_points = 0;
  octomap::point3d *point_on_surface;
 
  cloud.clear();
  
  std::ifstream file(file_name.c_str());
  if(file.is_open()){
//    file >> n_points;
    file >> x_p;
    
    while(file.good()){
	
	file >> y_p;
	file >> z_p;
	
	point_on_surface = new octomap::point3d(x_p, y_p, z_p);
	
	cloud.push_back(new octomap::point3d(x_p, y_p, z_p));
	
	readed_points ++;
	delete point_on_surface;
	
	file >> x_p;
    }
    
    file.close();
  } else {
    std::cout << "Unable to open file"  << file_name.c_str() << std::endl;
    return false;
  }
  
  std::cout << "done." << std::endl;
  std::cout << "Readed points: "<< readed_points << std::endl;
  
  return true;
}


void PartialModelBase::computeRayFromOriginToPoint(double x, double y, double z, double& i, double& j, double& k)
{
  float hypotenuse;
  float alpha;
  float beta;
  hypotenuse = sqrt( (x*x) + (y*y));
  alpha = asin( y/hypotenuse );
  beta = atan2(z,hypotenuse);
  i = cos(alpha);
  j = sin(alpha);
  k = sin(beta);
}

void PartialModelBase::computeRayFromPointToPoint(BoostMatrix pointA, BoostMatrix pointB, double& i, double& j, double& k)
{
  //TODO: This can be changed to ray = A-B;
  //double hypotenuse;
  //double alpha;
  //double beta;
  //double x,y,z;
  
  //WARNING !!!
  
  // B - A
  i = pointB(0,0) - pointA(0,0);
  j = pointB(1,0) - pointA(1,0);
  k = pointB(2,0) - pointA(2,0);
  
  
  /*
  x = pointB(0,0) - pointA(0,0);
  y = pointB(1,0) - pointA(1,0);
  z = pointB(2,0) - pointA(2,0);
  
  computeRayFromOriginToPoint(x,y,z,i,j,k);*/
}


bool PartialModelBase::registrationLowConstraint(long int n_occupied)
{
  long int registration_threshold;
  registration_threshold = 1;
  
  if (n_occupied >= registration_threshold)
    return true;
  else
    return false;
}


void PartialModelBase::setConfigFolder(std::string folder)
{
  configFolder = folder;
}

void PartialModelBase::setDataFolder(std::string folder)
{
  dataFolder = folder;
}


void PartialModelBase::sortCandidateViews()
{
  orderViewsHighToLow(candidateViews);
}


bool PartialModelBase::isInDOV(point3d origin, point3d point)
{
  float dist;
  dist = point.distance(origin);
  if(dist > minDOV && dist < maxDOV){
    return true;
  } else {
    return false;
  }
}


bool PartialModelBase::rayIntersectObjectSphere(const point3d raydir, const point3d rayorig)
{
  point3d L = rayorig - objectSphereCenter;
  double a = raydir.dot(raydir);
  double b = 2 * raydir.dot(L);
  double c = L.dot(L) - objRadius2;
  
  double discriminant;
  discriminant = b * b - 4 * a * c;
  
  if(discriminant < 0)
    return false;
  
  // TODO check whether the hit sphere is closer than maxDOV
  
  return true;
}

bool PartialModelBase::rayIntersectSphere(const point3d raydir, const point3d rayorig, const point3d sphere_center, double radius)
{
  point3d L = rayorig - sphere_center;
  double a = raydir.dot(raydir);
  double b = 2 * raydir.dot(L);
  double c = L.dot(L) - radius;
  
  double discriminant;
  discriminant = b * b - 4 * a * c;
  
  if(discriminant < 0)
    return false;
  
  // TODO check whether the hit sphere is closer than maxDOV
  
  return true;
}


// WARNING the view must have in w a view that is pointing to x
bool PartialModelBase::poitsToTheObject(ViewStructure& v)
{
   point3d ray(directorRay);
   ray.rotate_IP(v.w[5],v.w[4],v.w[3]);
   
   //std::cout << "rotated director ray" << ray << std::endl;
   
   point3d origin(v.w[0],v.w[1],v.w[2]);
   
   if(rayIntersectObjectSphere(ray, origin)){
     //std::cout << "points to the object :)" << std::endl;
      return true;
   }
   
   return false;
}


bool PartialModelBase::pointsToASphere(ViewStructure& v, double center_x, double center_y, double center_z, double radius)
{
   point3d ray(directorRay);
   ray.rotate_IP(v.w[5],v.w[4],v.w[3]);
   
   point3d origin(v.w[0],v.w[1],v.w[2]);
   
   point3d center(center_x,center_y,center_z);
   
   if(rayIntersectSphere(ray, origin, center, radius)){
    //cout << "points :)" << std::endl;
    return true;
   }
   
   return false;
}



void PartialModelBase::getOBB(double& x1, double& y1, double& z1, double& x2, double& y2, double& z2)
{
  x1 = x_cap_1;
  y1 = y_cap_1;
  z1 = z_cap_1;
  
  x2 = x_cap_2;
  y2 = y_cap_2;
  z2 = z_cap_2;
}


void PartialModelBase::evaluateCandidateViews(ViewList& views)
{
  std::cout << "Evaluating candidate views" << std::endl;
  ViewList::iterator it_v;
  it_v = views.begin();
  ViewStructure temp_view;
  
  while(it_v != views.end()){
    temp_view = *it_v;
    this->evaluateView(temp_view);
    //std::cout << temp_view << std::endl;
    *it_v = temp_view;
    it_v ++;
  }
}

void PartialModelBase::evaluateCandidateViews()
{
  std::list<ViewStructure>::iterator it_v;
  ViewStructure view;
  
  std::cout << "Evaluating candidate views:" << std::endl;
  it_v = candidateViews.begin();
  int i = 1;
  int n = candidateViews.size();
  while(it_v != candidateViews.end()){
    std::cout <<  i << "/" << n << std::endl; 
    this->evaluateView(*it_v);
    it_v ++; i++;
  }
}

