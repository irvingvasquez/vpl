#include <iostream>
#include <string>

#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

#include "coctreevpl.h"

using namespace std;
using namespace octomap;

int main(int argc, char **argv) {
    
  //##############################################################     

  COctreeVPL tree (0.02);  

  point3d origin (0.01f, 0.01f, 0.02f);
  point3d point_on_surface (1.01f, 0.01f, 0.01f);

  std::cout << "generating spherical scan at " << origin << " ..." << std::endl;

  Pointcloud cloud;

  for (int i=-50; i<51; i++) {
    for (int j=-50; j<51; j++) {
      point3d rotated = point_on_surface;
      rotated.rotate_IP(0, DEG2RAD(i*0.5), DEG2RAD(j*0.5));
      cloud.push_back(rotated);
    }
  }  

  // insert in global coordinates:
  tree.insertScan(cloud, origin);
  
  
  // insert free space;
  point3d p1(-1,-1,-1);
  //point3d p1()
  
  
  KeyRay rr1;
  std::vector<point3d> v1;
  KeyRay rr2;
  std::vector<point3d> v2;
  tree.computeRay(origin, point_on_surface, v1);
  tree.setResolution(tree.getResolution()*4);
  tree.computeRay(origin,point_on_surface, v2);
  
  std::cout << "v1 size: " << v1.size() << std::endl;
  std::cout << "v2 size: " << v2.size() << std::endl;
 
  point3d fin;
  tree.expand();
  switch( tree.castRayVPLHierarchical(origin, point_on_surface, fin, false, -1.0, tree.getTreeDepth()-2)){
    case VXL_OCCUPIED:
      std::cout << "occupied" << std::endl;
      break;  
    
    case VXL_UNKNOWN:
      std::cout << "Unknown" << std::endl;
      break;
    case VXL_OCCUPIED_TOUCHED:
      std::cout << "occupied touched" << std::endl;
      break;

  }
  
    switch( tree.castRayVPLHierarchical(origin, point_on_surface, fin, false, -1.0, tree.getTreeDepth()-2)){
    case VXL_OCCUPIED:
      std::cout << "occupied" << std::endl;
      break;  
    
    case VXL_UNKNOWN:
      std::cout << "Unknown" << std::endl;
      break;
    case VXL_OCCUPIED_TOUCHED:
      std::cout << "occupied touched" << std::endl;
      break;
  }
  
  
  std::cout << "writing to spherical_scan.bt..." << std::endl;
  tree.writeBinary("spherical_scan.bt");
  
  return 0;
}
