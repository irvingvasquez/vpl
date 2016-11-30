
#include "pmvoctree.h"
#include "pmvoctreehierarchicalrt.h"
#include "vufobjectfilters.h"

#include <math.h>
#include <octomap/math/Utils.h>

#include "coctreevpl.h"

int main(int argc, char **argv) {

  
  
  std::string config_folder("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/config");
  std::string data_folder("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/data");
  std::string octree_file("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/data/test_cotree.ot");
  std::string file_rays("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/config/rays_0.dat");
  std::string scan_file("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/data/scan_1.dat");
  std::string origin_file("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/data/scan_origin_1.dat");
  std::string cviews_file("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/config/sphere.vs");
  
  
  std::vector< std::vector<double> > a;
  vpFileReader r;
  r.readDoubleCoordinates(scan_file, a);
  
  
  //PMVMultiOctree2 esferota;
  PMVOctreeHierarchicalRT esferota;
  //PMVOctree esferota;
  VUFObjectFilters *uf = new VUFObjectFilters();
  
  esferota.setConfigFolder(config_folder);
  esferota.setDataFolder(data_folder);
  esferota.init();
  esferota.readRays(file_rays);
  
  uf->setMinimunOverlap(5);
  
  esferota.setUtilityFunction(uf);
  
  esferota.updateWithScan(scan_file, origin_file);
  
  esferota.readCandidateViews(cviews_file);
  
  //esferota.evaluateCandidateViews();
  //esferota.saveEvaluations();
  
  //esferota.paintOccupiedInCapsule();
  //esferota.paintOccluded();
  esferota.savePartialModel(octree_file);
  std::string tris("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/data/occupied.raw");  
  std::string obst_filename("/home/irving/projects/nbvPlanning-1.1/problems/crv13_Uniform_RT/data/Obst"); 
  esferota.saveObjectAsRawT(tris);
  esferota.saveObjectAsObst(obst_filename);
  
  return 0;
}