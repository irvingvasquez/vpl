#include "vufobjectfilters.h"
#include "pmvoctreehierarchicalrt.h"

int main(int argc, char **argv) {
  string config_folder("/home/irving/projects/NBVPlanning/test/test_octree/config");
  string data_folder("/home/irving/projects/NBVPlanning/test/test_octree/data");
  string octree_file("/home/irving/projects/NBVPlanning/test/test_octree/data/multi_test.ot");
  string file_rays("/home/irving/projects/NBVPlanning/test/test_octree/data/rays_0.dat");
  string scan_file("/home/irving/projects/NBVPlanning/test/test_octree/data/scan_0.dat");
  string origin_file("/home/irving/projects/NBVPlanning/test/test_octree/data/scan_origin_0.dat");
  string cviews_file("/home/irving/projects/NBVPlanning/test/test_octree/data/pointed_views.vs");
    
  
  //PMVMultiOctree2 esferota;
  PMVOctreeHierarchicalRT esferota;
  VUFObjectFilters *uf = new VUFObjectFilters();
  
  esferota.setConfigFolder(config_folder);
  esferota.setDataFolder(data_folder);
  esferota.init();
  esferota.readRays(file_rays);
  
  uf->setMinimunOverlap(5);
  
  esferota.setUtilityFunction(uf);
  
  esferota.updateWithScan(scan_file, origin_file);
  
  esferota.readCandidateViews(cviews_file);
  
  esferota.evaluateCandidateViews();
  esferota.saveEvaluations();
  
  //esferota.paintOccupiedInCapsule();
  //esferota.paintOccluded();
  esferota.savePartialModel(octree_file);
  
  delete uf;
}