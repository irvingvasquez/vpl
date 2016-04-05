#include <iostream>
#include <string>


#include "pmvoctree.h"

using namespace std;

int main(int argc, char **argv) {
    string scan_file("/home/irving/projects/nbvPlanning-1.1/test/EVA_Kat_horizontal/data/scan_1_reg.dat");
    string origin_file("/home/irving/projects/nbvPlanning-1.1/test/EVA_Kat_horizontal/data/scan_origin_1.dat");
    string octree_file("/home/irving/projects/nbvPlanning-1.1/test/EVA_Kat_horizontal/data/octree_color_test.ot");
    
    //string file_rays("/home/irving/projects/Reconstructor3D/build/TestPioneerKinect/rays_kinect.dat");
    string file_rays("/home/irving/projects/NBVPlanning_resp/test/test_octree/data/rays_0.dat");
    
    string cviews_file("/home/irving/projects/NBVPlanning_resp/test/test_octree/data/pointed_views.vs");
    //string ev_file("/home/irving/projects/OctreeNBVPlanner/build/TestPioneerKinect_mo/goals.dat");
        
    string config_folder("/home/irving/projects/nbvPlanning-1.1/test/EVA_Kat_horizontal/config");
    string data_folder("/home/irving/projects/nbvPlanning-1.1/test/EVA_Kat_horizontal/data");
    
    string evaluated_views("/home/irving/projects/NBVPlanning/test/test_octree/data/evaluated_views_test.dat");
    
    PMVOctree octree;
    
    octree.setConfigFolder(config_folder);
    octree.setDataFolder(data_folder);
    octree.init();
    octree.readRays(file_rays);
    
    octree.updateWithScan(scan_file, origin_file);
    
    octree.readCandidateViews(cviews_file);
    
    octree.evaluateCandidateViews();
    
    octree.savePartialModel(octree_file);
    
    
    
    //exit(0);
    
//     
//     
//     octree.paintOccupiedInCapsule();
//     
//     octree.readCandidateViews(cviews_file);
//     
//     //octree.testEvaluation();
//     octree.evaluateCandidateViews();
//     
//     octree.saveEvaluatedViews(evaluated_views);
//     octree.saveEvaluations();
//     
//     octree.savePartialModel(octree_file);
    
    return 0;
}