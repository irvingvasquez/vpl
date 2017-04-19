
#include "pmvoctree.h"

int main(int argc, char **argv) {
 std::string config_folder("/home/irving/projects/nbvPlanning-1.1/test/robot_silla_sur_eu_obstaculos/config");
 std::string data_folder("/home/irving/projects/nbvPlanning-1.1/test/robot_silla_sur_eu_obstaculos/data");
  
 if ( argc != 3 ) // argc should be 3 for correct execution
    // We print argv[0] assuming it is the program name
    std::cout<<"usage: "<< argv[0] <<" <filename>  <filename>\n";
 else {
    std::string octree_file(argv[1]);
    std::string output_file(argv[2]);
    PMVOctree oc;
  
    oc.setConfigFolder(config_folder);
    oc.setDataFolder(data_folder);
    oc.init();
    
    oc.loadPartialModel(octree_file);
    
    //oc.paintOccupiedInCapsule();
    std::cout << "Ready to paint and save" << std::endl;
    oc.savePartialModel(output_file);
 }
 return 0;
}