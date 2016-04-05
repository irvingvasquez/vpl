
#include "pmvoctree.h"

int main(int argc, char **argv) {
 string config_folder("/home/irving/projects/nbvPlanning-1.1/test/robot_silla_sur_eu_obstaculos/config");
 string data_folder("/home/irving/projects/nbvPlanning-1.1/test/robot_silla_sur_eu_obstaculos/data");
  
 if ( argc != 3 ) // argc should be 3 for correct execution
    // We print argv[0] assuming it is the program name
    cout<<"usage: "<< argv[0] <<" <filename>  <filename>\n";
 else {
    string octree_file(argv[1]);
    string output_file(argv[2]);
    PMVOctree oc;
  
    oc.setConfigFolder(config_folder);
    oc.setDataFolder(data_folder);
    oc.init();
    
    oc.loadPartialModel(octree_file);
    
    //oc.paintOccupiedInCapsule();
    cout << "Ready to paint and save" << endl;
    oc.savePartialModel(output_file);
 }
 return 0;
}