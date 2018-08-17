#include <iostream>
#include <string>

#include <pmvoctree.h>
#include <viewstructure.h>
#include <viewsynthesis.h>
#include <vpfilereader.h>

using namespace std;

int main(int argc, char **argv) {
  std::cout << "Consejo Nacional de Ciencia y Tecnología" << std::endl;
  std::cout << "View planning library" << std::endl;
  
  std::cout << "Program: Updates the partial model with a scan \n\n"; // Inform the user of how to use the program
  
  if (argc != 2) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.

        std::cout << "Usage is: " << argv[0] <<  " <configuration_folder> \n"; // Inform the user of how to use the program
	std::cout << "For exaple: ./program /home/john/update_example \n"; // Inform the user of how to use the program
        exit(0);
  }

  // configure the configuration folders
  // The configuration of a problem is given in ini files
  std::string folder(argv[1]);
  
  std::string config_folder(folder);
  config_folder.append("/config");
  
  std::string data_folder(folder);
  data_folder.append("/data");
  
  cout << "Configuration folder: " << config_folder.c_str() << "\n";
  cout << "Data folder: " << data_folder.c_str() << "\n";
  
  vpFileReader filemanager;
  
  for(int i = 1; i<2; i++){  
    // create a partial partial model
    cout << "Iteration: " << i << endl;
    PMVOctree* octree = new PMVOctree();
    octree->setConfigFolder(config_folder);
    octree->setDataFolder(data_folder);
    octree->init();
    
    // save the current status
    octree->savePartialModel("octree_before_updating.ot");
    
    
    // To test the lib with blensor you have to configure blensor first
    
    // Then the sensor position is stored
    vector<float> pose(6);
    pose[0] = 14;
    pose[1] = -12;
    pose[2] = 10;
    pose[3] = 63.55;
    pose[4] = 0.62;
    pose[5] = 46.69;
    
    filemanager.saveVector<double>(pose, "sensor_pose.txt");
    
    // TODO: terminar la implementación 
    
    // read the scan
    string scan_file(data_folder + "/scan_1.dat");
    string origin_file(data_folder + "/scan_origin_1.dat");

    // update the partial model 
    octree->updateWithScan(scan_file, origin_file);
    octree->paintVoxels();
  
    // save the current representation  
    octree->savePartialModel("octree_after_updating.ot");
    delete octree;
    //getchar();
  }
       
  return 0;
}