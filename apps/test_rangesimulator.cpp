#include <iostream>
#include <string>

#include <pmvoctree.h>
#include <viewstructure.h>
#include <viewsynthesis.h>
#include <rssraytracingoctree.h>

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
  std::string rays_file(data_folder);
  rays_file.append("/sensor_rays.dat");
  
  
//   // ---------------- sensor ----------------------
//   // Simulation of the scan with octree
//   // RangeSensor *s = new RSSimulated();
  RangeSensor *s = new RSSRayTracingOCtree();
  s->setConfigFolder(config_folder);
  s->setDataFolder(data_folder);
  s->init();
  // Set of rays that defines the sensor R
  s->saveRays(rays_file);
    
  
//   // ---------------- RobotSensor ----------------
//   RobotSensor *rs = new RobSenNoTransformation(r, s);
//   rs->setSensorPose(0,0,0,-M_PI/2,0,-M_PI/2);
//   rs->setConfigFolder(config_folder);
//   rs->setDataFolder(data_folder);
//   rs->init();
//     
//     
//   // --------------- Partial Model ------------
//   PartialModelBase *partial_model = new PMVOctree();
//   partial_model->setConfigFolder(config_folder);
//   partial_model->setDataFolder(data_folder);
//   partial_model->init();
//   // Reads the sensor definition
//   partial_model->readRays(rays_file);  
       
  return 0;
}