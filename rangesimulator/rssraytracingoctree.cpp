
#include "rssraytracingoctree.h"


RSSRayTracingOCtree::RSSRayTracingOCtree()
{

}


bool RSSRayTracingOCtree::init()
{
  std::cout << "--- Range Sensor Simulated ---" << std::endl;
  RangeSensor::init();
  
  std::string rays_file;
  rays_file.assign(configFolder);
  rays_file.append("/sensorRays.dat");
  saveRays(rays_file);  
  
  rangeSensor.setConfigFolder(configFolder);
  rangeSensor.setDataFolder(dataFolder);
  rangeSensor.init();
  
  std::string simulated_environment_f;
  simulated_environment_f.assign(dataFolder);
  simulated_environment_f.append("/simulated_environment.ot");
  rangeSensor.save(simulated_environment_f);
}


long int RSSRayTracingOCtree::getPoints(std::vector< mrpt::poses::CPoint3D >& points)
{
  //return RSSimulated::getPoints(points);
  //cout << "Range Sensor Simulated, file:" << scan_file.c_str() << std::endl;
  
  std::string out_file(dataFolder);
  out_file.append("/current_scan.dat");
  
  rangeSensor.takeAndSaveScan(currentView, out_file);
  
  double x,y,z;
  vpFileReader reader;
  std::vector< std::vector<double> > data;
  std::vector< std::vector<double> >::iterator it_data;
  
  /// try 3 times
  int intentos = 0;
  while( reader.readDoubleCoordinates(out_file, data) == false && intentos < 3){
     intentos ++;
  }
    
  //reader.readDoubleCoordFromPCD(filename, data, header);
  std::cout << data.size() << " readed points." << std::endl;
  
  /// Add points to point cloud
  mrpt::poses::CPoint3D *pt;
  points.clear();
  
  for (it_data = data.begin(); it_data!= data.end(); it_data++){
    x = (*it_data)[0];
    y = (*it_data)[1];
    z = (*it_data)[2];
    
    x = x;
    y = y;
    z = z;
    
    pt = new mrpt::poses::CPoint3D(x,y,z);
    points.push_back(*pt);
    delete pt;
  }
}
