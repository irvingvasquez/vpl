#include <iostream>
#include <string>
#include <fstream>

#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

#include "pmvoctreehierarchicalrt.h"
#include "pmvoctree.h"
#include "viewstructure.h"
#include "pmvoctreeig.h"
	
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/option.hpp>
#include <boost/filesystem.hpp>


using namespace std;
using namespace octomap;

int main(int argc, char **argv) {
  
  if (argc != 2) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
        std::cout << "Usage is ./program <folder> \nNote: The folder must contain inside the folders config and data. \nExample: ./program /home/robot"; // Inform the user of how to use the program
        std::cin.get();
        exit(0);
  } 
    
  std::string main_folder(argv[1]);
    //cout << main_folder << std::endl;
    //exit(0);
    
  std::string config_folder(main_folder);
  config_folder.append("/config");
    
  std::string data_folder(main_folder);
  data_folder.append("/data");
  
  std::string rays_file(main_folder);
  rays_file.append("/data/rays_0.dat");
  
  //std::cout << "Hello, world!" << std::endl;
  double resolution = 0.02;
  double distance = 1.5;
     
  
  PMVOctree octree;
  PMVOctreeHierarchicalRT hrt_octree;
  PMVOctreeIG ig_octree;
  
  octree.setConfigFolder(config_folder);
  octree.setDataFolder(data_folder);
  octree.init();
  octree.readRays(rays_file);
  
  hrt_octree.setConfigFolder(config_folder);
  hrt_octree.setDataFolder(data_folder);
  hrt_octree.init();
  hrt_octree.readRays(rays_file);
  
  ig_octree.setConfigFolder(config_folder);
  ig_octree.setDataFolder(data_folder);
  ig_octree.init();
  ig_octree.readRays(rays_file);
  
  
  ViewList lista;
  std::string lista_fn("/home/irving/projects/nbvPlanning-1.1/VPL/build/views_hrt.vs");
  lista.read(lista_fn);
  
  
  /// Some troubles with load partial model
  //octree.loadPartialModel(pm_fn);
  //hrt_octree.loadPartialModel(pm_fn);
  
 std::ofstream myfile ("example.txt");
  if (!myfile.is_open())
  {
    std::cout << "unable to opel log file" << std::endl;
    return 0;
  }
  
  myfile << "--------- Octree ------------  HRT Octree ----------- " << std::endl;
  myfile << "unknown \toccupied \ttime \tunknown \toccupied \ttime" << std::endl;
  
  // insert scans
  int n_scans = 1;
  for (int i = 1; i<=n_scans; i++){
    std::string filename_scan(data_folder);
    std::string filename_origin(data_folder);
  
    std::stringstream file_string_s;
    file_string_s << "/scan_" << i << ".dat";

    filename_scan.append(file_string_s.str());
    
    std::stringstream file_string_o;
    file_string_o << "/scan_origin_" << i << ".dat";

    filename_origin.append(file_string_o.str());
    
    if ( !boost::filesystem::exists( filename_scan.c_str() ) )
    {
      std::cout << "There are no more scans to read." << std::endl;
      std::cout << "Looking for: " << filename_scan << std::endl;
      return 0;
    }
    
    octree.updateWithScan(filename_scan, filename_origin);
    hrt_octree.updateWithScan(filename_scan, filename_origin);
    ig_octree.updateWithScan(filename_scan, filename_origin);
    
    octree.savePartialModel("modeltested.ot");
    hrt_octree.savePartialModel("modeltested_hrt.ot");
    
    
    ViewList::iterator it;
    
    int rsl;
    double t1, t2;
    
    
    for(it= lista.begin(); it!=lista.end();it++){
      ViewStructure v = *it;
      
      clock_t start = clock();
      rsl = octree.evaluateView(v);
      clock_t ends = clock();
      
      t1 = (double) (ends - start) / CLOCKS_PER_SEC;
      
      if(rsl==FEASIBLE_VIEW && v.n_unknown > 16){
	cout << "Octree evaluation: Unknown " << v.n_unknown << " occupied " << v.n_occupied << " eval " << v.eval << " time " << t1 << std::endl;
	myfile << v.n_unknown << "\t" << v.n_occupied << "\t" << t1 ;
	
	start = clock();
	rsl = hrt_octree.evaluateView(v);
	ends = clock();
	
	t2 = (double) (ends - start) / CLOCKS_PER_SEC ;
	
	if(rsl==FEASIBLE_VIEW)
	  std::cout << "HRT Octree evaluation. Unknown " << v.n_unknown << " occupied " << v.n_occupied << " eval " << v.eval << " time " << t2  << std::endl;
	else
	  std::cout << "Unfeasible view.  Unknown " << v.n_unknown << " occupied " << v.n_occupied << " eval " << v.eval << " time " << t2 << std::endl;
	
	myfile  << "\t" << v.n_unknown << "\t" << v.n_occupied << "\t" << t2;
	
	start = clock();
	rsl = ig_octree.evaluateView(v);
	ends = clock();
	
	double t3 = (double) (ends - start) / CLOCKS_PER_SEC ;
	
	cout << "IG: " << v.eval << "\t time " << t3 << std::endl;
	
	myfile  << "\t" << v.eval << "\t" << t3 << std::endl;
	
	cout << "\n" ;
	//cout << "Continue press any key" << std::endl;
	//getchar();
      }
    }
  }
  
  myfile.close();
  
  return 0;
}
