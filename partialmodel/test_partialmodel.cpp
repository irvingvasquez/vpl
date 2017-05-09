/*
 * 
 * 
Partial Model Library
Copyright (c) 2016, J. Irving Vasquez ivasquez@ccc.inaoep.mx
Consejo Nacional de Ciencia y Tecnología (CONACYT)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


//TODO: terminar este programa,
// Generar un conjunto de puntos en una esfera unitaria
// Generar una esfera de vistas
// 


#include <iostream>
#include <string>
#include "pmvoctree.h"

using namespace std;


static void show_usage(std::string name)
{
    std::cerr << "This program tests the partial model library \n"
	      << "Usage: " << name.c_str() << " [path_to_project_folder]"
              << std::endl;
}


/*
 * Demo program that test the partial model library.
 * The program sets a free space in the center and integrates a scan. Then several views are evaluated.
 * 
 */


int main(int argc, char **argv) {
    std::cout << "CONSEJO NACIONAL DE CIENCIA Y TECNOLOGIA (CONACYT) \n"
	      << " Partial Model Library \n"
	      << " J. Irving Vasquez-Gomez \n" 
	      << std::endl;
	      

    if (argc != 2) {
        show_usage(argv[0]);
        return 1;
    }
    
    std::string config_folder(argv[1]);
    std::string data_folder(config_folder);
    std::cout << "Config folder: " << config_folder.c_str() << std::endl;
    std::cout << "Data folder: " << data_folder.c_str() << std::endl;
    
    // output file
    std::string octree_file(argv[2]);
    std::cout << "octree_file: " << octree_file.c_str() << std::endl;

    return 0;
    
    // scan files
    std::string scan_file(data_folder + "scan_1_reg.dat");
    std::string origin_file(data_folder + "scan_origin_1.dat");
    
    
    //string file_rays("/home/irving/projects/Reconstructor3D/build/TestPioneerKinect/rays_kinect.dat");
    std::string file_rays(data_folder + "rays_0.dat");
    
    std::string cviews_file("/home/irving/projects/NBVPlanning_resp/test/test_octree/data/pointed_views.vs");
    //string ev_file("/home/irving/projects/OctreeNBVPlanner/build/TestPioneerKinect_mo/goals.dat");
        
    //string config_folder("../data_example/test_partialmodel");
    //tring data_folder("../data_example/test_partialmodel");
    
    std::string evaluated_views("/home/irving/projects/NBVPlanning/test/test_octree/data/evaluated_views_test.dat");
    
    PMVOctree octree;
    
    // You need to specify the route to the folder where the configuration file is
    octree.setConfigFolder(config_folder);
    // The routhe to a folder where log can be written
    octree.setDataFolder(data_folder);
    octree.init();
    octree.readRays(file_rays);
    
    octree.updateWithScan(scan_file, origin_file);
    
    octree.readCandidateViews(cviews_file);
    
    octree.evaluateCandidateViews();
    
    octree.savePartialModel(octree_file);
    
    return 0;
}












