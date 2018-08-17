#include <iostream>
#include <vp.h>
#include <pmvoctree.h>
#include <pmvoctreeig.h>
#include <pmvoctreeigkriegel12.h>
#include <rssraytracingoctree.h>

#include "nbvplannerrgflt.h"
#include "robotmobile.h"
#include "robotarmmdh.h"
#include "robotcomposed.h"

int main(int argc, char **argv) {
    
    std::cout << "Consejo Nacional de Ciencia y Tecnología" << std::endl;
    std::cout << "Instituto Nacional de Astrofísica Óptica y Electrónica" << std::endl;
    std::cout << "Centro de Innovación y Desarrollo Tecnológico en Cómputo" << std::endl;
    
    //std::cout << "Hello, world!" << std::endl;
    double resolution = 0.02;
    double distance = 1.5;
    
    // ./program folder 
    
    if (argc != 3) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
        std::cout << "Usage is ./program <folder> <ref_model>\n Note: The folder must contain inside the folders config and data. \n<ref_model> describes the points of the target model in order to calculate the coverage.";
	std:: cout << "\nExample:  ./test_sim_rrtnbv ../../data_example/rrt_nbvs ../../data_example/rrt_nbvs/config/dragon_ref_smalltable_nobase.dat\n"; 
	// Inform the user of how to use the program
        exit(0);
    } 
    
    
    std::string main_folder(argv[1]);
    std::string ref_model(argv[2]);
    
    std::string config_folder(main_folder);
    config_folder.append("/config");
    
    std::string data_folder(main_folder);
    data_folder.append("/data");
    
    std::string rays_file(main_folder);
    rays_file.append("/data/rays_0.dat");
    
   
    // ---------------- robot ---------------------
    vpRobot *r = new vprFreeFlyer;
    r->init();
    
    // ---------------- sensor ---------------------
    RangeSensor *s = new RSSRayTracingOCtree();
    s->setConfigFolder(config_folder);
    s->setDataFolder(data_folder);
    s->init();
    s->saveRaysForResolution(rays_file, resolution, distance);

    
    // ---------------- RobotSensor ----------------
    RobotSensor *rs = new RobSenNoTransformation(r, s);
    rs->setSensorPose(-0.030,0.078,-0.135,M_PI,0,0);
    //rs->setSensorPose(0,0,0,0,0,0);
    rs->setConfigFolder(config_folder);
    rs->setDataFolder(data_folder);
    rs->init();
    
    
    // --------------- Partial Model ------------
    //PartialModelBase *partial_model = new PMVOctree();
    PartialModelBase *partial_model = new PMVOctreeIG();
    //PartialModelBase *partial_model = new PMVOctreeIGKriegel12();
    partial_model->setConfigFolder(config_folder);
    partial_model->setDataFolder(data_folder);
    partial_model->init();
    partial_model->readRays(rays_file);
    
    
    // --------------- NBV Planner ------------
    NBVPlanner *planner = new NBVPlannerExtendTree(rs, partial_model);
    planner->setConfigFolder(config_folder);
    planner->setDataFolder(data_folder);
    planner->init();
        
    // ---------------- Reconstructor
    R3DDirectPositioning *rec = new R3DDirectPositioning(rs, planner);
    rec->setPartialModel(partial_model);
    rec->setConfigFolder(config_folder);
    rec->setDataFolder(data_folder);
    // specify the file that will be the reference to calculate the coverage
    rec->ref_obj_pts_fn.assign(ref_model);
    rec->tar_pts_fn.assign(partial_model->object_points_filename);
    
    rec->init(); 
    rec->solveReconstruction();
    
    delete r;
    delete s;
    delete rs;
    delete partial_model;
    delete planner;
    delete rec;
       
    return 0;    
}