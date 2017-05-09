
#include <vp.h>
#include <rssraytracingoctree.h>
#include <pmvoctree.h>

int main(int argc, char **argv) { 
  
    std::cout << "Consejo Nacional de Ciencia y Tecnología" << std::endl;
    std::cout << "Instituto Nacional de Astrofísica Óptica y Electrónica" << std::endl;
    std::cout << "Centro de Innovación y Desarrollo Tecnológico en Cómputo" << std::endl;
    
    // octree resolution
    double resolution = 0.02;
    
    // Best scanning distance of the sensor
    double distance = 1.5;
    
    if (argc != 3) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
        std::cout << "Usage is ./program <folder> <ref_model>\n Note: The folder must contain inside the folders config and data. \n<ref_model> describes the points of the target model in order to calculate the coverage\nExample: ./program /home/robot"; 
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
    
    
    // ------------------- Robot ---------------------------
    /*
     * Definition of the robot.
     */
    vpRobot *r = new vprFreeFlyer();
    r->init();
    
    
    // ---------------- sensor ----------------------
    // Simulation of the scan with octree
    // RangeSensor *s = new RSSimulated();
    RangeSensor *s = new RSSRayTracingOCtree();
    s->setConfigFolder(config_folder);
    s->setDataFolder(data_folder);
    s->init();
    // Set of rays that defines the sensor R
    s->saveRaysForResolution(rays_file, resolution, distance);
    
    
    // ---------------- RobotSensor ----------------
    RobotSensor *rs = new RobSenNoTransformation(r, s);
    rs->setSensorPose(0,0,0,-M_PI/2,0,-M_PI/2);
    rs->setConfigFolder(config_folder);
    rs->setDataFolder(data_folder);
    rs->init();
    
    
    // --------------- Partial Model ------------
    PartialModelBase *partial_model = new PMVOctree();
    partial_model->setConfigFolder(config_folder);
    partial_model->setDataFolder(data_folder);
    partial_model->init();
    // Reads the sensor definition
    partial_model->readRays(rays_file);
    
    
    // --------------- NBV Planner ------------
    NBVPlanner *planner = new WorkspaceNBVPlanner(rs, partial_model);
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