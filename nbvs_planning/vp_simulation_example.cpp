//#include <iostream>
#include <pmvoctree.h>
#include <rssraytracingoctree.h>
#include <vp.h>
#include "nbvplannerrgflt.h"

#include "robotmobile.h"
#include "robotarmmdh.h"
#include "robotcomposed.h"


//TODO: build a simplified example

// TODO move sensor definition to the partial model lib

/** This program performs a reconstruction with the Sabina Mobile robot
 */

int main(int argc, char **argv) {
    std::cout << "Consejo Nacional de Ciencia y Tecnología" << std::endl;
    std::cout << "Instituto Nacional de Astrofísica Óptica y Electrónica" << std::endl;
    
    // octree resolution
    double resolution = 0.02;
    
    // 
    double distance = 1.5;
    
    // ./program folder 
    
    if (argc != 3) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
        std::cout << "Usage is ./program <folder> <ref_model>\n Note: The folder must contain inside the folders config and data. \n<ref_model> describes the points of the target model in order to calculate the coverage\nExample: ./program /home/robot"; // Inform the user of how to use the program
        std::cin.get();
        exit(0);
    } 
    
    std::string main_folder(argv[1]);
    std::string ref_model(argv[2]);
    
    //cout << main_folder << std::endl;
    
    std::string config_folder(main_folder);
    config_folder.append("/config");
    
    std::string data_folder(main_folder);
    data_folder.append("/data");
    
    std::string rays_file(main_folder);
    rays_file.append("/data/rays_0.dat");

    
    // ------------------- Robot ---------------------------
    /*
     * Definition of the robot.
     * In this example a composed robot is defined.
     */
    RobotMobile *mobile = new RobotMobileDummie();
    mobile->setConfigFolder(config_folder);
    mobile->setDataFolder(data_folder);
    mobile->init();
    mrpt::poses::CPose3D pose_mobile(0,0,0,0,0,0);
    
    RobotArmMDH *arm = new RobotArmMDHDummie();
    arm->setConfigFolder(config_folder);
    arm->setDataFolder(data_folder);
    arm->init();
    
    //mrpt::poses::CPose3D pose_arm(0.20,0,0.50,0,0,0); 
    mrpt::poses::CPose3D pose_arm(0.23, 0, 0.775, 0, 1.579, 0); 
        
    RobotComposed *rcomp = new RobotComposedEVA();
    rcomp->setConfigFolder(config_folder);
    rcomp->setDataFolder(data_folder);
    rcomp->init();
    rcomp->addRobot(pose_mobile, mobile);
    rcomp->addRobot(pose_arm, arm);
    
    vpRobot *r = rcomp;
    r->init();
    
    
    // ---------------- sensor ----------------------
    // Simulation of the scan with blensor
    // RangeSensor *s = new RSSimulated();
    RangeSensor *s = new RSSRayTracingOCtree();
    s->setConfigFolder(config_folder);
    s->setDataFolder(data_folder);
    s->init();
    s->saveRaysForResolution(rays_file, resolution, distance);
    
    
    // ---------------- RobotSensor -----------------
    // Since we are using blensor the scan is saved in the global reference frame
    // So we do not need to transform the scan by the robot pose.
    RobotSensor *rs = new RobSenNoTransformation(r, s);
    rs->setSensorPose(-0.030,0.078,-0.135,M_PI,0,0);
    //rs->setSensorPose(0,0,0,0,0,0);
    rs->setConfigFolder(config_folder);
    rs->setDataFolder(data_folder);
    rs->init();
    
    
    // --------------- Partial Model ------------
    // We use a probabilistic octree as partial model
    PartialModelBase *partial_model = new PMVOctree();
    
    partial_model->setConfigFolder(config_folder);
    partial_model->setDataFolder(data_folder);
    partial_model->init();
    partial_model->readRays(rays_file);
    
    
    // --------------- NBV Planner ------------
    //NBVPlanner *planner = new NBVPlannerRGFlt(rs, partial_model);
    NBVPlanner *planner = new NBVPlannerRGFltOneGoal(rs, partial_model); 
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