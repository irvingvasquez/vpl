/*
    Copyright 2013 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef ROBOTSENSOR_H
#define ROBOTSENSOR_H

#include "vprobot.h"
#include "rangesensor.h"

#include <string>
#include <mrpt/poses.h>


/**
 * Clase base that represents a robot with a range sensor mounted on it. 
 * With this we can perform scans and save them to the Robot coordinate reference.
 * Requires a robot and a sensor
 */

class RobotSensor
{

public:
RobotSensor(vpRobot *r, RangeSensor *s);
    
    /** 
     * Performs a scan at current position
     */
    virtual long int performScan();
    
    virtual bool init();
    
    void setConfigFolder(std::string folder);
  
    void setDataFolder(std::string folder);
    
    void getHTMfromRobot(BoostMatrix &htmRobot);
    
    void setSensor(RangeSensor *rs);
  
    /** IMPORTANT:
    * yaw is rotation over Z
    * pitch is a rotation over Y
    * roll is a rotation over X
    * radians
    */
    void setSensorPose(double x, double y, double z, double yaw, double pitch, double roll);
    
    /**
    * Saves the last point cloud. Adds the folder and the extension.
    */
    bool saveLastPointCloud(std::string &file_name, double factor);
    
    /**
     * Saves the last point cloud in a raw text file (mts). 
     * The file will have the name "scan_[number_of_scan].dat" and it will be saved at the folder specified by problem Adress.
     */
    bool saveLastPointCloudMts(std::string &file_name);
    
    /**
     * Saves the origins of the scans. Adds the folder and extension automaticly.
     */
    bool saveSensorTrajectory(std::string &file_name, double factor);
    
    bool saveSensorTrajectoryMts(std::string &file_name);
    
    
    void getHTMfromSensor(BoostMatrix &htmSensor);
    
    /**
     * NOTE: this is only a patch!!!!
     * Receives a set of configurations in views format. Only the vector q is taken into account.
     * Returns the same set of views but with HTM filled.
     * You can also use a getViewsFromComfigurations
     */
    void getViewsFromComfigurations(std::list<ViewStructure> &views);
    
    
    /**
     * Receives a set of configurations and returns a set of views.
     * A view include besides the configuration, the Homogenous Transformation Matrix in Workspace
     */
    void getViewsFromComfigurations(std::list<ViewStructure> &views, std::list< std::vector<double> > configurations);
    
    /**
    * Gets a random state between lower and upper states
    */
    void getRandomState(std::vector<double> &state);

    /**
     * Gets a View structure from a state
     */
    void getViewFromState(ViewStructure &view, const std::vector<double> state);
    
    bool moveToConfiguration(std::vector<double> configuration);
    
    bool setPathToExecute(std::vector< std::vector<double> > path);
    
    bool setTrajectory(std::vector< std::vector<double> > trajectory, double delta_t);
    
    bool setGoalConfiguration(std::vector<double> goal_q);
    
    //bool executeTrajectory(std::vector< std::vector<double> > controls, double delta_t, std::vector<double> goal_q);
    
    float executeMovement();
    
    void getCurrentConfiguration(std::vector<double> &configuration);
    
    /// updates the configuration of the robot with a transformation. Usually this trasnformation is obtained by the Registration process.
    bool updateRobotLocalization(mrpt::poses::CPose3D transformation);
    
   /** 
   *	verificar si el rayo director del sensor intersecta con el robot en determinada configuración
   */
    virtual bool autoOcclusion(std::vector<double> q);
    
   /**
    */
   virtual bool pointsToTheRobot(ViewStructure view, std::vector<double> q);
    
protected:
  RangeSensor *sensor;
  
  vpRobot *robot;
  
  std::string configFolder;
  std::string dataFolder;
  
  /** IMPORTANT:
     * yaw is rotation over Z
     * pitch is a rotation over Y
     * roll is a rotation over X
     */
  std::vector<double> sensorPose;
  
    /**
    * Sensor pose respect to the Robot
    */
    mrpt::poses::CPose3D SensorPose;
    
    /**
    * The last captured point cloud.
    */
    std::list<mrpt::poses::CPoint3D> pointCloud;
    
    /**
    * Counter for the captured point clouds.
    */
    uint32_t pointCloudIdCounter;
    
    /**
    * Trajectory of the sensor (if there was a sweeping) or origin in cases of a 3D sensor
    */
    std::list<mrpt::poses::CPoint3D> SensorTrajectory;
    
    bool sensorReady;
    bool robotReady;
    
    void convertPointsToRobotReference(std::vector<mrpt::poses::CPoint3D> &points,
				  std::vector<mrpt::poses::CPoint3D> &result);
    
    void addPointsToPointCloud(std::vector<mrpt::poses::CPoint3D> &points,
				      std::vector<mrpt::poses::CPoint3D> &origins,
				      bool clear = true );
    
    inline double dotProduct(mrpt::poses::CPoint3D a, mrpt::poses::CPoint3D b);
  
};

#endif // ROBOTSENSOR_H
