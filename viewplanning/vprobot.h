/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VPROBOTBASE_H
#define VPROBOTBASE_H

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <stdlib.h>
#include <string>
#include <vector>
#include <math.h>
#include <list>

#include "vpdefinitions.h"
#include "vplutils.h"
#include <pmutils.h>
#include <vpfilereader.h>

#include <mrpt/math.h>
//#include <mrpt/base.h>

using namespace mrpt::math;

/**
 * Class for the description of a robot.
 * Units: mts y radians
 */
class vpRobot 
{
public:
vpRobot();

  /**
   * Initialices the robot, for example the comunication
   */
  virtual bool init();
  
  
  ///-------------------------------------------------------
  /// Abstract functions to be implemented in derived classes for physical robot
  ///-------------------------------------------------------
  
  /**
   * Returns the current configuration
   */
  virtual void getCurrentConfiguration(std::vector<double> &q)=0;
  
  /**
   * Replaces the current configuration with the given configuration
   * There is no movement
   * TODO for all derived classes
   */
  //virtual void overrideCurrentConfiguration(std::vector<double> &q);

  /**
  * The robot moves to a configuration.
  * The movement is directly. No collision is checked.
  * The robot moves by its own planning
  */
  virtual bool moveToConfiguration(std::vector<double> configuration)=0;
  
  /**
   * For physical robot
   */ 
  virtual bool setVelocities(std::vector<double> velocities)=0;
  
  /**
   * 
   */
  virtual float executeTrajectory(std::vector< std::vector<double> > controls, double delta_t, std::vector<double> goal_q)=0;
    
   /**
    * Stored movement in trajectory or path. Trajectory of path is executed depending on the robot capabilities
    * Requires a path and a trajectory stablished in pathToExecute and trajectoryToExecute
    */
  virtual float executeMovement()=0;
  
  /**
   * Translates the location of the robot
   */
  virtual void updateRobotLocalization(mrpt::poses::CPose3D transformation)=0;
   
  ///-------------------------------------------------------
  ///       Model implementations 
  ///-------------------------------------------------------
  
  
  /**
   * 
   */
  virtual void getHTMfromConfiguration(BoostMatrix &HTM, std::vector<double> q)=0;
  
  /**
   * 
   */
  virtual void getPoseFromConfiguration(mrpt::poses::CPose3D &pose, std::vector<double> q)=0;
  
  
  ///-------------------------------------------------------
  /// --- Generic functions to all derived classes implementations --------
  ///-------------------------------------------------------
  
  
  /**
   * Gets the transformation matrix for the end effector of the robot.
   * A point seen from the end effector frame reference must be multiplied by this HTM
   * @param HTM [out] Matrix of 4x4 
   */
  void getCurrentHTM(BoostMatrix &HTM);
  
  
  /**
   * Gets the end effector pose
   */
  void getCurrentPose(mrpt::poses::CPose3D &pose);
  
  
  /**
   * Used for convinience
   * Just call to getPoseFromConfiguration
   */
  void getPoseFromConfiguration(std::vector< double, std::allocator< double > > &pose, std::vector< double, std::allocator< double > > q);
  
  
  /**
   * Returns information about the robot
   */
  void getInfo(std::string &inf);

  
  /**
   * Calls repeately to moveToConfiguration
   */
  bool executePath();

  
  void setConfigFolder(std::string folder);
  
  void setDataFolder(std::string folder);
  
  void setLowerState(std::vector<double> state);
  
  void setUpperState(std::vector<double> state);
  
  void setStateDim(int dim);
  
  /**
   * Gets a random state between lower and upper states
   */
  void getRandomState(std::vector<double> &state);
  
  int getConfigDimension();
  
  int getInputDim();
  
  std::vector< std::vector<double> > pathToExecute;
  
  std::vector< std::vector<double> > trajectoryToExecute;
  
  double deltaT;
  
  std::vector< double > goalConfig;
  
 
protected:
  
  ///Configuration size of the robot
  int configurationDim;
  
  /// State Dimension
  int stateDim;
  
  /// input Dimension;
  int inputDim;
  
  std::vector<double> lowerState;
  
  std::vector<double> upperState;
  
  //vector<double> currentConfig;

  std::string info;
  
  bool ready;
  
  bool verbose;
  
  std::string configFolder;
  std::string dataFolder;
  std::string configFile;
  
private:

  // Generate a random number between 0 and 1
  // return a uniform number in [0,1].
  inline double unifRand();
  
  
  // Generate a random number in a real interval.
  // param a one end point of the interval
  // param b the other end of the interval
  // return a inform rand numberin [a,b].
  inline double unifRand(double a, double b);
  

};

#endif // VPROBOTBASE_H
