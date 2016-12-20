/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

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


#include "robotcomposed.h"

void RobotComposed::addRobot(poses::CPose3D pose, vpRobot* robot)
{
  robotPoses.push_back(pose);
  robots.push_back(robot);
  configurationDim += robot->getConfigDimension();
  numberOfRobots ++;
}


void RobotComposed::getPoseFromConfiguration(mrpt::poses::CPose3D& pose, std::vector< double, std::allocator< double > > q)
{
  mrpt::poses::CPose3D end_pose;
  mrpt::poses::CPose3D total_pose(0,0,0,0,0,0);
  double value;
  int l=0;
  std::vector<double> partial_q;
  
  //para cada robot
  for(int i =0;i<numberOfRobots; i++){
    //obtener su configuraion
    partial_q.clear();
    for(int k=0; k<robots[i]->getConfigDimension(); k++){
      value = q[l];
      l++;
      partial_q.push_back(value);
    }
    //obtener su pose
    robots[i]->getPoseFromConfiguration(end_pose, partial_q);
//    std::cout << end_pose << std::endl;
//    std::cout << robotPoses[i] << std::endl;
    total_pose = total_pose + robotPoses[i] + end_pose;
  }
  
  pose = total_pose;
//  std::cout << total_pose << std::endl;
}


void RobotComposed::getHTMfromConfiguration(BoostMatrix& HTM, std::vector< double, std::allocator< double > > q)
{
  //cout << "Robot composed: " << std::endl;
  mrpt::poses::CPose3D tp;
  
  CMatrixDouble44 R;
  
  this->getPoseFromConfiguration(tp, q);
  
  tp.getHomogeneousMatrix(R);
  
  BoostMatrix Matrix(4,4);
  for(int j=0; j<4; j++){
    for(int k = 0; k<4; k++){
      Matrix(j,k) = R(j,k);
    }
  }
  
  HTM = Matrix;
  //cout << "R" << R << std::endl;
  //cout << "HTM" << std::endl << HTM << std::endl;
}


bool RobotComposed::moveToConfiguration(std::vector< double, std::allocator< double > > configuration)
{
  std::vector<double> partial_q;
  int l =0;
  double value;
  
  for(int i =0;i<numberOfRobots; i++){
    partial_q.clear();
    for(int k=0; k<robots[i]->getConfigDimension(); k++){
      value = configuration[l];
      l++;
      partial_q.push_back(value);
    }
    robots[i]->moveToConfiguration(partial_q);
  }
  
  //CPose3D pose;
  //this->getPoseFromConfiguration(pose, configuration);
  //cout << "Composed end-effector pose" << pose << std::endl;
  
  return true;
}


float RobotComposed::executeTrajectory(std::vector< std::vector< double > > controls, double delta_t, std::vector< double > goal_q)
{
  
  std::vector< std::vector< std::vector<double> > > separated_controls(numberOfRobots);
  std::vector< std::vector< std::vector<double> > > separated_paths(numberOfRobots);
  
  //TODO separate goals
  std::vector< std::vector<double> > separated_goals(numberOfRobots);
  
  std::vector<double> partial_u;
  std::vector<double> partial_q;
  
  double velocity;
  double value;
  
  std::vector< std::vector<double> >::iterator it_control;
  std::vector< std::vector<double> >::iterator it_config;
  
  // separate controls
  // para cada control
  for(it_control = controls.begin(); it_control != controls.end(); it_control++){
    int l =0;
    // PMUtils::printVector(*it_control);    
    if(it_control->size()!= 0){
    
      // para cada robot 
      for(int i =0;i<numberOfRobots; i++){
	partial_u.clear();
	
	// para cada dimesion u del robot
	for(int k=0; k<robots[i]->getInputDim(); k++){
	  velocity = (*it_control)[l];
	  l++;
	  partial_u.push_back(velocity);
	}
	// PMUtils::printVector(partial_u);
	
	separated_controls[i].push_back(partial_u);
      }
      
    }else{
    //cout << "empty control " << std::endl;
    }
  }
  

  
  
  
  //separate paths
  // para cada configuration
  for(it_config = pathToExecute.begin(); it_config != pathToExecute.end(); it_config++){
    int l =0;
    // PMUtils::printVector(*it_control);    
    if(it_config->size()!= 0){
    
      // para cada robot 
      for(int i =0;i<numberOfRobots; i++){
	partial_q.clear();
	
	// para cada dimesion u del robot
	for(int k=0; k<robots[i]->getConfigDimension(); k++){
	  value = (*it_config)[l];
	  l++;
	  partial_q.push_back(value);
	}
	// PMUtils::printVector(partial_u);
	
	separated_paths[i].push_back(partial_q);
      }
      
    }else{
    //cout << "empty control " << std::endl;
    }
  }
  
  
  // excecute controls
  // TODO: execute in parallel for each robot
  // para cada robot 
  for(int i =0;i<numberOfRobots; i++){
    if(i==1)
    {
      robots[i]->pathToExecute = separated_paths[i];
      robots[i]->executePath();
      //TODO
    }
    else{
      robots[i]->executeTrajectory(separated_controls[i], delta_t, goal_q); // here goals must have the goal for the right robot
    }
  }
  
  return true;
}


float RobotComposed::executeMovement()
{
  return true;
}


bool RobotComposed::setVelocities(std::vector< double > velocities)
{
  //TODO
  return false;
}



bool RobotComposed::init()
{
  vpRobot::init();
  

}

RobotComposed::RobotComposed():vpRobot()
{
  numberOfRobots = 0;
  
}


void RobotComposed::updateRobotLocalization(mrpt::poses::CPose3D transformation)
{
  //for(int i = 0 ; i<numberOfRobots ; i++){
  //  robots[i]->updateRobotLocalization(transformation);;
  //}
  
  if(numberOfRobots>0){
    robots[0]->updateRobotLocalization(transformation);
  }
  
}



void RobotComposed::getCurrentConfiguration(std::vector< double >& q)
{
  q.clear();
  std::vector<double> q_r;
  for(int i = 0 ; i<numberOfRobots ; i++){
    q_r.clear();
    robots[i]->getCurrentConfiguration(q_r);
    q.insert(q.end(), q_r.begin(), q_r.end());
  }
}



//*********************************************************************************+
float RobotComposedEVA::executeMovement()
{
  std::vector< std::vector< std::vector<double> > > separated_controls(numberOfRobots);
  std::vector< std::vector< std::vector<double> > > separated_paths(numberOfRobots);
  
  //TODO separate goals
  std::vector< std::vector<double> > separated_goals(numberOfRobots);
  
  std::vector<double> partial_u;
  std::vector<double> partial_q;
  
  double velocity;
  double value;
  
  std::vector< std::vector<double> >::iterator it_control;
  std::vector< std::vector<double> >::iterator it_config;
  
  //TODO: remove only for display
  std::vector< std::vector<double> > path;
  std::vector<double> xt;
  
  
  // separate controls
  // para cada control
  for(it_control = trajectoryToExecute.begin(); it_control != trajectoryToExecute.end(); it_control++){
    int l =0;
    //PMUtils::printVector(*it_control);    
    if(it_control->size()!= 0){
    
      // para cada robot 
      for(int i =0;i<numberOfRobots; i++){
	partial_u.clear();
	
	// para cada dimesion u del robot
	for(int k=0; k<robots[i]->getInputDim(); k++){
	  velocity = (*it_control)[l];
	  l++;
	  partial_u.push_back(velocity);
	}
	// PMUtils::printVector(partial_u);
	
	separated_controls[i].push_back(partial_u);
      }
      
    }else{
      //cout << "Empty control " << std::endl;
    }
  }
  
  
  
  
  
  
  //separate paths
  //cout << "separate paths" << std::endl;
  // para cada configuration
  for(it_config = pathToExecute.begin(); it_config != pathToExecute.end(); it_config++){
    int l =0;
    //PMUtils::printVector(*it_config);    
    if(it_config->size()!= 0){
    
      // para cada robot 
      for(int i =0;i<numberOfRobots; i++){
	partial_q.clear();
	
	// para cada dimesion u del robot
	for(int k=0; k<robots[i]->getConfigDimension(); k++){
	  value = (*it_config)[l];
	  l++;
	  partial_q.push_back(value);
	}
	// PMUtils::printVector(partial_u);
	
	separated_paths[i].push_back(partial_q);
      }
      
    }else{
      //cout << "Empty configuration " << std::endl;
    }
  }
  
  
  
  
  //------------- excecute controls ----------------------
  std::cout << "Excecuting controls..." << std::endl;
  clock_t begin = clock();

  double microseconds = deltaT * 1000000;
  
  // WARNING this is only for robot EVA
  
  std::vector<double> cero_vel_mobile;
  cero_vel_mobile.push_back(0.0);
  cero_vel_mobile.push_back(0.0);
  // para cada control u_i
  for(int i=0; i<trajectoryToExecute.size(); i++){
    std::cout << "control " << i << "/" << trajectoryToExecute.size() << std::endl;
    
    // para cada robot r_j
    for(int j = 0; j<numberOfRobots; j++){
      if(j==1){
	robots[j]->moveToConfiguration(separated_paths[j][i]);
      }
      else {
	  robots[j]->setVelocities(separated_controls[j][i]);
	  usleep(microseconds);
	  robots[j]->setVelocities(cero_vel_mobile);
	  // TODO remove next two lines
	  robots[j]->getCurrentConfiguration(xt);
	  path.push_back(xt);
      }
    }
  }
  
  
  clock_t end = clock(); 
  double elapsed_time = float(end - begin) / CLOCKS_PER_SEC;
  
 // TODO: execute in parallel for each robot
  std::cout << "Trajectory completed" << std::endl;
  vpFileReader reader;
  reader.saveVectorOfVectors<double>(path, dataFolder + "/executed_path_eu");
  
  return elapsed_time;
}


void RobotComposedEVA::updateRobotLocalization(mrpt::poses::CPose3D transformation)
{
  robots[0]->updateRobotLocalization(transformation);
}



bool RobotComposedEVAError::init()
{
  RobotComposed::init();
  
  vpFileReader reader;
  std::string sigmas_file(configFolder);
  sigmas_file.append("/Sigmas");
  
  if(!reader.readMSLVector<double>(Sigmas, sigmas_file))
    exit(0);
  
  std::string means_file(configFolder);
  means_file.append("/Means");
  reader.readMSLVector<double>(Means, means_file);
  
  // TODO read from configuration
  m = new Model3DRigidEVA(configFolder);
  //m = new Model3DRigidEVAmetricJ(configFolder);
  g = new GeomPQP3DRigidMulti(configFolder);
  
//   std::string InitialStateFile(configFolder);
//   InitialStateFile.append("/InitialState");
//   if(!reader.readMSLVector<double>(initialConfiguration, InitialStateFile))
//     exit(0);
//   
//   vpl::stdvector2mslvector<double>(initialConfiguration, initialState);
//   currentState = initialState;
//   currentConfiguration = initialConfiguration;
  
}



float RobotComposedEVAError::executeMovement()
{
  // ------------- excecute controls ----------------------
  // Integrate with the controls and introduced errors
  std::cout << "Excecuting controls with simulated model and NOISE" << std::endl;
  
  std::vector< std::vector< std::vector<double> > > separated_controls(numberOfRobots);
  std::vector< std::vector< std::vector<double> > > separated_paths(numberOfRobots);
  
  std::vector<double> q_0;
  MSLVector x_t(stateDim);
  MSLVector x_tmas1;
  
  std::vector<double> currentConfiguration;
  getCurrentConfiguration(currentConfiguration);
  nbvs::stdvector2mslvector<double>(currentConfiguration, x_t);
  
  std::cout << "Current state" << x_t << std::endl;
  
  double velocity;
  double epsilon;
  
  std::vector< std::vector<double> >::iterator it_control;
  std::vector< std::vector<double> >::iterator it_config;
  
  std::vector<double> point;
  std::vector<double> ctrl;
  std::vector< std::vector<double> > path;
  std::vector< std::vector<double> > executed_controls;
  vpFileReader reader;
  
  randomGenerator.randomize();
  
  nbvs::mslvector2stdvector<double>(x_t,point); // remove
  path.push_back(point);
  
  for(it_control = trajectoryToExecute.begin(); it_control != trajectoryToExecute.end(); it_control++){
    MSLVector u_circunfleja(m->InputDim);  

    // perturbar control
    for (int i = 0; i < m->InputDim; i++){
      // Solo si la velocidad es diferente de cero introduciremos error
      if(!((*it_control)[i] == 0)){
      	// el error es centrado en cero, luego se le resta la media para trasladarlo,
	// si es que hubiera una media diferente de cero
	// de esta forma se calcula la probabilidad de un error centrado en cero
	epsilon = randomGenerator.drawGaussian1D(0, Sigmas[i]);
	u_circunfleja[i] = (*it_control)[i] + epsilon + Means[i];
	//cout << "p_eps:" << p_eps << " " ;
      }
    }
    
    x_tmas1 = this->m->Integrate(x_t, u_circunfleja, deltaT);
    nbvs::mslvector2stdvector<double>(x_tmas1,point); // remove
    path.push_back(point); // remove
    nbvs::mslvector2stdvector<double>(u_circunfleja,ctrl);
    executed_controls.push_back(ctrl);
     
    // Verificar si esta libre de colisión
    if (!g->CollisionFree(m->StateToConfiguration(x_tmas1))){
      std::cout << "Colision during the execution of the trajectory!" << std::endl;
      reader.saveVectorOfVectors<double>(path, dataFolder + "/executed_path_eu");
      getchar();
      return -1;
    }
    
    x_t = x_tmas1;
  }
  
  std::cout << "Trajectory executed sucessfully" << std::endl;
  reader.saveVectorOfVectors<double>(path, dataFolder + "/executed_path_eu");
  reader.saveVectorOfVectors<double>(executed_controls, dataFolder + "/executed_controls_eu");
  
  // WARNING: this should be replaced for the function overrideCurrentConfig
  nbvs::mslvector2stdvector<double>(x_t, currentConfiguration);
  //cout << "Goal config:";  
  //printVector(this->goalConfig); 
  //cout << "Reached config:"; printVector(currentConfiguration);
  
  //WARNING voy a forzar la configuración final a la que llega el brazo
  for(int i = 3; i< currentConfiguration.size(); i++)
      currentConfiguration[i] = this->goalConfig[i];
  
  this->moveToConfiguration(currentConfiguration);
  
  return 0;
}


// void RobotComposedEVAError::getCurrentConfiguration(std::vector< double >& q)
// {
//   q = currentConfiguration;
// }

// bool RobotComposedEVAError::moveToConfiguration(std::vector< double > configuration)
// {
//     RobotComposed::moveToConfiguration(configuration);
//     getCurrentConfiguration(currentConfiguration);
//     vpl::stdvector2mslvector<double>(currentConfiguration, currentState);
// }

