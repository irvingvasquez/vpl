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


#include "vprobot.h"


void vpRobot::getInfo(std::string& inf)
{
  inf = info;
}

bool vpRobot::init()
{
  configFile = configFolder + "/viewPlanning.ini";
  
  vpFileReader reader;
  std::string file_x;
  
  file_x = configFolder + "/StateDim";
  if(!reader.readSingleValue<int>(stateDim, file_x))
    exit(0);
  
  configurationDim = stateDim;
  
  file_x = configFolder + "/InputDim";
  if(!reader.readSingleValue<int>(inputDim, file_x))
    exit(0);
  
  file_x = configFolder + "/LowerState";
  if(!reader.readMSLVector<double>(lowerState, file_x))
    exit(0);
  
  file_x = configFolder + "/UpperState";
  if(!reader.readMSLVector<double>(upperState, file_x))
    exit(0);
  
  
}


vpRobot::vpRobot()
{
  configurationDim = 0;
  
  srand(time(0));
}

void vpRobot::setConfigFolder(std::string folder)
{
  configFolder = folder;
}

void vpRobot::setDataFolder(std::string folder)
{
  dataFolder = folder;
}

bool vpRobot::executePath()
{
  std::vector< std::vector<double> >::iterator it_config;
  for(it_config = pathToExecute.begin(); it_config!= pathToExecute.end(); it_config++){
    this->moveToConfiguration(*it_config);
  }
  
  return true;
}


void vpRobot::setLowerState(std::vector< double > state)
{
  //TODO validate
  lowerState = state;
}

void vpRobot::setUpperState(std::vector< double > state)
{
  //TODO validate
  upperState = state;
}


void vpRobot::setStateDim(int dim)
{
  //TODO validate
  stateDim = dim;
}


double vpRobot::unifRand()
{
   return (double) ((double)rand() / double(RAND_MAX));
}

double vpRobot::unifRand(double a, double b)
{
  return a + (b-a)*unifRand();
  
}


void vpRobot::getRandomState(std::vector< double >& state)
{
  state.clear();
  state.resize(stateDim);

  for(int i = 0; i<stateDim ; i++){
    state[i] = unifRand(lowerState[i], upperState[i]);
  }
}


void vpRobot::getCurrentHTM(BoostMatrix& HTM)
{
  std::vector<double> q;
  this->getCurrentConfiguration(q);
  this->getHTMfromConfiguration(HTM, q);
}

void vpRobot::getCurrentPose(mrpt::poses::CPose3D& pose)
{
  std::vector<double> q;
  this->getCurrentConfiguration(q);
  this->getPoseFromConfiguration(pose,q);
}



void vpRobot::getPoseFromConfiguration(std::vector< double >& pose, std::vector< double > q)
{
  mrpt::poses::CPose3D P;
  this->getPoseFromConfiguration(P,q);
  pose.clear();
  pose[0] = P.m_coords[0];
  pose[1] = P.m_coords[1];
  pose[2] = P.m_coords[2];
  pose[3] = P.yaw();
  pose[4] = P.pitch();
  pose[5] = P.roll();
}


int vpRobot::getConfigDimension()
{
  return configurationDim;
}


int vpRobot::getInputDim()
{
  return inputDim;
}
