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


#include "robotarmmdh.h"



bool RobotArmMDH::init()
{
   vpRobot::init();
   
   vpFileReader reader;
   
   std::string file_x = configFolder + "/DH";
   if(!reader.readMSLVector<double>(DH, file_x))
      exit(0);
   
   file_x = configFolder + "/StateIndices";
   if(!reader.readInt(file_x, stateIndices))
      exit(0);
   
   
   file_x = configFolder + "/ArmStateDim";
   if(!reader.readSingleValue<int>(armStateDim, file_x))
      exit(0);
   
   configurationDim = armStateDim;
   inputDim = armStateDim;
   
   info.assign("Name: Arm Robot");
}

RobotArmMDH::RobotArmMDH()
{
  
}


void RobotArmMDH::getPoseFromConfiguration(mrpt::poses::CPose3D& pose, std::vector< double, std::allocator< double > > q)
{
  int i;
  CMatrixDouble44  r, rn, ro, r_arm;
  
//   std::cout << "q:" << q << std::endl;
//   std::cout << armStateDim << std::endl;
//   std::cout << stateIndices << std::endl;

  std::vector<double> A(armStateDim); 
  std::vector<double> D(armStateDim); 
  std::vector<double> Alpha(armStateDim);
  std::vector<double> Theta(armStateDim);
  
  // Get Denavit Hartemberg Parameters
  for (i = 0; i < armStateDim; i++ ) {
    if (stateIndices[i] != 0) {
      int y = stateIndices[i];
      DH[y-1] = q[i];
    }
  }
  
//   std::cout << "DH: " << DH << std::endl; 
  
  for (i = 0; i < armStateDim; i++) {
       Alpha[i] = DH[i];
       Theta[i] = DH[armStateDim*1+i];
       A[i] = DH[armStateDim*2+i];
       D[i] = DH[armStateDim*3+i];
  }
  
//   std::cout << Alpha << std::endl;
//   std::cout << Theta << std::endl;
//   std::cout << A << std::endl;
//   std::cout << D << std::endl;

  // initialize a identity matrix
  for (i = 0; i < 4 ; i ++ ) {
    for (int j = 0 ; j < 4 ; j ++ ) {
      if (i==j) {r(i,j) = 1.0;}
      else {r(i,j) = 0.0;}
    }
  }
  
//   std::cout << "identity" << std::endl << r << std::endl;
//   std::cout << "numBodies " << armStateDim << std::endl;
//   std::cout << "Theta" << Theta << std::endl;
  // --------------  Denavit Hartemberg -----------------------------
  for (i = 0; i < armStateDim; i++) {
    ro=r;
    
    rn(0,0) = cos(Theta[i]);
    rn(0,1) = -sin(Theta[i]);
    rn(0,2) = 0.0;
    rn(0,3) = A[i];
    rn(1,2) = -sin(Alpha[i]);
    rn(2,2) = cos(Alpha[i]);
    rn(3,2) = 0.0;
    rn(1,3) = rn(1,2)*D[i];
    rn(2,3) = rn(2,2)*D[i];
    rn(3,3) = 1.0;
    rn(1,0) = -rn(0,1)*rn(2,2);
    rn(2,0) = -rn(0,1)*(-rn(1,2));
    rn(3,0) = 0.0;
    rn(1,1) = rn(0,0)*rn(2,2);
    rn(2,1) = rn(0,0)*(-rn(1,2));
    rn(3,1) = 0.0;
  
//     std::cout << "rn" << std::endl << rn << std::endl;
    
    r = ro * rn;  
    
//     std::cout << "r" << r << std::endl;
  }
  
//   std::cout << r << std::endl;
  
  mrpt::poses::CPose3D tp(r);
  pose = tp;
  
  //cout << r << std::endl;
  // std::cout << tp << std::endl;
}



void RobotArmMDH::getHTMfromConfiguration(BoostMatrix& HTM, std::vector< double, std::allocator< double > > q)
{
  int i;
  CMatrixDouble44  r, rn, ro, r_arm;

  std::vector<double> A(armStateDim); 
  std::vector<double> D(armStateDim); 
  std::vector<double> Alpha(armStateDim);
  std::vector<double> Theta(armStateDim);
  
  // Get Denavit Hartemberg Parameters
  for (i = 0; i < armStateDim; i++ ) {
    if (stateIndices[i] != 0) {
      int y = stateIndices[i];
      DH[y-1] = q[i];
    }
  }
  
  //cout << "DH: " << DH << std::endl; 
  for (i = 0; i < armStateDim; i++) {
       Alpha[i] = DH[i];
       Theta[i] = DH[armStateDim*1+i];
       A[i] = DH[armStateDim*2+i];
       D[i] = DH[armStateDim*3+i];
  }
  

  // initialize a identity matrix
  for (i = 0; i < 4 ; i ++ ) {
    for (int j = 0 ; j < 4 ; j ++ ) {
      if (i==j) {r(i,j) = 1.0;}
      else {r(i,j) = 0.0;}
    }
  }
 
   
  // --------------  Denavit Hartemberg -----------------------------
  for (i = 0; i < armStateDim; i++) {
    ro=r;
    
    rn(0,0) = cos(Theta[i]);
    rn(0,1) = -sin(Theta[i]);
    rn(0,2) = 0.0;
    rn(0,3) = A[i];
    rn(1,2) = -sin(Alpha[i]);
    rn(2,2) = cos(Alpha[i]);
    rn(3,2) = 0.0;
    rn(1,3) = rn(1,2)*D[i];
    rn(2,3) = rn(2,2)*D[i];
    rn(3,3) = 1.0;
    rn(1,0) = -rn(0,1)*rn(2,2);
    rn(2,0) = -rn(0,1)*(-rn(1,2));
    rn(3,0) = 0.0;
    rn(1,1) = rn(0,0)*rn(2,2);
    rn(2,1) = rn(0,0)*(-rn(1,2));
    rn(3,1) = 0.0;
  
    r = ro * rn;  
  }
  
  for(int j=0; j<4; j++){
    for(int k = 0; k<4; k++){
      HTM(j,k) = r(j,k);
    }
  }
  
  
}







float RobotArmMDHDummie::executeTrajectory(std::vector< std::vector< double > > controls, double delta_t, std::vector< double > goal_q)
{
  currentConfig = goal_q;
  return true;
}

float RobotArmMDHDummie::executeMovement()
{
  return true;
}


void RobotArmMDHDummie::getCurrentConfiguration(std::vector< double >& q)
{
  q = currentConfig;
}

bool RobotArmMDHDummie::moveToConfiguration(std::vector< double > configuration)
{
  currentConfig = configuration;
  return true;
}

RobotArmMDHDummie::RobotArmMDHDummie()
{
  
}

bool RobotArmMDHDummie::setVelocities(std::vector< double > velocities)
{
  return true;
}

void RobotArmMDHDummie::updateRobotLocalization(mrpt::poses::CPose3D transformation)
{

}
