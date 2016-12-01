
#include "vplutils.h"

// void vpl::getPoseFromHTM(std::vector< double > pose, CMatrixDouble44 HTM)
// {
//     pose.clear();
//     pose.resize(6);
//     
//     pose[0] = HTM(0,3);
//     pose[1] = HTM(1,3);
//     pose[2] = HTM(2,3);
// 
//     //pose[4] = atan2(  -HTM(2,0), sqrt(sqr(HTM(0,0))+sqr(HTM(1,0)))  );
//     
//     pose[4] = atan2(  -HTM(2,0), sqrt(pow(HTM(0,0),2)+pow(HTM(1,0),2))  );
//     
//     if ( (pose[4]>0.0) && (cos(pose[4])==0.0) ) {
//       pose[3] = 0.0;
//       pose[5] = atan2(HTM(0,1), HTM(1,1));
//     }  
//     else {
//       if ( (pose[4]<0.0) && (cos(pose[4])==0.0) ) {
// 	pose[3] = 0.0;
// 	pose[5] = -atan2(HTM(0,1), HTM(1,1));
//       }
//       else {
// 	pose[3] = atan2(  HTM(1,0)/cos(pose[4]), HTM(0,0)/cos(pose[4])  );
// 	pose[5] = atan2(  HTM(2,1)/cos(pose[4]), HTM(2,2)/cos(pose[4])  );
//       }
//     }
//     
//     
//     if (pose[3] < -M_PI)
//       pose[3] += 2.0*M_PI; // Force the orientation into [-pi,pi)
//     else if (pose[3] >= M_PI)
//       pose[3] -= 2.0*M_PI;
//     if (pose[4] < -M_PI)
//       pose[4] += 2.0*M_PI; // Force the orientation into [-pi,pi)
//     else if (pose[4] >= M_PI)
//       pose[4] -= 2.0*M_PI;
//     if (pose[5] < -M_PI)
//       pose[5] += 2.0*M_PI; // Force the orientation into [-pi,pi)
//     else if (pose[5] >= M_PI)
//       pose[5] -= 2.0*M_PI;
// }
// 

void vpl::convertBoost2Eigen(const boost::numeric::ublas::matrix< double >& BoostM, CMatrixDouble44& EigenMtx)
{
  for(int i =0; i<4; i++){
    for(int j=0; j<4; j++){
      EigenMtx(i,j) = BoostM(i,j);
    }
  }
  
  //cout << BoostM << std::endl;
//   std::cout << EigenMtx << std::endl;
//   getchar();
}


void vpl::Pose2Vector(const mrpt::poses::CPose3D& pose, std::vector< double >& vec)
{
  vec.clear();
  vec.resize(6);
  
  vec[0] = pose.m_coords[0];
  vec[1] = pose.m_coords[1];
  vec[2] = pose.m_coords[2];
  
  vec[3] = pose.yaw();
  vec[4] = pose.pitch();
  vec[5] = pose.roll();
  
//   std::cout << pose;
//   getchar();
}


bool vpl::userContinues()
{
  bool ok = false;
  char a;
  bool return_value;
  
  while(!ok){
    std::cout << "Continue? [y/n]:";
    cin >> a;
    if (a == 'n') {
      return_value = false;
      ok = true;
    }
    else {
      if (a == 'y'){
	return_value = true;
	ok = true;
      }
    }
  }
  return return_value;
}

bool vpl::readTransformationMatrix(std::string filename, mrpt::poses::CPose3D& pose)
{
  // read transformation
  Eigen::Matrix4f T;
  
  std::ifstream myfile (filename.c_str());
  if (myfile.is_open())
  {
    myfile >> T(0,0);
    myfile >> T(0,1);
    myfile >> T(0,2);
    myfile >> T(0,3);
    
    myfile >> T(1,0);
    myfile >> T(1,1);
    myfile >> T(1,2)  ;
    myfile >> T(1,3)  ;
    
    myfile >> T(2,0)  ;
    myfile >> T(2,1)  ;
    myfile >> T(2,2)  ;
    myfile >> T(2,3)  ;
    
    myfile >> T(3,0)  ;
    myfile >> T(3,1)  ;
    myfile >> T(3,2)  ;
    myfile >> T(3,3)  ;
    
    myfile.close();
    //cout << T << std::endl;
  }
  else { 
    std::cout << "Unable to open transformation file"; 
    return false;
  }
  
  
  std::vector<double> m(12);
  m[0] = T(0,0);
  m[1] = T(1,0);
  m[2] = T(2,0);
  
  m[3] = T(0,1);
  m[4] = T(1,1);
  m[5] = T(2,1);
  
  m[6] = T(0,2);
  m[7] = T(1,2);
  m[8] = T(2,2);
  
  m[9] = T(0,3);
  m[10] = T(1,3);
  m[11] = T(2,3);
  
  pose.setFrom12Vector< std::vector<double> >(m);
  
  //cout << "Readed transformation htm : " << std::endl;
  //cout << pose << std::endl;
  
  return true;
}

