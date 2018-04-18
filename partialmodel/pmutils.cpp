#include "pmutils.h"


bool PMUtils::raw2triangles(const std::string file_raw, const std::string file_tri)
{
  vpTriangleList tl;
  
  tl.readFile(file_raw);
  tl.saveToMSLTriangle(file_tri);
  
  return true;
}


bool PMUtils::saveCoordinatesAsVRML(std::vector< std::vector<double> > data, std::string file_name)
{
  std::vector< std::vector<double> >::iterator it;
  octomap::Pointcloud pc;
  
  for(it = data.begin(); it!= data.end(); it++){

    octomap::point3d p((*it)[0],(*it)[1],(*it)[2]);    
    pc.push_back(p);
  }
  
  pc.writeVrml(file_name);
  return true;
}


double PMUtils::euclideanDistance(std::vector< int > A, std::vector< int > B)
{
   std::vector<int>::iterator ita;
   std::vector<int>::iterator itb;
   
   double sum = 0.0;
   double distance =  0.0;
   double resta = 0.0;
  // int a=0;
  // int b=0;
   
   itb = B.begin();
   for(ita = A.begin(); ita != A.end(); ita++ ){
     // a = *ita;
      resta = (double) (*ita) - (double) (*itb);
      sum = sum + pow(resta, 2);
      itb ++;
    }
    
    distance = sqrt(sum);
    return distance;
}


double PMUtils::euclideanDistance(std::vector< double > A, std::vector< double > B)
{
   std::vector<double>::iterator ita;
   std::vector<double>::iterator itb;
   
   double sum = 0.0;
   double distance =  0.0;
   double resta = 0.0;
  // int a=0;
  // int b=0;
   
   itb = B.begin();
   for(ita = A.begin(); ita != A.end(); ita++ ){
     // a = *ita;
      resta = (double) (*ita) - (double) (*itb);
      sum = sum + pow(resta, 2);
      itb ++;
    }
    
    distance = sqrt(sum);
    return distance;
}


void PMUtils::printVector(std::vector< int > v)
{
  std::vector<int>::iterator it;
  for(it = v.begin(); it!= v.end(); it++){
    std::cout << " " << *it;
  }
  std::cout <<std::endl;
}

void PMUtils::printVector(std::vector< float > v)
{
  std::vector<float>::iterator it;
  for(it = v.begin(); it!= v.end(); it++){
    std::cout << " " << *it;
  }
  std::cout << std::endl;
}

void PMUtils::printVector(std::vector< double > v)
{
  std::vector<double>::iterator it;
  for(it = v.begin(); it!= v.end(); it++){
    std::cout << " " << *it;
  }
  std::cout << std::endl;
}



bool PMUtils::utilsAreBoolVectorsEqual(std::vector< bool > A, std::vector< bool > B)
{
  if(A.size() != B.size())
    return false;
  
  std::vector<bool>::iterator it;
  std::vector<bool>::iterator it_B;
  it_B = B.begin();
  for(it=A.begin(); it!=A.end(); it++){
    
    if(*it != *it_B)
      return false;
    
    it_B++;
  }
  
  return true;
}


void PMUtils::utilsGetActivatedMotors(std::vector< int > u, std::vector< bool >& activations)
{
  activations.clear();
//  activations.resize(u.size());
  std::vector<int>::iterator it;
  bool value;
  
  for(it=u.begin(); it!=u.end(); it++){
    if((*it) == 0){
      value = false;
      activations.push_back(value);
    }
    else{
      value = true;
      activations.push_back(value);
    }
  }
  
  //cout << activations.size() <<std::endl;
}


int PMUtils::applyIncrement(const std::vector< int >& q_t, const std::vector< int >& control, std::vector< int >& q_tmas1)
{
    q_tmas1.clear();
    q_tmas1.resize(q_t.size());
    
    int aux;
    
    // apply control and determine the next configuration
    for(int i = 0; i< q_t.size(); i++){
        aux = q_t[i] + control[i];
	q_tmas1[i] = aux;
    }
    
    return 0;
}


double PMUtils::compareFilePoints(std::string file_target, std::string file_reference, double gap)
{
   //string file_a("/home/irving/projects/nbvPlanning-1.1/test/EVA/data/object_accumulated_points.dat");
   std::string file_a(file_target);
   
   //string file_reference("/home/irving/Blensor/scenes/teapot_ref.dat");
//   std::string file_reference(argv[2]);
//   std::string output_file(argv[3]);
   
   std::cout << "file to read: " << file_a << std::endl; 
   
   vpFileReader reader;
   
   std::vector< std::vector<double> > a,reference;
   reader.readDoubleCoordinates(file_a, a);
   reader.readDoubleCoordinates(file_reference, reference);
   
   //double gap = 0.004;
   long int correspondences=0;
   
   for(int i = 0 ; i < reference.size();i++){
     for(int j= 0; j< a.size(); j++)
     {
       if( fabs(reference[i][0] - a[j][0]) < gap ){
	  if( fabs(reference[i][1] - a[j][1]) < gap ){
	   if( fabs(reference[i][2] - a[j][2]) < gap ){
	     correspondences ++;
	     break;
	   }
	  }
       }
     }
   }
   
   std::cout << "Correspondences: " << correspondences << std::endl;
   double percentage = (correspondences / (double) reference.size()) * 100 ;
   std::cout << "%" << percentage << std::endl;
   
   return percentage;
}


/** 
 * pose = (x,y,z,yaw,pitch,roll)
 * 
 */
void PMUtils::getHTMfromPose(std::vector< double > pose, boost::numeric::ublas::matrix< double > &HTM)
{
  if(pose.size()!= 6)
  {
    std::cout << "ERROR: wrong pose size, getHTMfromPose" << std::endl;
    return;
  }
  
  boost::numeric::ublas::matrix< double > result(4,4);
  double x,y,z, yaw, pitch, roll;
  x = pose[0];
  y = pose[1];
  z = pose[2];
  yaw = pose[3];
  pitch = pose[4];
  roll = pose[5];
  
  double cy = cos(yaw);
  double sy = sin(yaw);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);

  result(0,0) = cy * cp;  
  result(0,1) = cy * sp *sr - sy * cr;
  result(0,2) = cy * sp * cr + sy * sr;
  result(0,3) = x;
  
  result(1,0) = sy * cp;
  result(1,1) = sy * sp * sr + cy * cr;
  result(1,2) = sy * sp * cr - cy * sr;
  result(1,3) = y;
  
  result(2,0) = -sp;
  result(2,1) = cp*sr;
  result(2,2) = cp *cr;
  result(2,3) = z;
  
  result(3,0) = 0;
  result(3,1) = 0;
  result(3,2) = 0;
  result(3,3) = 1;
  
  //std::cout << "result:" << result << "\n" << std::endl;
  HTM = result;
}

void PMUtils::getHTMfromPose(octomath::Pose6D pose, boost::numeric::ublas::matrix< double >& HTM)
{ 
  boost::numeric::ublas::matrix< double > result(4,4);
  double x,y,z, yaw, pitch, roll;
  x = pose.x();
  y = pose.y();
  z = pose.z();
  yaw = pose.yaw();
  pitch = pose.pitch();
  roll = pose.roll();
  
  double cy = cos(yaw);
  double sy = sin(yaw);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);

  result(0,0) = cy * cp;  
  result(0,1) = cy * sp *sr - sy * cr;
  result(0,2) = cy * sp * cr + sy * sr;
  result(0,3) = x;
  
  result(1,0) = sy * cp;
  result(1,1) = sy * sp * sr + cy * cr;
  result(1,2) = sy * sp * cr - cy * sr;
  result(1,3) = y;
  
  result(2,0) = -sp;
  result(2,1) = cp*sr;
  result(2,2) = cp *cr;
  result(2,3) = z;
  
  result(3,0) = 0;
  result(3,1) = 0;
  result(3,2) = 0;
  result(3,3) = 1;
  
  //std::cout << "result:" << result << "\n" << std::endl;
  HTM = result;
}

