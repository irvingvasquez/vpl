#include "vputils.h"


bool vpUtils::raw2triangles(const string file_raw, const string file_tri)
{
  vpTriangleList tl;
  
  tl.readFile(file_raw);
  tl.saveToMSLTriangle(file_tri);
  
  return true;
}


bool vpUtils::saveCoordinatesAsVRML(vector< std::vector< double > > data, string file_name)
{
  vector<vector<double> >::iterator it;
  octomap::Pointcloud pc;
  
  for(it = data.begin(); it!= data.end(); it++){

    octomap::point3d p((*it)[0],(*it)[1],(*it)[2]);    
    pc.push_back(p);
  }
  
  pc.writeVrml(file_name);
  return true;
}


double vpUtils::euclideanDistance(std::vector< int > A, std::vector< int > B)
{
   vector<int>::iterator ita;
   vector<int>::iterator itb;
   
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


double vpUtils::euclideanDistance(std::vector< double > A, std::vector< double > B)
{
   vector<double>::iterator ita;
   vector<double>::iterator itb;
   
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


void vpUtils::printVector(std::vector< int > v)
{
  vector<int>::iterator it;
  for(it = v.begin(); it!= v.end(); it++){
    std::cout << " " << *it;
  }
  cout << endl;
}

void vpUtils::printVector(std::vector< float > v)
{
  vector<float>::iterator it;
  for(it = v.begin(); it!= v.end(); it++){
    std::cout << " " << *it;
  }
  cout << endl;
}

void vpUtils::printVector(std::vector< double > v)
{
  vector<double>::iterator it;
  for(it = v.begin(); it!= v.end(); it++){
    std::cout << " " << *it;
  }
  cout << endl;
}



bool vpUtils::utilsAreBoolVectorsEqual(std::vector< bool > A, std::vector< bool > B)
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


void vpUtils::utilsGetActivatedMotors(std::vector< int > u, std::vector< bool > &activations)
{
  activations.clear();
//  activations.resize(u.size());
  vector<int>::iterator it;
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
  
  //cout << activations.size() << endl;
}


int vpUtils::applyIncrement(const std::vector< int >& q_t, const std::vector< int >& control, std::vector< int >& q_tmas1)
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


double vpUtils::compareFilePoints(string file_target, string file_reference, double gap)
{
   //string file_a("/home/irving/projects/nbvPlanning-1.1/test/EVA/data/object_accumulated_points.dat");
   string file_a(file_target);
   
   //string file_reference("/home/irving/Blensor/scenes/teapot_ref.dat");
//   string file_reference(argv[2]);
//   string output_file(argv[3]);
   
   cout << "file to read: " << file_a << endl; 
   
   vpFileReader reader;
   
   vector< vector<double> > a,reference;
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
   
   cout << "Correspondences: " << correspondences << endl;
   double percentage = (correspondences / (double) reference.size()) * 100 ;
   cout << "%" << percentage << endl;
   
   return percentage;
}

