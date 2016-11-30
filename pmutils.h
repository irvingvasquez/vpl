#include <vector>
#include <math.h>
#include <octomap/Pointcloud.h>
#include <iostream>
//#include "vpfilereader.h"
#include "vptriangle.h"

//TODO change the name of this file and namespace

namespace PMUtils {

  double euclideanDistance (std::vector< int > A, std::vector< int > B);
    
  double euclideanDistance (std::vector<double> A, std::vector<double> B);

  void printVector( std::vector<int> v);
  
  void printVector( std::vector<float> v);
  
  void printVector( std::vector<double> v);

  bool utilsAreBoolVectorsEqual(std::vector<bool> A, std::vector<bool> B);

  void utilsGetActivatedMotors(std::vector<int> u, std::vector<bool> &activations);

  int applyIncrement(const std::vector<int> &q_t, const std::vector<int> &control, std::vector<int> &q_tmas1);

  bool raw2triangles(const std::string file_raw, const std::string file_tri);
  
  bool saveCoordinatesAsVRML( std::vector< std::vector<double> > data, std::string file_name);
  
  /**
   * Reads the files and return the percentage of matched points based on the reference file
   */
  double compareFilePoints(std::string file_target, std::string file_reference, double gap);
}