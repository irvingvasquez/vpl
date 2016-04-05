#include <vector>
#include <math.h>
#include <octomap/Pointcloud.h>
#include <iostream>
#include "viewstructure.h"
#include "vpfilereader.h"
#include "vptriangle.h"

using namespace std;



namespace vpUtils {

  double euclideanDistance (std::vector< int > A, std::vector< int > B);
    
  double euclideanDistance (std::vector<double> A, std::vector<double> B);

  void printVector( vector<int> v);
  
  void printVector( vector<float> v);
  
  void printVector( vector<double> v);

  bool utilsAreBoolVectorsEqual(vector<bool> A, vector<bool> B);

  void utilsGetActivatedMotors(vector<int> u, vector<bool> &activations);

  int applyIncrement(const vector<int> &q_t, const vector<int> &control, vector<int> &q_tmas1);

  bool raw2triangles(const string file_raw, const string file_tri);
  
  bool saveCoordinatesAsVRML( vector<vector<double> > data, string file_name);
  
  /**
   * Reads the files and return the percentage of matched points based on the reference file
   */
  double compareFilePoints(string file_target, string file_reference, double gap);
}