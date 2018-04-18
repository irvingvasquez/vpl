/*
 * 
 * 
Partial Model Library
Copyright (c) 2016, J. Irving Vasquez ivasquez@ccc.inaoep.mx
Consejo Nacional de Ciencia y Tecnología (CONACYT)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <vector>
//#include <math.h>
#include <octomap/Pointcloud.h>
#include <iostream>
//#include "vpfilereader.h"
#include "vptriangle.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp> 

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
  
  /*
  * gets the homogeneous transformation matrix from a pose (x,y,z, yaw(z), pitch(y), roll(x))
  * status: not tested
  */
  void getHTMfromPose(std::vector<double> pose, boost::numeric::ublas::matrix<double> &HTM); 
  
  /*
  * gets the homogeneous transformation matrix from a pose (x,y,z, yaw(z), pitch(y), roll(x))
  * status: not tested
  */
  void getHTMfromPose(octomath::Pose6D pose, boost::numeric::ublas::matrix<double> &HTM); 
  
}