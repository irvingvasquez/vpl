
#ifndef VPLUTILS_H
#define VPLUTILS_H

#include <vector>
#include <mrpt/base/include/mrpt/poses.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <viewstructure.h>

using namespace mrpt::math;

namespace vpl{
  
  void convertBoost2Eigen(const boost::numeric::ublas::matrix<double> &BoostM, CMatrixDouble44 &EigenMtx );
  
  void Pose2Vector(const mrpt::poses::CPose3D &pose, std::vector<double> &vec);
  
  bool userContinues();
  
  bool readTransformationMatrix(std::string filename, mrpt::poses::CPose3D &pose);
  

}


#endif