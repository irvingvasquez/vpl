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


#ifndef PMVOCTREE_H
#define PMVOCTREE_H

#include "pmvolumetric.h"
#include "volumetricutilityfunction.h"
#include "coctreevpl.h"


/**
 * Octree class for view planning in 3D object Reconstruction.
 * 
 * Initialization:
 * 	- Set config folder
 * 	- Set data folder 
 * 	- init()
 * 	- readRays();
 * 	- set utility function
*/

class PMVOctree :  public PMVolumetric
{

public:
PMVOctree();

  virtual float updateWithScan(std::string file_name_scan, std::string file_name_origin);
  
  virtual bool savePartialModel(std::string file_name);
  
  virtual bool loadPartialModel(std::string file_name);
  
  virtual void evaluateCandidateViews();
  
  virtual int evaluateView(ViewStructure &v);
  
  /**
  * Reads the rays (directions) thought by the sensor
  * They are stored at rays with the notation [x, y, z, 1]^T
  * The 1 is added to perform matrix multiplications by a HTM
  */ 
  virtual long int readRays(std::string file_address);
  

   
  //virtual void paintOccupiedInCapsule();
  
  /**
   * Paints in blue occupied voxels that are inside the object capsule.
   * Paints in orange unknown voxels
   */
  long int paintVoxels(COctreeVPL *octree);
  
  virtual bool init();
  
  /**
   * Inserts a cube of free space given by two xtreme points
   */
  void insertFreeSpace(double x1, double y1, double z1, double x2, double y2, double z2);  
  
//   void saveEvaluations();
  
  void setUtilityFunction(VolumetricUtilityFunction *uf);
  
  virtual void saveObjectAsRawT(std::string file_name);

  virtual void saveObjectAsObst(std::string file_name);
  
  virtual bool saveUnknownVolumeAsObst(std::string file_name);
  
  virtual bool saveUnknownVolumeAsRawT(std::string file_name);
  
  /**
   * Saves the occupied  and unknown voxels as Obs
   */
  virtual bool saveObstacle(std::string file_name);
  
  virtual void getOccupiedTriangles(vpTriangleList &tris);
  
  virtual void getUnknownTriangles(vpTriangleList &tris);
  
  virtual double getUnknownVolume();

protected:
  
  /// Octree representation
  //octomap::ColorOcTree *map;
  COctreeVPL *map;
  
  
  /**
   * Performs a ray tracing to gather how much voxels ware touched. It uses the translation of the HTM like origin of the ray tracing.
   * Returns false if the origin is not free.
   * @param m [in] Homogenous transformation matrix. It will be applied to the sensor model
   * @param n_occupied [out]
   * @param n_unknown [out]
   */
  bool rayTracingHTM(boost::numeric::ublas::matrix<double> m, EvaluationResult &result);
  
};

#endif // PMVOCTREE_H
