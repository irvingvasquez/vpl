/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef PMVOREARSIDEVOXEL_H
#define PMVOREARSIDEVOXEL_H

#include "pmvoctree.h"

#include "coctreevpl.h"

class COctreeRearSideVoxel : public COctreeVPL
{
public:
COctreeRearSideVoxel(double resolution);

octomap::ColorOcTreeNode::Color colorRearSide;

protected:

    /**
   * Traces a Ray in the octree and returns the type of voxel found
   * It has been modified to return rearside voxels
   * It does not return unknown voxels
   */
virtual int castRayVPL(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknownCells = false, double maxRange = -1.0);
};


class PMVORearSideVoxel : public PMVOctree
{
public:
PMVORearSideVoxel();

virtual int evaluateView(ViewStructure& v);

virtual bool init();

virtual float updateWithScan(std::__cxx11::string file_name_scan, std::__cxx11::string file_name_origin);

protected:
  /**
   * Gathers rearside voxels
   * RearSide voxels are returned in result.unknown voxels
   */
  virtual bool rayTracingHTM(boost::numeric::ublas::matrix< double > m, EvaluationResult& result);
};

#endif // PMVOREARSIDEVOXEL_H
