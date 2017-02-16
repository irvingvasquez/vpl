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

#ifndef PMVOCTREEIGKRIEGEL12_H
#define PMVOCTREEIGKRIEGEL12_H

#include "pmvoctree.h"

#include "coctreevpl.h"


/**
 * 
 * Implementation status: ok
 * Testing status: ok
 */
class COctreeKriegel12 : public COctreeVPL
{
protected:
  
public:
COctreeKriegel12(double resolution);
  
  /**
   * Traces a Ray and returns the information gain of a ray. Integrates even for already touched voxels. Kriegel 12
   */
virtual double castRayIG(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknownCells = false, double maxRange = -1.0);
};



/**
 * Partial Model based on a probabilistic octree
 * Reported in: 
 * Kriegel 12
 * 
 * Initialization:
 * 	- Set config folder
 * 	- Set data folder 
 * 	- init()
 * 	- readRays();
 * 	- set utility function
 * 
 * Implementation status: ok
 * Testing status: n/a
 */
class PMVOctreeIGKriegel12 : public PMVOctree
{
protected:
   double rayTracingHTMIGKriegel(boost::numeric::ublas::matrix<double> m, EvaluationResult &result);
  
public:
PMVOctreeIGKriegel12();

virtual bool init();

virtual int evaluateView(ViewStructure& v);

virtual float updateWithScan(std::string file_name_scan, std::string file_name_origin);
};



#endif // PMVOCTREEIGKRIEGEL12_H
