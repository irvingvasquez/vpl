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

#ifndef VIEWSYNTHESIS_H
#define VIEWSYNTHESIS_H

#include "viewstructure.h"
#include <time.h>
//#include <stdlib.h>
#include <vector>
//#include <stdio.h>
#include <cstring>

#include <mrpt/poses.h>

/*
 * Synthesis methods compute the NBV or a set of promising views by geometrical analysis of the reconstructed object.
 */
class ViewSynthesis
{
public:
ViewSynthesis();
virtual void getViews(ViewList &views)=0;

protected:
  // maximum number of generated views, -1 no limit
  int n_max;
  
};



/*
 * Synthesis methods compute the NBV or a set of promising views by geometrical analysis of the reconstructed object.
 * Random views
 */
class RandomViewSynthesis :public ViewSynthesis
{
public:
  RandomViewSynthesis(int n, ViewStructure min, ViewStructure max);
  
virtual void getViews(ViewList& views);

protected:
  // cube encapsulator
  ViewStructure min_view;
  ViewStructure max_view;
};


/*
 * View sphere generation by icosahedron tesselation
 * Parameters: radious, sphere center, tesselation level
 * (double r, double x, double y, double z, int level=1);
 */
class ViewSphereSynthesis: public ViewSynthesis
{
public:
  /*
   * Parameters: radious, sphere center, tesselation level
   * (double r, double x, double y, double z, int level=1);
   */
ViewSphereSynthesis(double r, double x, double y, double z, int level=1);
  
virtual void getViews(ViewList& views);

protected:
  double radius;
  double cx, cy,cz;
  // level of subdivisions	
  int tesselation_level;
  
  int n_vertices;
  int n_faces;
  int n_edges;
  float *vertices;
  int *faces; 
  
  int edge_run; 
  int *start; 
  int *end; 
  int *midpoint; 
  
  /**
   * Returns a set of points that represent a sphere of radius 1
   */
  int getSpherePositionsByTesselation(int level, std::vector< std::vector<double> > &points);
  
  void initIcosahedro();
  
  void subdivide();
  
  int midpointSearch(int index_start, int index_end);
  
  void copySphere(std::vector< std::vector<double> > &points);
  
  float **mFMatrix(int nrl,int nrh,int ncl,int nch);
  
  void mFreeFMatrix(float **m,int nrl,int nrh,int ncl,int nch);

  /**
   * only works in workspace. And for freeflyer robot.
   */
  void getPointedViews(std::vector< std::vector<double> > points, ViewList &views);
};


#endif // VIEWSYNTHESIS_H
