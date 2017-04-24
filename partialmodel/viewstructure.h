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


#ifndef VIEWSTRUCTURE_H
#define VIEWSTRUCTURE_H

#include <list>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

//#include "pmutils.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp> 


enum viewTypes { SURFACE_TYPE, EXPLORATION_TYPE};

/**
 * Results from the evaluation of a view
 */
enum _VIEW_FEASIBILITY {
  FEASIBLE_VIEW,
  UNFEASIBLE_VIEW
};


/**
 * 
 * Robot state: q
 * Sensor pose:  w[x, y, z, yaw(z), pitch(y), roll(x)] radians
 * homogeneous transformtation matrix of the eye in hand sensor: HTM 
 */
class ViewStructure
{

public:
ViewStructure();


/**
 * Sensor pose
 * w [x, y, z, yaw(z), pitch(y), roll(x)]
 * accordinly to a 3D pose of MRPT
 */
std::vector<double> w;


/**
 * Robot state or configuration
 * It is only a storage variable for convinience but the member functions do not use it
 */
std::vector<double> q;


///Homogeneous transformtation matrix
boost::numeric::ublas::matrix<double> HTM;

int type;

/*
 * Evaluation of a view
 * In some papers it is called fitness
 */
double eval;

long int n_occupied;
long int n_unknown;
long int n_occplane;
double d;

void setPose(double x, double y, double z, double yaw, double pitch, double roll);

void setPose(std::vector<double> pose);

friend std::ostream& operator << (std::ostream& out, ViewStructure& view);
  
bool operator == (const ViewStructure &) const;

protected:
  int pose_lenght;
};


class ViewList : public std::list<ViewStructure>
{ 
protected:


public:
  bool save(std::string file_name);
  
  bool saveAsMSLStates(std::string file_name);
  
  bool read(std::string file_name);
  
  ViewStructure getBestView();
  
  /*
   * Sort views depending on their evaluation value
   */
  void sortHighToLow();
};


/**
 * Convert a HTM by a scale
 * To conver from mm to mts scale must be 1000
 */
//boost::numeric::ublas::matrix<double> vsScaleHTM(boost::numeric::ublas::matrix<double> HTM, float scale);


/**
 * Save a list of views.
 * @param factor is used for changing scale. 
 * For example, if the original HTM was computed with mm and it will be used with mts, factor should be 1000
 * Inner matix is computed with S' x HTM x S
 */
//bool vsSaveViewList(std::list<ViewStructure> views, std::string fileName, double factor);


/**
 * Save a list of views
 */
bool vsSaveViewList(std::list<ViewStructure> views, std::string fileName);

bool vsReadViewList(std::list<ViewStructure> &views, std::string fileName);

ViewStructure bestViewOfList(std::list<ViewStructure> &viewsList);

void orderViewsHighToLow(std::list<ViewStructure> &viewsList);

bool compareByEval(ViewStructure first, ViewStructure second);

bool compareByEvalInverse(ViewStructure first, ViewStructure second);

void getNViewsFromList(std::list<ViewStructure> &viewsList, int n, std::list<ViewStructure> &result);


#endif // VIEWSTRUCTURE_H
