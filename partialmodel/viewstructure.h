/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2012  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VIEWSTRUCTURE_H
#define VIEWSTRUCTURE_H

#include <list>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#include "pmutils.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp> 

using namespace PMUtils;

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
 * Robot configuration: q
 * Sensor pose:  w[x, y, z, yaw(z), pitch(y), roll(x)]
 * homogeneous transformtation matrix of the eye in hand sensor: HTM 
 */
class ViewStructure
{

public:
ViewStructure();

/**
 * Sensor pose
 * q [x, y, z, yaw(z), pitch(y), roll(x)]
 */
std::vector<double> q;

/// workspace projection
std::vector<double> w;

///Homogeneous transformtation matrix
boost::numeric::ublas::matrix<double> HTM;

int type;

double eval;
long int n_occupied;
long int n_unknown;
long int n_occplane;
double d;

  friend std::ostream& operator << (std::ostream& out, ViewStructure& view);
  
  bool operator == (const ViewStructure &) const;

};


class ViewList : public std::list<ViewStructure>
{ 
protected:


public:
  bool save(std::string file_name);
  
  bool saveAsMSLStates(std::string file_name);
  
  bool read(std::string file_name);
  
  ViewStructure getBestView();
  
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
