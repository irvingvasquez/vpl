/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

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


#ifndef PMVOCTREEIG_H
#define PMVOCTREEIG_H

#include "pmvoctree.h"

/**
 * Partial Model based on a probabilistic octree
 * Frustum Information Gain
 * Reported in: 
 * IROS17 submitted
 * 
 * Initialization:
 * 	- Set config folder
 * 	- Set data folder 
 * 	- init()
 * 	- readRays();
 * 	- set utility function
 * 
*/

class PMVOctreeIG : public PMVOctree
{
protected:
   double rayTracingHTMIG(boost::numeric::ublas::matrix<double> m, EvaluationResult &result);
  
public:
PMVOctreeIG();

virtual int evaluateView(ViewStructure &v);

virtual float updateWithScan(std::string file_name_scan, std::string file_name_origin);

};

#endif // PMVOCTREEIG_H
