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


#ifndef PMRAYNODE_H
#define PMRAYNODE_H

#include <vector>
#include <list>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

class pmRayNode
{
public:
pmRayNode(double x, double y, double z);
pmRayNode(octomap::point3d r);

/** 
 * Receives a matrix r = [x, y, z, 1]^T
 */ 
pmRayNode(boost::numeric::ublas::matrix<double> r);

  octomap::point3d ray;
  boost::numeric::ublas::matrix<double> ray_matrix;
  int nLeafs;
  int deepth;
  int children_count;
  std::list< std::list<pmRayNode>::pointer > childrenPtrList;
  std::list<pmRayNode>::pointer parent;
  
  double angleTo(pmRayNode B);
  double dot(pmRayNode B);
};

typedef std::list< std::list<pmRayNode>::pointer > rtChildrenPtrListType;
typedef std::list<pmRayNode> rtTreeType;
typedef std::list<pmRayNode>::pointer rtPointerToElementType ; 

#endif // PMRAYNODE_H
