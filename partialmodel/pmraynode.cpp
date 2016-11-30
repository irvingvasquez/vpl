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


#include "pmraynode.h"

pmRayNode::pmRayNode(double x, double y, double z):
ray(x,y,z),
ray_matrix(4,1)
{
  ray_matrix(0,0) = x;
  ray_matrix(1,0) = y;
  ray_matrix(2,0) = z;
  ray_matrix(3,0) = 1;
  
  nLeafs = 0; 
  children_count = 0;
  parent = 0;
  deepth = 0;
}

pmRayNode::pmRayNode(octomap::point3d r):
ray(r),
ray_matrix(4,1)
{
  ray_matrix(0,0) = ray.x();
  ray_matrix(1,0) = ray.y();
  ray_matrix(2,0) = ray.z();
  ray_matrix(3,0) = 1;
  
  nLeafs = 0; 
  children_count = 0;
  parent = 0;
  deepth = 0;
}

pmRayNode::pmRayNode(boost::numeric::ublas::matrix< double > r):
ray_matrix(r),
ray(r(0,0), r(1,0), r(2,0))
{
  nLeafs = 0; 
  children_count = 0;
  parent = 0;
  deepth = 0;
}

double pmRayNode::angleTo(pmRayNode B)
{
  return ray.angleTo(B.ray);

}

double pmRayNode::dot(pmRayNode B)
{
  return ray.dot(B.ray);
}
