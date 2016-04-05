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


#ifndef PMRAYTREE_H
#define PMRAYTREE_H

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include <iostream>

#include <vector>
#include <list>
#include <string>

#include "pmraynode.h"

using namespace std;

typedef vector< boost::numeric::ublas::matrix<double> > RaysVector;

typedef std::list< std::list<pmRayNode>::pointer > RayNodePtr_List;


class pmRayTree
{

public:
pmRayTree();

void setAbstractionLevel(int level);

/** 
 * Establishes the folder where the files of rays are
 */
// void setFolder(string folder);

/**
 * file_rays must name the first resolution, for example rays_kinect_0.dat 
 * It is very important that the file ends with "_0.dat"
 * Then the function automatically will search for rays_kinect_1.dat
 */
void rtGenerateRaysTree(string file_rays);

void rtTraverseWithInfo();

//list<pmRayNode>::pointer getRootPtr();

//pmRayNode getNodeFromPtr(list<pmRayNode>::pointer ptr);

protected:

/**
 * Adds a set of rays R as leafs
 * Clears the tree
 * Returns a list of pointers to the created leafs
 */
void rtAddLeafsToTree( vector< boost::numeric::ublas::matrix<double> > & R, std::list< std::list<pmRayNode>::pointer > & leafsPtrList);

void rtAddNodesToTree( RaysVector &R,  RayNodePtr_List &node_ptr_list);

void rtAddRoot(RayNodePtr_List &node_ptr_list);

/**
 * 
 */
void rtLinkChildrenWithParents(RayNodePtr_List children, RayNodePtr_List parents);

/**
 * We are assuming that the rays are normalized.
 */
list<pmRayNode>::pointer rtGetNearestRay(std::list<pmRayNode>::pointer node_ptr, RayNodePtr_List node_ptr_list);


bool rtReadRays(string file_name, vector< boost::numeric::ublas::matrix<double> > &R);   

private:
   bool rtHasRoot;
   bool hasAbstractionLevel;
   
   int abstractionLevel;
   
   list<pmRayNode> rtRaysTree;
   list<pmRayNode>::pointer rtRootPtr;
   
   /**
    * Gets the name of a rays file for a given abstraction level 
    */
   string rtGetNameForRaysFile(string filename_cero, int level);
};

#endif // PMRAYTREE_H
