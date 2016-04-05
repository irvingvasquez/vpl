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


#include "pmraytree.h"

pmRayTree::pmRayTree()
{
  rtHasRoot = false;
  hasAbstractionLevel = false;
//   hasFolder = false;
  abstractionLevel = 0;
}

void pmRayTree::setAbstractionLevel(int level)
{
  if(level < 0){
    cout << "ERROR: Abstraction level must be higher than 0" << endl;
    exit(0);
  }
  
  abstractionLevel = level;
  hasAbstractionLevel = true;
}

// // void pmRayTree::setFolder(string folder)
// // {
// //   raysFileFolder.assign(folder);
// //   hasFolder = true;
// // }


bool pmRayTree::rtReadRays(string file_name, vector< boost::numeric::ublas::matrix<double> > & R)
{
  double x_r, y_r, z_r;
  long int n_rays;
  long int readed_rays = 0;
  R.clear();
  boost::numeric::ublas::matrix<double> ray(4,1);
  
  cout << "Reading file: " << file_name << endl;
  
  ifstream file(file_name.c_str());
  if(file.is_open()){
    //file >> n_rays;
    file >> x_r;
    
    while(file.good()){
        
	file >> y_r;
	file >> z_r;
	ray(0,0) = x_r;
	ray(1,0) = y_r;
	ray(2,0) = z_r;
	ray(3,0) = 1;
	
	R.push_back(ray);
	
	readed_rays ++;
	
	file >> x_r;
    }
    file.close();
  } else {
    cout << "Unable to open file " << file_name.c_str() << endl;
    return false;
  }
  
  //cout << "done." << endl;
  cout << "Readed rays: "<< readed_rays << endl;
  return true;
}

void pmRayTree::rtAddLeafsToTree(vector< boost::numeric::ublas::matrix< double > >& R, std::list< pmRayNode* >& leafsPtrList)
{
  rtRaysTree.clear();
  leafsPtrList.clear();
  list<pmRayNode>::pointer node_ptr;
  
  vector< boost::numeric::ublas::matrix< double > >::iterator it_rays;
  for(it_rays = R.begin(); it_rays!=R.end(); it_rays++){
    pmRayNode node(*it_rays);  
    node.nLeafs = 1; // it is 1 because byitself is a leaf if there is no more links
    rtRaysTree.push_back(node);
    node_ptr = &rtRaysTree.back();
    leafsPtrList.push_back(node_ptr);
  }
}


void pmRayTree::rtAddNodesToTree(RaysVector& R, RayNodePtr_List& node_ptr_list)
{
  node_ptr_list.clear();
  list<pmRayNode>::pointer node_ptr;
  
  RaysVector::iterator it_rays;
  //vector< boost::numeric::ublas::matrix< double > >::iterator it_rays;
  for(it_rays = R.begin(); it_rays!=R.end(); it_rays++){
    pmRayNode node(*it_rays);
    rtRaysTree.push_front(node);
    node_ptr = &rtRaysTree.front();
    node_ptr_list.push_back(node_ptr);
  }
}


void pmRayTree::rtGenerateRaysTree(string file_rays)
{
  int l=0;
  RaysVector R_l;
  RayNodePtr_List children_ptr_list;
  RayNodePtr_List parent_ptr_list;
  string name_for_level;
  
  rtReadRays(file_rays, R_l);
  rtAddLeafsToTree(R_l, children_ptr_list);
  l++;
  
  //rtTraverseWithInfo();
  
  while(l <= abstractionLevel){
    // generate nodes
    name_for_level = rtGetNameForRaysFile(file_rays, l);
    rtReadRays(name_for_level, R_l);
    rtAddNodesToTree(R_l,parent_ptr_list);
    
    //link nodes
    rtLinkChildrenWithParents(children_ptr_list, parent_ptr_list);
    
    //update references
    children_ptr_list.clear();
    children_ptr_list.insert(children_ptr_list.begin(), parent_ptr_list.begin(), parent_ptr_list.end());
    l++;
    
    //rtTraverseWithInfo();
  }
  
  //add root
  rtAddRoot(children_ptr_list);
  
  rtTraverseWithInfo();
}


void pmRayTree::rtLinkChildrenWithParents(RayNodePtr_List children, RayNodePtr_List parents)
{
  RayNodePtr_List::iterator children_it;
  RayNodePtr_List::iterator parents_it;
  list<pmRayNode>::pointer np; // nearest parent
  
  for(children_it = children.begin(); children_it != children.end(); children_it ++){
    np = rtGetNearestRay(*children_it, parents);
    
    //link
    (*children_it)->parent = np;
    np->childrenPtrList.push_back((*children_it));
    np->children_count++;
    np->nLeafs = np->nLeafs + (*children_it)->nLeafs;

  }
}

pmRayNode* pmRayTree::rtGetNearestRay(pmRayNode* node_ptr, RayNodePtr_List node_ptr_list)
{
  //cout << "Get nearest ray." << endl;
  RayNodePtr_List::iterator list_it;
  cout << "Node: " << node_ptr->ray.x() << " " << node_ptr->ray.y() << " " << node_ptr->ray.z() << endl;
  
  pmRayNode* nn = NULL;
  double distance ;
  //double angle;
  double min_dist= 32000;
  for(list_it = node_ptr_list.begin(); list_it!= node_ptr_list.end(); list_it++ ){
    // the distance iw the angle between vectors
    distance = node_ptr->angleTo(*(*list_it));
    if(distance < 0)
      distance = distance * (-1);
    
    //cout << "Parent: " << (*list_it)->ray.x() << " " << (*list_it)->ray.y() << " " << (*list_it)->ray.z() << "\t" << "d: " << distance <<endl;
    if(distance < min_dist)
    {
      nn = *list_it;
      min_dist = distance;
    }
  }
  
  //cout << "Selected parent: " << nn->ray.x() << " " << nn->ray.y() << " " << nn->ray.z() << endl << endl;
  return nn;
}



string pmRayTree::rtGetNameForRaysFile(string filename_cero, int level)
{
  
  string name;
  string extension;
  string name_for_level;
  
  size_t found;
  found = filename_cero.find_last_of("0.dat");
  name.assign(filename_cero.substr(0,found-4));
  extension.assign(".dat");
  
  ostringstream oss;
  oss << name << level << extension;
  name_for_level.assign(oss.str());
  
  return name_for_level;
}

void pmRayTree::rtTraverseWithInfo()
{
  list<pmRayNode>::iterator node_it;
  
  int i;
  int counter = 0;
   
  cout << "-------------------------------------\n" << endl;
  cout << "	rtRaysTree\n";
  cout << "-------------------------------------\n" << endl;
  
  for(node_it = rtRaysTree.begin(); node_it != rtRaysTree.end(); node_it++){
    cout.precision(3);
    cout << fixed << node_it->ray.x() << " " << fixed << node_it->ray.y() << " " << fixed << node_it->ray.z() << "\t";
    cout << "Children count: " << node_it->children_count << "\t";
    cout << "Leafs: " << node_it->nLeafs << endl;
    counter ++;
    
    if (counter == 100){
      cout << "press any key to continue..." << endl;
      getchar();
      counter = 0;
    }
  }
  cout << "-------------------------------------" << endl;
}


void pmRayTree::rtAddRoot(RayNodePtr_List& node_ptr_list)
{
  pmRayNode root(0,0,0);
  list<pmRayNode>::pointer root_ptr;
  rtRaysTree.push_front(root);
  root_ptr = &rtRaysTree.front();
  
  RayNodePtr_List::iterator node_it;
  for(node_it = node_ptr_list.begin(); node_it != node_ptr_list.end(); node_it++){
    root_ptr->children_count ++;
    root_ptr->childrenPtrList.push_back( (*node_it) );
    root_ptr->nLeafs = root_ptr->nLeafs + (*node_it)->nLeafs;
    (*node_it)->parent = root_ptr;
  }
  
  rtRootPtr = root_ptr;
}

