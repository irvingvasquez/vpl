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


#include "viewstructure.h"

ViewStructure::ViewStructure():
q(6),
w(6),
HTM(4,4)
{
n_occupied = 0;
n_unknown = 0;
d = 0;
type = 0;
eval = 0.0;
}

bool ViewStructure::operator==(const ViewStructure& theOther) const
{
  if(q.size() != theOther.q.size())
    return false;
  
  for(int i =0; i<q.size(); i++){
    if(q[i] != theOther.q[i])
      return false;
  }
  
  return true;
}


ViewStructure bestViewOfList(std::list< ViewStructure >& viewsList)
{
  std::cout << "Best View of List" << std::endl;
  
  
  std::list<ViewStructure>::iterator it_list;
  ViewStructure best_view;
  double max_eval = -1;
  it_list = viewsList.begin();
  while(it_list != viewsList.end()){
    //cout << "Encoders: " <<std::endl;
    //printIntVector(it_list->q);
    //cout << "eval: " << it_list->eval;
    if(it_list->eval > max_eval){
      max_eval = it_list->eval;
      best_view = *it_list;
    }
    it_list++;
  }
  
  //cout << "B: " <<std::endl;
  printVector(best_view.q);
  std::cout << "Max eval :" << max_eval;
  return best_view;
}


void ViewList::sortHighToLow()
{
  this->sort(compareByEvalInverse);
}


ViewStructure ViewList::getBestView()
{
  //cout << "Best View of List" <<std::endl;
  
  std::list<ViewStructure>::iterator it_list;
  ViewStructure best_view;
  double max_eval = -1;
  it_list = this->begin();
  while(it_list != this->end()){
    //cout << "Encoders: " <<std::endl;
    //printIntVector(it_list->q);
    //cout << "eval: " << it_list->eval;
    if(it_list->eval > max_eval){
      max_eval = it_list->eval;
      best_view = *it_list;
    }
    it_list++;
  }
  
  //cout << "B: " <<std::endl;
  printVector(best_view.q);
  std::cout << "Max eval :" << max_eval;
  return best_view;
}



bool vsReadViewList(std::list< ViewStructure >& views, std::string fileName)
{
  double value_double;
  int value_int;
  long int n_views, ni;
  int size_of_q, qi;
  int size_of_w, wi;
  ViewStructure view;
  std::ifstream file(fileName.c_str());
  
  views.clear();
  
  int i,j;
  views.clear();
  if(file.is_open()){
    //TODO: verify when the file it is good, is it loading the right number of points?
    file >> n_views;
    file >> size_of_q;
    file >> size_of_w;
    
    // this can be speeded up by changins the push back by [index]=
    
    ni = 0;
    while(file.good() && ni < n_views){
      // read q
      view.q.clear();
      for(qi = 0; qi < size_of_q; qi++){
	file >> value_double;
	view.q.push_back(value_double);
      }
      
      //read w;
      view.w.clear();
      for(wi = 0; wi < size_of_w; wi++){
	file >> value_double;
	view.w.push_back(value_double);
      }
      
      //read HTM
      for (i=0; i<4; i++){
	for(j=0; j<4; j++){
	  file >> value_double;
	  view.HTM(i,j) = value_double;
	}
      }
      file >> value_int;
      view.type = value_int;
      file >> value_double;
      view.eval = value_double;
      
      views.push_back(view);
      ni ++;
    }
    
    file.close();
    std::cout << "Readed complete. " << views.size() << "views." << std::endl;
    return true;
  } else {
    std::cout << "Unable to open file" << std::endl;
    return false;
  }
}



bool ViewList::read(std::string file_name)
{
  double value_double;
  int value_int;
  long int n_views, ni;
  int size_of_q, qi;
  int size_of_w, wi;
  ViewStructure view;
  std::ifstream file(file_name.c_str());
  
  this->clear();
  
  int i,j;
  this->clear();
  if(file.is_open()){
    //TODO: verify when the file it is good, is it loading the right number of points?
    file >> n_views;
    file >> size_of_q;
    file >> size_of_w;
    
    // this can be speeded up by changins the push back by [index]=
    
    ni = 0;
    while(file.good() && ni < n_views){
      // read q
      view.q.clear();
      for(qi = 0; qi < size_of_q; qi++){
	file >> value_double;
	view.q.push_back(value_double);
      }
      
      //read w;
      view.w.clear();
      for(wi = 0; wi < size_of_w; wi++){
	file >> value_double;
	view.w.push_back(value_double);
      }
      
      //read HTM
      for (i=0; i<4; i++){
	for(j=0; j<4; j++){
	  file >> value_double;
	  view.HTM(i,j) = value_double;
	}
      }
      file >> value_int;
      view.type = value_int;
      file >> value_double;
      view.eval = value_double;
      
      this->push_back(view);
      ni ++;
    }
    
    file.close();
    std::cout << "Readed complete. " << this->size() << "views." << std::endl;
    return true;
  } else {
    std::cout << "Unable to open file" << std::endl;
    return false;
  }
}


bool ViewList::saveAsMSLStates(std::string file_name)
{
  if(this->size() == 0){
    std::cout << "Empty list" << std::endl;
    return false;
  }
  
  std::list< ViewStructure >::iterator it_v;
  std::vector<double>::iterator it_q;
  std::vector<double>:: iterator it_w;
  int i,j;  
  
  std::ofstream myfile (file_name.c_str());
  if (myfile.is_open())
  {
    it_v = this->begin();
    for (it_v = this->begin(); it_v!= this->end(); it_v++){
      // save q
      myfile << it_v->q.size() << " ";
      for(it_q = it_v->q.begin(); it_q != it_v->q.end(); it_q ++){
	myfile << *it_q << " ";
      }
      myfile << std::endl;
    }
    
    myfile.close();
    std::cout << "Views saved. File: " << file_name.c_str() << std::endl;
    return true;
  }
  
  else std::cout << "Unable to open file" << std::endl;
  return false;
}


bool ViewList::save(std::string file_name)
{
  
  if(this->size() == 0){
    std::cout << "Empty vector" << std::endl;
    return false;
  }
  
  std::list< ViewStructure >::iterator it_v;
  std::vector<double>::iterator it_q;
  std::vector<double>:: iterator it_w;
  int i,j;  
  
  std::ofstream myfile (file_name.c_str());
  if (myfile.is_open())
  {
    myfile << this->size() << std::endl;
    it_v = this->begin();
    myfile << it_v->q.size() << std::endl;
    myfile << it_v->w.size() << std::endl;
    
    for (it_v = this->begin(); it_v!= this->end(); it_v++){
      // save q
      for(it_q = it_v->q.begin(); it_q != it_v->q.end(); it_q ++){
	myfile << *it_q << " ";
      }
      myfile <<std::endl;
      
      // save w
      for(it_w = it_v->w.begin(); it_w != it_v->w.end(); it_w ++){
	myfile << *it_w << " ";
      }
      myfile <<std::endl;
      
      // save HTM
      for(i = 0; i<4 ;i++){
	for (j=0; j<4; j++){
	  myfile << it_v->HTM(i,j) << " ";
	}
	myfile <<std::endl;
      }
      
      //save eval
      //cout << "eval " << it_v->eval <<std::endl;
      myfile << it_v->type <<std::endl;
      myfile << it_v->eval <<std::endl;
    }
    
    myfile.close();
    std::cout << "Views saved. File: " << file_name.c_str() <<std::endl;
    return true;
  }
  
  else std::cout << "Unable to open file" <<std::endl;
  return false;
}



bool vsSaveViewList(std::list< ViewStructure > views, std::string fileName)
{
if(views.size() == 0){
    std::cout << "Empty vector" << std::endl;
    return false;
  }
  
  std::list< ViewStructure >::iterator it_v;
  std::vector<double>::iterator it_q;
  std::vector<double>:: iterator it_w;
  int i,j;  
  
  std::ofstream myfile (fileName.c_str());
  if (myfile.is_open())
  {
    myfile << views.size() << std::endl;
    it_v = views.begin();
    myfile << it_v->q.size() << std::endl;
    myfile << it_v->w.size() << std::endl;
    
    for (it_v = views.begin(); it_v!= views.end(); it_v++){
      // save q
      for(it_q = it_v->q.begin(); it_q != it_v->q.end(); it_q ++){
	myfile << *it_q << " ";
      }
      myfile << std::endl;
      
      // save w
      for(it_w = it_v->w.begin(); it_w != it_v->w.end(); it_w ++){
	myfile << *it_w << " ";
      }
      myfile << std::endl;
      
      // save HTM
      for(i = 0; i<4 ;i++){
	for (j=0; j<4; j++){
	  myfile << it_v->HTM(i,j) << " ";
	}
	myfile << std::endl;
      }
      
      //save eval
      //cout << "eval " << it_v->eval <<std::endl;
      myfile << it_v->type << std::endl;
      myfile << it_v->eval << std::endl;
    }
    
    myfile.close();
    std::cout << "Views saved. File: " << fileName.c_str() << std::endl;
    return true;
  }
  else std::cout << "Unable to open file" << std::endl;
  return false;
}




bool compareByEval(ViewStructure first, ViewStructure second)
{
  if(first.eval < second.eval)
    return true;
  else
    return false;
}

bool compareByEvalInverse(ViewStructure first, ViewStructure second)
{
  if(first.eval > second.eval)
    return true;
  else
    return false;

}


void orderViewsHighToLow(std::list< ViewStructure >& viewsList)
{
  viewsList.sort(compareByEvalInverse);
}


void getNViewsFromList(std::list< ViewStructure >& viewsList, int n, std::list< ViewStructure >& result)
{
 // if(n<1)
 //   return;
  std::list<ViewStructure>::iterator it;
  it = viewsList.begin();
  int i = 0;
  result.clear();
  while(it != viewsList.end() && i<n){
    result.push_back(*it);
    it++;
    i++;
  }
}

std::ostream& operator << (std::ostream& out, ViewStructure& view)
{
  std::vector< double >::iterator it;
  
  out << "Configuration: ";
  for(it = view.q.begin(); it!= view.q.end(); it++){
    out << *it << " ";
  } 
  out << " \n" <<std::endl;
  
  out << "Pose: " << "x:" << view.w[0] << "  y:" << view.w[1] << "  z:" << view.w[2] << "\t\t"
	<< "  yaw(z):" << view.w[3] << "  pitch(y):" << view.w[4]   << "  roll(x):" << view.w[5] <<std::endl;	
  out << "Pose in deg: " << view.w[3] * 180/M_PI << " " << view.w[4] * 180/M_PI << " " << view.w[5] * 180/M_PI <<std::endl;
  out << "Occupied:" << view.n_occupied << " \tUnknown:" << view.n_unknown  << " \tUtility:" << view.eval;
  	
  out <<std::endl;
  
}

