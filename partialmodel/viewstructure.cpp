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
  pose_lenght = 6;

  HTM(0,0) = 0.0;
  HTM(0,1) = 0.0;
  HTM(0,2) = 0.0;
  HTM(0,3) = 0.0;

  HTM(1,0) = 0.0;
  HTM(1,1) = 0.0;
  HTM(1,2) = 0.0;
  HTM(1,3) = 0.0;

  HTM(2,0) = 0.0;
  HTM(2,1) = 0.0;
  HTM(2,2) = 0.0;
  HTM(2,3) = 0.0;

  HTM(3,0) = 0.0;
  HTM(3,1) = 0.0;
  HTM(3,2) = 0.0;
  HTM(3,3) = 1.0;
}


void ViewStructure::setPose(std::vector< double > pose)
{
  if(pose.size()!=6)
    return;
  
  this->w = pose;
}

void ViewStructure::setPose(double x, double y, double z, double yaw, double pitch, double roll)
{
  w[0] = x;
  w[1] = y;
  w[2] = z;
  w[3] = yaw;
  w[4] = pitch;
  w[5] = roll;
}


// WARNING added a comparison over w also
bool ViewStructure::operator==(const ViewStructure& theOther) const
{
  if(q.size() != theOther.q.size())
    return false;
  
  for(int i =0; i<q.size(); i++){
    if(q[i] != theOther.q[i])
      return false;
  }
  
  if(w.size() != theOther.w.size())
    return false;
  
  for(int i =0; i<w.size(); i++){
    if(w[i] != theOther.w[i])
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
  //printVector(best_view.q);
  //std::cout << "Max eval :" << max_eval;
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
  //printVector(best_view.q);
  //std::cout << "Max eval :" << max_eval;
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
  
  out << "Robot Configuration: ";
  for(it = view.q.begin(); it!= view.q.end(); it++){
    out << *it << " ";
  } 
  out << " \n" <<std::endl;
  
  out << "Sensor Pose: " << "x:" << view.w[0] << "  y:" << view.w[1] << "  z:" << view.w[2] << "\t\t"
	<< "  yaw(z):" << view.w[3] << "  pitch(y):" << view.w[4]   << "  roll(x):" << view.w[5] <<std::endl;	
  out << "Sesor Orientation in deg: " << view.w[3] * 180/M_PI << " " << view.w[4] * 180/M_PI << " " << view.w[5] * 180/M_PI <<std::endl;
  out << "Occupied:" << view.n_occupied << " \tUnknown:" << view.n_unknown  << " \tUtility:" << view.eval;
  	
  out <<std::endl;
  
}

