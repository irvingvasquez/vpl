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


#include "vptriangle.h"

vpTriangle::vpTriangle()
{

}

vpTriangle::vpTriangle(vpVertex v1, vpVertex v2, vpVertex v3)
{
  a = v1;
  b = v2;
  c = v3;
}



bool vpTriangleList::saveToMSLTriangle(string file_name)
{
  ofstream file;
  vpTriangleList::iterator triangle_it;
  
  cout << "Saving data..." << endl;
  file.open(file_name.c_str());
  
  if(file.is_open()){
    triangle_it = this->begin();
    int i = 0;
    while(triangle_it != this->end()){
      file << "(" ;
      file << triangle_it->a.x << ", " << triangle_it->a.y << ", " << triangle_it->a.z;
      file << ")" ;
      
      file << " (" ;
      file << triangle_it->b.x << ", " << triangle_it->b.y << ", " << triangle_it->b.z;
      file << ")" ;
      
      file << " (" ;
      file << triangle_it->c.x << ", " << triangle_it->c.y << ", " << triangle_it->c.z;
      file << ")" ;
      
      file << endl;
      triangle_it ++;
      
    }
    file.close();
  }
  else {
    std::cout << "Error: unable to open file.\n";
    return false;
  }
  
  std::cout << "Last point cloud saved. " << file_name << "\n";
  return true;
}


bool vpTriangleList::saveToFile(string file_name)
{
  ofstream file;
  vpTriangleList::iterator triangle_it;
  
  cout << "Saving data..." << endl;
  file.open(file_name.c_str());
  
  if(file.is_open()){
    triangle_it = this->begin();
    int i = 0;
    while(triangle_it != this->end()){
      file << triangle_it->a.x << " " << triangle_it->a.y << " " << triangle_it->a.z << " ";
      
      file << triangle_it->b.x << " " << triangle_it->b.y << " " << triangle_it->b.z << " ";

      file << triangle_it->c.x << " " << triangle_it->c.y << " " << triangle_it->c.z;
      
      file << endl;
      triangle_it ++;
      
    }
    file.close();
  }
  else {
    std::cout << "Error: unable to open file.\n";
    return false;
  }
  
  std::cout << "Last point cloud saved. " << file_name << "\n";
  return true;
}



bool vpTriangleList::readFile(string file_name)
{  
  double value;
  double x,y,z;
  vpVertex a,b,c;
  
  ifstream file(file_name.c_str());
  
  if(file.is_open()){
    //TODO: verify when the file it is good, is it loading the right number of points?
    this->clear();
    
    while(file >> x){	
        // vertex 1
        file >> y;
        file >> z;
	a.setCoordinates(x,y,z);
	
	//vertex 2
	file >> x;
        file >> y;
        file >> z;
	b.setCoordinates(x,y,z);
	
	//vertex 3
	file >> x;
        file >> y;
        file >> z;
	c.setCoordinates(x,y,z);
	
	vpTriangle tris(a,b,c);
	
	this->push_back(tris);
    }
    
    file.close();
    return true;
  } else {
    cout << "Unable to open file" << endl;
    return false;
  }
}


