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



#ifndef VPFILEREADER_H
#define VPFILEREADER_H

#include <vector>
#include <list>
#include <string>

#include <iostream>
#include <fstream>

#include <string.h>

using namespace std;


class vpFileReader
{
public:
  
vpFileReader();
virtual ~vpFileReader();

/**
 * Read all int number from a file
 */
bool readInt(string file_name, vector<int> &data);


/**
 * 
 */ 
template <typename Tipo>
bool readSingleValue(Tipo &value, std::string file_name){ 
  ifstream file(file_name.c_str());
  if(file.is_open()){
    file >> value;
    
    file.close();
    return true;
  } else {
    cout << "Unable to open file " << file_name.c_str() << endl;
    return false;
  }
}



/**
 * 
 */
template <typename TipoDato>
bool readMSLVector(vector<TipoDato> &data, string file_name)
{
  TipoDato value;
  int Elements;
  
  ifstream file(file_name.c_str());
  
  if(file.is_open()){
    file >> Elements;
    
    while(file >> value)
	data.push_back(value);
    
    file.close();
    return true;
  } else {
    cout << "Unable to open file " << file_name.c_str() << endl;
    return false;
  }
}


/**
 * 
 */
template <typename TipoDato>
bool readMSLListVector(std::list < vector<TipoDato> > &data, string file_name)
{
  TipoDato value;
  int Elements;
  std::vector<TipoDato> vec;
  
  ifstream file(file_name.c_str());
  
  if(file.is_open()){
    while(file >> Elements){
      vec.clear();
      vec.resize(Elements);
      
      for(int i = 0; i<Elements; i++){
	file >> value;
	vec[i] = value;
      }
      
      data.push_back(vec);
    }
    
    file.close();
    cout << "File readed, " << data.size() << " vectors loaded" << endl;
    return true;
  } else {
    cout << "Unable to open file " << file_name.c_str() << endl;
    return false;
  }
}


/**
 * Saves a list of T type vectors in MSL format
 */
template <typename T>
bool saveToMSLListVector(std::list< vector<T> > data, string file_name, bool append=false){
  ofstream file;
  
  if(append){
    file.open(file_name.c_str(),ios_base::app);
  }else {
    file.open(file_name.c_str());
  }
  
  int list_size = data.size();
  int i = 0;
  
  if(file.is_open()){
    
    typename std::list< std::vector<T> >::iterator it_lista = data.begin(); 
    
    while(it_lista != data.end()){
      int vec_size = it_lista->size();
      file << vec_size;
      for(int i = 0; i<vec_size; i++){
	file << " " << (*it_lista)[i] ;
      }
      file << endl;
      it_lista ++;
    }
    
    file.close();
    return true;
  } else {
    cout << "Unable to open file " << file_name.c_str() << endl;
    return false;
  }
}


/**
 * Saves a vector of vectors
 */
template <typename T>
bool saveVectorOfVectors(std::vector< vector<T> > data, string file_name, bool append=false){
  ofstream file;
  
  if(append){
    file.open(file_name.c_str(),ios_base::app);
  }else {
    file.open(file_name.c_str());
  }
  
  int list_size = data.size();
  int i = 0;
  
  if(file.is_open()){
    
    typename std::vector< std::vector<T> >::iterator it_lista = data.begin(); 
    
    while(it_lista != data.end()){
      int vec_size = it_lista->size();
      //file << vec_size;
      for(int i = 0; i<vec_size; i++){
	file << (*it_lista)[i] << " "  ;
      }
      file << endl;
      it_lista ++;
    }
    
    file.close();
    return true;
  } else {
    cout << "Unable to open file " << file_name.c_str() << endl;
    return false;
  }
}


/**
 *  Saves a vector in MSL format
 */
template <typename TipoDato>
bool saveToMSLVector(vector<TipoDato> &data, string file_name, bool append= false)
{
  ofstream file;
  
  if(append){
    file.open(file_name.c_str(),ios_base::app);
  }else {
    file.open(file_name.c_str());
  }
  
  int size = data.size();
  int i = 0;
    
  if(file.is_open()){
    file << size;
    
    while(i<size){
	file << " " << data[i] ;
	i++;
    }
    
    file << endl;
    
    file.close();
    return true;
  } else {
    cout << "Unable to open file " << file_name.c_str() << endl;
    return false;
  }
}



/**
 * Reads all double numbers from a file
 */
bool readDouble(string file_name, vector<double> &data);

bool readDoubleCoordFromPCD(string file_name, vector< vector<double> > &data, string &header);

/**
 * Reads all double coordinates.
 */
bool readDoubleCoordinates(string file_name, vector< vector<double> > &data);

bool saveDoubleCoordinates(string file_name, vector< vector<double> > &data);


bool readDoubleCoordinatesComa(string file_name, vector< vector<double> > &data);

bool readDoubleCoordinatesFromWRL(string file_name, vector< vector< double> > &data);

bool saveDoubleCoordinatesAsPCD(string file_name, vector< vector<double> > &data);


bool saveDoubleCoordinatesAsOBJ(string file_name, vector< vector<double> > &data);


/**
 * 
 */
template <typename TipoDato>
bool saveVector(vector<TipoDato> &data, string file_name, bool append= false)
{
  ofstream file;
  
  if(append){
    file.open(file_name.c_str(),ios_base::app);
  }else {
    file.open(file_name.c_str());
  }
  
  int size = data.size();
  int i = 0;
    
  if(file.is_open()){
    while(i<size){
	file << " " << data[i] ;
	i++;
    }
    
    file.close();
    return true;
  } else {
    cout << "Unable to save file " << file_name.c_str() << endl;
    return false;
  }
}


/**
 * Saves a data to a text file,
 */
template <typename TipoDato>
bool saveData2Text(TipoDato data, string file_name, bool append= false, char separator  = ' ')
{
  ofstream file;
  
  if(append){
    file.open(file_name.c_str(),ios_base::app);
  }else {
    file.open(file_name.c_str());
  }
  
  
  if(file.is_open()){
    file << separator << data;
    
    file.close();
    return true;
  } else {
    cout << "Unable to save file " << file_name.c_str() << endl;
    return false;
  }
}


private:
  
bool readLineCoordinatesfromPCD(string file_name, vector<double> &data, string &header);
  
};

#endif // VPFILEREADER_H
//}