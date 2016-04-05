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


#ifndef VPTRIANGLE_H
#define VPTRIANGLE_H

#include <vector>
#include <list>
#include <string>
#include <fstream>

#include "vpvertex.h"
#include "vpfilereader.h"

using namespace std;

class vpTriangle
{

public:

vpTriangle();
vpTriangle(vpVertex v1, vpVertex v2, vpVertex v3);

vpVertex a;
vpVertex b;
vpVertex c;

};


class vpTriangleList : public std::list<vpTriangle>
{
public:
  /** 
   * Save to raw File
   */
  bool saveToFile(string file_name);
  
  bool saveToMSLTriangle(string file_name);
  
  /**
   * Reads a raw triangle file
   */
  bool readFile(string file_name);
  
};


#endif // VPTRIANGLE_H
