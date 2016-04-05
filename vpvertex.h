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


#ifndef VPVERTEX_H
#define VPVERTEX_H

#include <iostream>

class vpVertex
{

public:
  vpVertex();
  vpVertex(double coord_x, double coord_y, double coord_z);

  double x;
  double y;
  double z;
  
  void setCoordinates(double coord_x, double coord_y, double coord_z);
};

#endif // VPVERTEX_H
