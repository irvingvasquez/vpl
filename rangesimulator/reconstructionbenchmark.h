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


#ifndef RECONSTRUCTIONBENCHMARK_H
#define RECONSTRUCTIONBENCHMARK_H

class ReconstructionBenchmark
{

public:
ReconstructionBenchmark();

/**
 * The coverage ci  describes, what percentage of the reference surface is covered by reconstructed points. 
 */
float c;

/** 
  * Average distance of neighboring points d av
  */
float d;

/**
 * Average error
 */
float e;

/**
 * Number of views
 */
  int v;

};

#endif // RECONSTRUCTIONBENCHMARK_H
