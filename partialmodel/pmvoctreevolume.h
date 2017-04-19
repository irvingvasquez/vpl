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


#ifndef PMVOCTREEVOLUME_H
#define PMVOCTREEVOLUME_H

#include "pmvoctree.h"


class PMVOctreeVolume : public PMVOctree
{

public:
PMVOctreeVolume();

protected:
  
  /**
  * Performs a ray tracing to gather how much voxels ware touched. It uses the translation of the HTM like origin of the ray tracing.
  * Returns false if the origin is not free.
  * @param m [in] Homogenous transformation matrix. It will be applied to the sensor model
  * @param n_occupied [out]
  * @param n_unknown [out]
  */
  bool rayTracingHTM(boost::numeric::ublas::matrix<double> m, EvaluationResult &result);
  
};

#endif // PMVOCTREEVOLUME_H
