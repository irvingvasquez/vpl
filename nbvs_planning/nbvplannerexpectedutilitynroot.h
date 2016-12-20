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


#ifndef NBVPLANNEREXPECTEDUTILITYNROOT_H
#define NBVPLANNEREXPECTEDUTILITYNROOT_H

#include "nbvplannerexpectedutility.h"


class NBVPlannerExpectedUtilityNRoot : public NBVPlannerExpectedUtility
{

public:
NBVPlannerExpectedUtilityNRoot(RobotSensor* rs, PartialModelBase* pm);

virtual bool planNBV(ViewStructure& v);

virtual bool init();

 /**
  * Generate a sample given an initial state and a trajectory
 * Returns false if a colision was found
 * Returns in @param prob the natural logaritm of the probability of the path wich generates the random sample
 * and in @param prob_rn the n-th square root is returned
 */
bool generateSampleStateComp(list< MSLVector > trajectory, MSLVector x_0, std::vector< double > nsigmas, MSLVector& sample, double& prob, double &prob_nr, Geom *g);

  /**
     * Algorithm for computing densities of a zero-centered normal distribution
     * Computes the probability of @a of a zero-centered normal distribution with variance @b 
     * returns the n-th root of the probability. Also the probability is in the range [0,1]
     */
double rootProbOf(double a, double b, int n);

};

#endif // NBVPLANNEREXPECTEDUTILITYNROOT_H
