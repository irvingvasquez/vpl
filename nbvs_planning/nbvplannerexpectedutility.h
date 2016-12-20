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


#ifndef NBVPLANNEREXPECTEDUTILITY_H
#define NBVPLANNEREXPECTEDUTILITY_H

#include "nbvplannerrgflt.h"


#include <mrpt/utils.h>
#include <mrpt/math.h>
#include <mrpt/system.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;

class NBVPlannerExpectedUtility : public NBVPlannerRGFlt
{

public:
NBVPlannerExpectedUtility(RobotSensor* rs, PartialModelBase* pm);


 /** trata de alcanzar las n vistas*/
 virtual bool planNBV(ViewStructure& v);
 
 /**
 * Generate a sample given an initial state and a trajectory
 * Returns false if a colision was found
 * Returns in @param prob the natural logaritm of the probability of the path wich generates the random sample
 */
 bool generateSampleState(list< MSLVector > trajectory, MSLVector x_0, MSLVector goal, std::vector<double> nsigmas, MSLVector &sample, double &prob, Geom *g);
 
 virtual bool init();

protected:
  int LSamples;
  std::vector< double > Sigmas;
  std::vector <double> Means;
  // usually 2 sigma, it contains the coefficient
  double empiricalRule;
  
  float std_eu_time;
  
  /** 
   * Combines two trajectoryes into one single trajectory
   */
  void combinePolycies(list< MSLVector > &trajectory, list< MSLVector > &policy1, list< MSLVector > &policy2);
  
  /**
     * Algorithm for computing densities of a zero-centered normal distribution
     * Table 5.2 Thrun
     * Computes the probability of @a of a zero-centered normal distribution with variance @b 
     */
  virtual double probabilityOf(double a, double b);
  
  /**
     * Algorithm for computing densities of a zero-centered normal distribution
     * Computes the probability of @a of a zero-centered normal distribution with variance @b 
     * returns the natural logaritm of the probability. Also the probability is in the range [0,1]
     */
  virtual double logProbabilityOf(double a, double b);
  
  /**
   * performs log(x+y) but using log(x) and log(y)
   */
  double logAdd(double logX, double logY);
  
};


class NBVPlannerExpectedUtilityFast : public NBVPlannerExpectedUtility
{
  
public:
  NBVPlannerExpectedUtilityFast(RobotSensor* rs, PartialModelBase* pm);

  virtual bool planNBV(ViewStructure& v);
  
  virtual bool init();
};


class NBVPlannerEUFastBiased : public NBVPlannerExpectedUtilityFast
{
  // TODO
protected:
  virtual bool generateCandidateViews(ViewList &views, Model *model, Geom *geometry);
};


#endif // NBVPLANNEREXPECTEDUTILITY_H
