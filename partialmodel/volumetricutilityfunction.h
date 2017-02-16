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


#ifndef VOLUMETRICUTILITYFUNCTION_H
#define VOLUMETRICUTILITYFUNCTION_H

#include "evaluationresult.h"
#include "viewstructure.h"
#include <iostream>


class VolumetricUtilityFunction
{
public:
VolumetricUtilityFunction();

/**
 * evaluates a view after the raytracing
 * @param r provides to the function the amounts of visible voxel from the candidate view. 
 * Returns if the view feasible
 * The evaluation is returned in @param r.evaluation
 */
virtual int evaluate(EvaluationResult &r);

/**
 * Establishes the minimun overlap to satisfy the registration constraint
 * @param m Number of occupied voxels
 */
void setMinimunOverlap(float m);

/**
 * Verify if the registration constraint is fulfilled
 */
virtual bool registrationConstraint(EvaluationResult r);

protected:
  float minimumOverlap;
};


class VUF_OverlapPercent : public VolumetricUtilityFunction
{
public:
VUF_OverlapPercent();

/**
 * Verify if the registration constraint is fulfilled
 */
virtual bool registrationConstraint(EvaluationResult r);

virtual int evaluate(EvaluationResult &r);

};

#endif // VOLUMETRICUTILITYFUNCTION_H
