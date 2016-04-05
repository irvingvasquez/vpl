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


#include "volumetricutilityfunction.h"

VolumetricUtilityFunction::VolumetricUtilityFunction()
{
  minimumOverlap = 50;
}


int VolumetricUtilityFunction::evaluate(EvaluationResult& r)
{
  r.evaluation = r.n_unmark;
}

void VolumetricUtilityFunction::setMinimunOverlap(float m)
{
  std::cout << "Utility fuction minimunOverlap set to " << m << std::endl;
  minimumOverlap = m;
}

bool VolumetricUtilityFunction::registrationConstraint(EvaluationResult r)
{
  if(r.n_occupied > minimumOverlap)
    return true;
  else
    return false;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

VUF_OverlapPercent::VUF_OverlapPercent()
{
  minimumOverlap = 50;
}


bool VUF_OverlapPercent::registrationConstraint(EvaluationResult r)
{
  //return VolumetricUtilityFunction::registrationConstraint(r);
  float overlap;
  overlap = (float) r.n_occupied /(r.n_occupied + r.n_unmark);
  overlap = overlap * 100;
  
  //std::cout << r.n_occupied << " overlap: " << overlap << std::endl;
  
  if(overlap >= minimumOverlap)
    return true;
  else 
    return false;
  
}


int VUF_OverlapPercent::evaluate(EvaluationResult& r)
{
  //return VolumetricUtilityFunction::evaluate(r);
    // New surface
  if(r.n_unmark == 0){
	r.evaluation = 0.0;
	return UNFEASIBLE_VIEW;
  } else {	
	if(registrationConstraint(r)){
	  r.evaluation = (float) r.n_unmark;
	  return FEASIBLE_VIEW;
	} else {
	  r.evaluation = 0.0;
	  return UNFEASIBLE_VIEW;
	}
  } 
}
