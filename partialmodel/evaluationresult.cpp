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


#include "evaluationresult.h"

EvaluationResult::EvaluationResult()
{
  n_lost = 0;
  n_occupied = 0;
  n_occupied_scene = 0;
  n_unknown = 0;
  n_unknown_scene = 0;
  n_rear_side = 0;
  evaluation = 0.0;
  computation_time = 0.0;
}


void EvaluationResult::clear()
{
  n_lost = 0;
  n_occupied = 0;
  n_occupied_scene = 0;
  n_unknown = 0;
  n_unknown_scene = 0;
  evaluation = 0.0;
  computation_time = 0.0;
}

void EvaluationResult::addVoxelAmouts(EvaluationResult A)
{
  n_lost += A.n_lost;
  n_occupied += A.n_occupied;
  n_occupied_scene += A.n_occupied_scene;
  n_unknown += A.n_unknown;
  n_unknown_scene += A.n_unknown_scene;
}

