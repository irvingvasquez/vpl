/*
 * 
 * 
Partial Model Library
Copyright (c) 2016, J. Irving Vasquez ivasquez@ccc.inaoep.mx
Consejo Nacional de Ciencia y Tecnología (CONACYT)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef EVALUATIONRESULT_H
#define EVALUATIONRESULT_H

#include <vector>
#include <iostream>
#include <string>
#include <fstream>


/*
 * Structure that describes how much voxels of each type were collected
 */
class EvaluationResult
{

public:
EvaluationResult();

long int n_occupied;
long int n_unknown;
long int n_occupied_scene;
long int n_unknown_scene;
long int n_lost;
long int n_rear_side;
float evaluation;
double computation_time;

void clear();

/**
 * Adds the voxel amouts
 *  
 * n_lost += A.n_lost;
  n_occupied += A.n_occupied;
  n_occupied_scene += A.n_occupied_scene;
  n_unknown += A.n_unknown;
  n_unknown_scene += A.n_unknown_scene;
 */
void addVoxelAmouts(EvaluationResult A);
};


class EvaluationResultVector : public std::vector<EvaluationResult>
{
public:
EvaluationResultVector();
void saveVector(std::string file_name);

};

#endif // EVALUATIONRESULT_H
