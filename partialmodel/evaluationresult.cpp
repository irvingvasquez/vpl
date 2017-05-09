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




EvaluationResultVector::EvaluationResultVector()
{

}

void EvaluationResultVector::saveVector(std::string file_name)
{
  std::cout << "Saving Evaluation Vector" << std::endl;
  std::cout << "File: " << file_name.c_str() << std::endl;
  std::cout << this->size() << " elements" << std::endl;
 std::ofstream myfile (file_name.c_str());
  
  EvaluationResultVector::iterator result_it;
  
  if (myfile.is_open())
  {
    //myfile << "";
    myfile << "# occupied occupied_scene unmark unmark_scene ray_lost computation_time" << std::endl;
    for(result_it = this->begin(); result_it != this->end(); result_it++){
      myfile << result_it->n_occupied << " " << result_it->n_occupied_scene << " " << result_it->n_unknown 
	      << " " << result_it->n_unknown_scene << " " << result_it->n_lost << " " << result_it->computation_time << std::endl;
    }
    myfile.close();
  }
  else{ 
    std::cout << "Unable to open file";
    getchar();
  }
  
}
