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


#include "evaluationresultvector.h"

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
    std::cout << "Unable to save file " << file_name.c_str() << std::endl;
  }
  
}
