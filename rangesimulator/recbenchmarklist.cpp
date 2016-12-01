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


#include "recbenchmarklist.h"

RecBenchmarkList::RecBenchmarkList()
{

}


bool RecBenchmarkList::saveList(std::string file_name)
{
  std::list<ReconstructionBenchmark>::iterator it_list;
  
  std::ofstream myfile (file_name.c_str());
  
  if (myfile.is_open())
  {
    myfile << "# Reconstruction Bechmark.\n";
    myfile << "# c d e v.\n";
    
    for(it_list = this->begin(); it_list != this->end(); it_list++){
      myfile << it_list->c << " \t" << it_list->d << "\t" << it_list->e << "\t" << it_list->v << std::endl; 
    }
    
    myfile.close();
  }
  else { 
    std::cout << "Unable to open file" << std::endl;
    return false;
  }
  
  return true;
}

