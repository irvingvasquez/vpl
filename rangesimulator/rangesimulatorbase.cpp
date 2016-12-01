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


#include "rangesimulatorbase.h"

RangeSimulatorBase::RangeSimulatorBase()
{
  scanIdCounter = 0;
}

void RangeSimulatorBase::setConfigFolder(std::string folder)
{
  configFolder = folder;
}

void RangeSimulatorBase::setDataFolder(std::string folder)
{
  dataFolder = folder;
}


bool RangeSimulatorBase::init()
{
//    std::string config_file(configFolder);
//    config_file.append("/");
//    config_file.append("partialModelConfig.ini");
//   
   result_file.clear();
   result_file.assign(dataFolder);
   result_file.append("/reconstruction_benchmark.dat");
}


bool RangeSimulatorBase::takeAndSaveScan(ViewStructure v, std::string file_name)
{

}


bool RangeSimulatorBase::takeAndSaveScan(boost::numeric::ublas::matrix< double > htm, std::string &scan_name, std::string &origin_name)
{

}


bool RangeSimulatorBase::loadModel(std::string file)
{

}


bool RangeSimulatorBase::save(std::string file)
{

}


bool RangeSimulatorBase::saveReconstructionData()
{
  resultList.saveList(result_file);
}
