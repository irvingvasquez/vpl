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


#ifndef RANGESIMULATORBASE_H
#define RANGESIMULATORBASE_H

#include <string>
#include <viewstructure.h>
#include <vpfilereader.h>
#include <INIReader.h>

//#include <mrpt/system/filesystem.h>
//#include <mrpt/utils/CConfigFile.h>

#include "recbenchmarklist.h"

//using namespace std;

class RangeSimulatorBase
{

public:
RangeSimulatorBase();

virtual bool init();

virtual bool takeAndSaveScan( ViewStructure v, std::string file_name );

virtual bool takeAndSaveScan( boost::numeric::ublas::matrix<double> htm, std::string &scan_name, std::string &origin_name );

virtual bool loadModel(std::string file);

/**
 * Saves the scene
 */
virtual bool save(std::string file);

void setConfigFolder(std::string folder);

void setDataFolder(std::string folder);

protected:
  std::string configFolder;
  std::string dataFolder;
  std::string result_file;
  
  std::string modelFile;
  std::string sensorFile; 
  
  int scanIdCounter;
  
  /**
   * Saves the data of the reconstruction
   * Reconstruction Benchmark
   */
  bool saveReconstructionData();
  
  RecBenchmarkList resultList;
};

#endif // RANGESIMULATORBASE_H
