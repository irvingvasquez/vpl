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


#include "rssimulated.h"


RSSimulated::RSSimulated():RangeSensor()
{

}


bool RSSimulated::init()
{
  RangeSensor::init();
  
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("rangeSensor.ini");
  
  dictionary * ini_file;
    
  ini_file = iniparser_load(config_file.c_str());
  if (ini_file ==NULL ) {
      fprintf(stderr, "cannot parse file: %s\n", config_file.c_str());
      return false ;
  }
    //iniparser_dump(ini_file, stderr);
  
  scan_file.clear();
  scan_file.assign(iniparser_getstring(ini_file, "rangeSensor:scan_file", "error"));

  return true;
}




long int RSSimulated::getPoints(std::vector< mrpt::poses::CPoint3D >& points)
{
  /// Read points from file
  
  std::cout << "Range Sensor Simulated: " << scan_file.c_str() << std::endl;
  std::cout << "Push any key to continue..." ;

  //WARNING!!!
  std::string out_file(dataFolder);
  out_file.append("/current_scan.dat");
  std::string comm;
  comm.assign("/home/irving/projects/PCLProg/build/pcd2dat /home/irving/Blensor/scans/table_tof_noise00000.pcd ");
  comm.append(out_file);
  
  std::system(comm.c_str());
  sleep(3);
  
//   getchar();
//   getchar();
  
  double x,y,z;
  vpFileReader reader;
  std::vector< std::vector<double> > data;
  std::vector< std::vector<double> >::iterator it_data;
  
  std::string header;
  
  /// try 3 times
  int intentos = 0;
  while( reader.readDoubleCoordinates(scan_file, data) == false && intentos < 3){
     intentos ++;
  }
    
  //reader.readDoubleCoordFromPCD(filename, data, header);
  std::cout << data.size() << " readed points." << std::endl;
  
  /// Add points to point cloud
  mrpt::poses::CPoint3D *pt;
  points.clear();
  
  for (it_data = data.begin(); it_data!= data.end(); it_data++){
    x = (*it_data)[0];
    y = (*it_data)[1];
    z = (*it_data)[2];
    
    x = x;
    y = y;
    z = z;
    
    pt = new mrpt::poses::CPoint3D(x,y,z);
    points.push_back(*pt);
    delete pt;
  }
  
  std::system("rm /home/irving/Blensor/scans/*");
  return points.size();
}


