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


#ifndef R3DDIRECTPOSITIONING_H
#define R3DDIRECTPOSITIONING_H

#include "reconstructor3d.h"


class R3DDirectPositioning : public Reconstructor3D
{

public:
  R3DDirectPositioning(RobotSensor *rs, NBVPlanner *p);
  
  virtual bool positioning(ViewStructure v);

  virtual bool updateModel(std::string scan_name, std::string origin_name);
  
  virtual bool init();
  
  /** 
   * Reference object points filename. The points are used to compare the percentage of reconstruction
   */
  std::string ref_obj_pts_fn;
  
  /** 
   * Target points filename (Accumulated points from the reconstruction). The points are used to compare the percentage of reconstruction
   */
  std::string tar_pts_fn;
  
  /**
   * Reconstruction percentage filename
   */
  std::string reconstruction_perc_fn;
  
  /**
   * gap for determinig if two points match. Recosntruction percentage calculation
   */
  double gap_points;
  
  
  
protected:
  
  
};


class Reconstructor3DRobot: public Reconstructor3D
{
public:
  
  Reconstructor3DRobot(RobotSensor* rs, NBVPlanner* p);
  
  virtual bool positioning(ViewStructure v);
  
  virtual bool takeScan(std::string& scan_name, std::string& origin_name);
  
  virtual bool updateModel(std::string scan_name, std::string origin_name);
  
protected:
  
  //   void segmentObject(std::string scan_name);
};


class Reconstructor3DExpUtility: public R3DDirectPositioning
{
public:
Reconstructor3DExpUtility(RobotSensor* rs, NBVPlanner* p);
  
virtual bool positioning(ViewStructure v);

};

#endif // R3DDIRECTPOSITIONING_H
