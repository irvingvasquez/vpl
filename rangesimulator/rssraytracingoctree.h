
#ifndef RSSRAYTRACINGOCTREE_H
#define RSSRAYTRACINGOCTREE_H

#include "rangesimulatoroctree.h"
#include <rangesensor.h>

/**
 * Range sensor simulated with raytracing over octree
 */
class RSSRayTracingOCtree: public RangeSensor
{
public:
  RSSRayTracingOCtree();
  
  virtual bool init();
  
  virtual long int getPoints(std::vector< mrpt::poses::CPoint3D >& points);

protected:
  //std::string scan_file;
  
  RangeSimulatorOctree rangeSensor;
};


#endif