cmake_minimum_required(VERSION 2.6)

project(VPL)

set(NBVS false)

add_subdirectory(partialmodel)

add_subdirectory(viewplanning)

add_subdirectory(rangesimulator)

add_subdirectory(apps)

if(NBVS)
  add_subdirectory(nbvs_planning)
endif()