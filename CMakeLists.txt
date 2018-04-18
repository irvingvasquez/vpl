cmake_minimum_required(VERSION 2.6)

### VIEW PLANNING LIBRARY ###
project(VPL)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(BUILD_PARTIAL_MODEL true)

set(BUILD_VP2 true)

set(BUILD_VIEW_PLANNING true)

# set this as false if you are going to use another range simulator
set(BUILD_RANGE_SIM false)

# set this as false if you do not want the app examples (under development)
set(BUILD_APPS true)

# View/state planning It requires additional libraries, I recommend to put this as false (under development)
set(BUILD_NBVS false)

# Configure the path to the octomap library
set(OCTOMAP_LIB_PATH /home/irving/projects/ViewPlanning/octomap-devel/lib)
set(OCTOMAP_INCLUDE_PATH /home/irving/projects/ViewPlanning/octomap-devel/octomap/include)


if(BUILD_PARTIAL_MODEL)
  add_subdirectory(partialmodel)
endif()

if(BUILD_VP2)
  add_subdirectory(viewplanning2)
endif()


if(BUILD_VIEW_PLANNING)
   add_subdirectory(viewplanning)
   
   if(BUILD_RANGE_SIM)
      add_subdirectory(rangesimulator)
   endif()
   
   if(BUILD_NBVS)
      add_subdirectory(nbvs_planning)
   endif()
endif()

if(BUILD_APPS)
   add_subdirectory(apps)
endif()



add_subdirectory(tbplanning)