cmake_minimum_required(VERSION 2.6)

project(VPL)


# set this as false if you are going to use another range simulator
set(RANGE_SIM true)

# set this as false if you do not want the app examples (under development)
set(BUILD_APPS true)

# View/state planning It requires additional libraries, I recommend to put this as false (under development)
set(NBVS true)

# Configure the path to the octomap library
set(OCTOMAP_LIB_PATH /home/irving/projects/ViewPlanning/octomap-devel/lib)
set(OCTOMAP_INCLUDE_PATH /home/irving/projects/ViewPlanning/octomap-devel/octomap/include)

add_subdirectory(partialmodel)

add_subdirectory(viewplanning)

if(RANGE_SIM)
add_subdirectory(rangesimulator)
endif()

if(BUILD_APPS)
add_subdirectory(apps)
endif()

if(NBVS)
  add_subdirectory(nbvs_planning)
endif()
