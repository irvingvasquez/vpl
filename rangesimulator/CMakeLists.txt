cmake_minimum_required(VERSION 2.6)

project(rangesimulator)

########### First configure the following ###########
set(RELATED_DIRS_DIRECTORY ${VPL_SOURCE_DIR}/..)

set(SRC rangesimulatorbase.cpp rangesimulatoroctree.cpp reconstructionbenchmark.cpp recbenchmarklist.cpp)

 #---------------- View Planning Library ---------
include_directories(${VPL_SOURCE_DIR}/viewplanning)
include_directories(${VPL_SOURCE_DIR}/partialmodel)
link_directories(${VPL_SOURCE_DIR}/lib)
set(LIBS_VIEW_PLANNING ViewPlanning PartialModel)

# --------------- Octomap --------------------
include_directories(${RELATED_DIRS_DIRECTORY}/octomap-devel/octomap/include)
link_directories(${RELATED_DIRS_DIRECTORY}/octomap-devel/lib)
set(LIBS_OCTOMAP octomap octomath octovis)

# ---------------- INIPARSER  -----------------
include_directories(${RELATED_DIRS_DIRECTORY}/iniparser/src)
link_directories(${RELATED_DIRS_DIRECTORY}/iniparser)
set(LIBS_INIPARSER iniparser)

# ---------------- MRPT ---------------------
find_package(MRPT REQUIRED)
include_directories(${MRPT_INCLUDE_DIRS})
link_directories(${MRPT_LINK_DIRECTORIES})
#include_directories(/usr/include/mrpt/gui/include)
set(LIBS_MRPT mrpt-base)

#--------------------------------------------
set(LIBS_ALL ${LIBS_VIEW_PLANNING} ${LIBS_OCTOMAP} ${LIBS_INIPARSER} ${LIBS_MRPT}) 

#add_executable(rangesimulator main.cpp ${SRC})
#target_link_libraries(rangesimulator ${LIBS_ALL})

# -------------------------------------------------
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${VPL_SOURCE_DIR}/lib)
add_library(RangeSimulator SHARED ${SRC})
