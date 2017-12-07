cmake_minimum_required(VERSION 2.6)

project(rangesimulator)

set(SRC rangesimulatorbase.cpp rangesimulatoroctree.cpp reconstructionbenchmark.cpp recbenchmarklist.cpp rssraytracingoctree.cpp)

# --------------- Octomap --------------------
# Octomap
include_directories(${OCTOMAP_INCLUDE_PATH})
link_directories(${OCTOMAP_LIB_PATH})
set(LIBS_OCTOMAP octomap octomath)

#---------------- View Planning Library ---------
include_directories(${VPL_SOURCE_DIR}/partialmodel)
include_directories(${VPL_SOURCE_DIR}/viewplanning)
link_directories(${VPL_SOURCE_DIR}/lib)
set(LIBS_VIEW_PLANNING ViewPlanning PartialModel)

# ---------------- MRPT ---------------------
find_package(MRPT REQUIRED)
include_directories(${MRPT_INCLUDE_DIRS})
link_directories(${MRPT_LINK_DIRECTORIES})
# #include_directories(/usr/include/mrpt/gui/include)
set(LIBS_MRPT mrpt-base)

#--------------------------------------------
set(LIBS_ALL ${LIBS_VIEW_PLANNING} ${LIBS_OCTOMAP} ${LIBS_MRPT}) 

#add_executable(rangesimulator main.cpp ${SRC})
#target_link_libraries(rangesimulator ${LIBS_ALL})

# -------------------------------------------------
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${VPL_SOURCE_DIR}/lib)
add_library(RangeSimulator SHARED ${SRC})
target_link_libraries(RangeSimulator ${LIB_PARTIAL_MODEL} ${LIBS_OCTOMAP} ${LIBS_MRPT})
