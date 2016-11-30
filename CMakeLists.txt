cmake_minimum_required(VERSION 2.6)

project(nbvpartialmodel)

#the version number
set (pm_VERSION_MAJOR 1)
set (pm_VERSION_MINOR 0)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/nbvPartialModelConfig.h.in"
  "${PROJECT_BINARY_DIR}/nbvPartialModelConfig.h"
  )
 
# add the binary tree to the search path for include files
# so that we will find nbvPartialModelConfig.h
include_directories("${PROJECT_BINARY_DIR}")


set(SRC_STRUCTURES coctreevpl.cpp)
set(SRC_PARTIAL_MODEL partialmodelbase.cpp pmvolumetric.cpp pmvoctree.cpp pmvoctreeig.cpp pmvoctreevasquez14.cpp)  
# set(SRC_HRT pmraynode.cpp pmraytree.cpp pmvoctreehierarchicalrt.cpp)
set(SRC_VIEW viewstructure.cpp vpfilereader.cpp pmutils.cpp vptriangle.cpp vpvertex.cpp)
set(SRC_OTHER evaluationresult.cpp evaluationresultvector.cpp volumetricutilityfunction.cpp vufobjectfilters.cpp)
set(SRC_ALL ${SRC_PARTIAL_MODEL} ${SRC_VIEW} ${SRC_OTHER} ${SRC_STRUCTURES})
#set(SRC_ALL ${SRC_HRT} )


# ----------------------------- Octomap --------------------
set(OCTOMAP_DIR ${CMAKE_SOURCE_DIR}/../octomap-devel) 
include_directories(${OCTOMAP_DIR}/octomap/include)
link_directories(${OCTOMAP_DIR}/lib ${OCTOMAP_DIR}/octovis/lib)
set(LIBS_OCTOMAP octomap octomath octovis)

#include_directories(/home/irving/projects/octomap-1.7.1/octomap/include)
#link_directories(/home/irving/projects/octomap-1.7.1/lib /home/irving/projects/octomap-1.7.1/octovis/lib)
#set(LIBS_OCTOMAP octomap octomath octovis)


# ---------------------------- INIPARSER --------------------
set(INIPARSER_DIR ${CMAKE_SOURCE_DIR}/../iniparser) 
include_directories(${INIPARSER_DIR}/src)
link_directories(${INIPARSER_DIR})
set(LIBS_INIPARSER iniparser)


#----------------------------- BOOST --------------------
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories( ${Boost_INCLUDE_DIR} )


#-----------------------------------------------
#add_executable(nbvpartialmodel main.cpp ${SRC_ALL})
#target_link_libraries(nbvpartialmodel ${LIBS_OCTOMAP} ${LIBS_INIPARSER})

# -------------------------------------------------
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)
add_library(PartialModel SHARED ${SRC_ALL})


# ///////////////////// other testing programs ////////////////////////////

#add_executable(test_multires_octree test_multires_octree.cpp ${SRC_ALL})
#target_link_libraries(test_multires_octree ${LIBS_OCTOMAP}  ${LIBS_INIPARSER}) 

#add_executable(test_hrt test_hrt.cpp ${SRC_ALL})
#target_link_libraries(test_hrt ${LIBS_OCTOMAP} ${LIBS_INIPARSER} ${Boost_LIBRARIES} -lboost_system)

#add_executable(test_esfera test_esfera_octree.cpp ${SRC_ALL})
#target_link_libraries(test_esfera ${LIBS_OCTOMAP}  ${LIBS_INIPARSER})

#add_executable(test_esfera_multi test_esfera_multioctree.cpp ${SRC_ALL})
#target_link_libraries(test_esfera_multi ${LIBS_OCTOMAP}  ${LIBS_INIPARSER})

add_executable(test_partialmodel test_partialmodel.cpp ${SRC_ALL})
target_link_libraries(test_partialmodel ${LIBS_OCTOMAP}  ${LIBS_INIPARSER})

#add_executable(paint_octree paint_octree.cpp ${SRC_ALL})
#target_link_libraries(paint_octree ${LIBS_OCTOMAP}  ${LIBS_INIPARSER})

#add_executable(raw2triangles raw2triangles.cpp ${SRC_ALL})
#target_link_libraries(raw2triangles ${LIBS_OCTOMAP}  ${LIBS_INIPARSER})
