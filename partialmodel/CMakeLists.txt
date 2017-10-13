cmake_minimum_required(VERSION 2.6)

### PARTIAL MODEL PROJECT ###
project(nbvpartialmodel)

set(SRC_STRUCTURES coctreevpl.cpp)
set(SRC_PARTIAL_MODEL partialmodelbase.cpp pmvolumetric.cpp pmvoctree.cpp pmvoctreeig.cpp pmvoctreeigkriegel12.cpp pmvoctreevasquez09.cpp pmvorearsidevoxel.cpp)
set(SRC_VIEW viewstructure.cpp vpfilereader.cpp pmutils.cpp vptriangle.cpp vpvertex.cpp)
set(SRC_OTHER evaluationresult.cpp viewsynthesis.cpp)
set(SRC_ALL ${SRC_PARTIAL_MODEL} ${SRC_VIEW} ${SRC_OTHER} ${SRC_STRUCTURES})


# ----------------------------- Octomap --------------------
#set(OCTOMAP_DIR ${VPL_SOURCE_DIR}/../octomap-devel) ## Needs configuration
include_directories(${OCTOMAP_INCLUDE_PATH})
link_directories(${OCTOMAP_LIB_PATH})
set(LIBS_OCTOMAP octomap octomath)


#----------------------------- BOOST --------------------
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories( ${Boost_INCLUDE_DIR} )


# -------------------------------------------------
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${VPL_SOURCE_DIR}/lib)
add_library(PartialModel SHARED ${SRC_ALL})
target_link_libraries(PartialModel ${LIBS_OCTOMAP})
