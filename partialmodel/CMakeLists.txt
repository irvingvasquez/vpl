cmake_minimum_required(VERSION 2.6)

project(nbvpartialmodel)

set(SRC_STRUCTURES coctreevpl.cpp)
set(SRC_PARTIAL_MODEL partialmodelbase.cpp pmvolumetric.cpp pmvoctree.cpp pmvoctreeig.cpp pmvoctreevasquez09.cpp)  
# set(SRC_HRT pmraynode.cpp pmraytree.cpp pmvoctreehierarchicalrt.cpp)
set(SRC_VIEW viewstructure.cpp vpfilereader.cpp pmutils.cpp vptriangle.cpp vpvertex.cpp)
set(SRC_OTHER evaluationresult.cpp evaluationresultvector.cpp volumetricutilityfunction.cpp vufobjectfilters.cpp)
set(SRC_ALL ${SRC_PARTIAL_MODEL} ${SRC_VIEW} ${SRC_OTHER} ${SRC_STRUCTURES})
#set(SRC_ALL ${SRC_HRT} )


# ----------------------------- Octomap --------------------
set(OCTOMAP_DIR ${VPL_SOURCE_DIR}/../octomap-devel) ## Needs configuration

include_directories(${OCTOMAP_DIR}/octomap/include)
link_directories(${OCTOMAP_DIR}/lib ${OCTOMAP_DIR}/octovis/lib)
set(LIBS_OCTOMAP octomap octomath octovis)


# ---------------------------- INIPARSER --------------------
set(INIPARSER_DIR ${VPL_SOURCE_DIR}/../iniparser) ## Needs configuration

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
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${VPL_SOURCE_DIR}/lib)
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

#add_executable(test_partialmodel test_partialmodel.cpp ${SRC_ALL})
#target_link_libraries(test_partialmodel ${LIBS_OCTOMAP}  ${LIBS_INIPARSER})

#add_executable(paint_octree paint_octree.cpp ${SRC_ALL})
#target_link_libraries(paint_octree ${LIBS_OCTOMAP}  ${LIBS_INIPARSER})

#add_executable(raw2triangles raw2triangles.cpp ${SRC_ALL})
#target_link_libraries(raw2triangles ${LIBS_OCTOMAP}  ${LIBS_INIPARSER})