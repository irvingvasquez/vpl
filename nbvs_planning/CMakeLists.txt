cmake_minimum_required(VERSION 2.6)

project(nbvs_planning)

########### First configure the following ###########
set(OCTREE_RANGE_SIMULATOR TRUE)
set(RELATED_DIRS_DIRECTORY ${VPL_SOURCE_DIR}/..)

set(SRC_EXT_ROBOTS robotmobile.cpp robotarmmdh.cpp robotcomposed.cpp)
#set(SRC_ROBOTS vprobot.cpp robotmobile.cpp robotarmmdh.cpp vprfreeflyer.cpp robotcomposed.cpp)
#set(REALROBOT_SRC robotpioneer.cpp robotkatana.cpp)

#set(SRC_EXT_SENSORS rssimulated.cpp)
#set(SRC_SENSORS rangesensor.cpp rskinect.cpp rssimulated.cpp)


#set(SRC_EXT_ROBOTSENSOR robotsensor.cpp)
#set(SRC_ROBOTSENSOR robotsensor.cpp robsennotransformation.cpp)

set(SRC_EXT_NBVPLANNER rrtnbv.cpp nbvplannerrgflt.cpp nbvplannerexpectedutility.cpp nbvplannerexpectedutilitynroot.cpp)
#set(SRC_NBVPLANNER nbvplannerrgflt.cpp nbvplanner.cpp  workspacenbvplanner.cpp  rrtnbv.cpp nbvplannerexpectedutility.cpp nbvplannerexpectedutilitynroot.cpp)


#set(SRC_RECONSTRUCTOR reconstructor3d.cpp r3ddirectpositioning.cpp vplutils.cpp)
#set(SRC_EXT_RECONSTRUCTOR reconstructor3d.cpp r3ddirectpositioning.cpp vplutils.cpp)

set(SRC ${SRC_EXT_ROBOTS} ${SRC_EXT_SENSORS} ${SRC_EXT_NBVPLANNER} ${SRC_EXT_ROBOTSENSOR} ${SRC_EXT_RECONSTRUCTOR})


# ---------------- INIPARSER  -----------------
include_directories(${RELATED_DIRS_DIRECTORY}/iniparser/src)
link_directories(${RELATED_DIRS_DIRECTORY}/iniparser)
set(LIBS_INIPARSER iniparser)


 #---------------- View Planning Library ---------
include_directories(${VPL_SOURCE_DIR}/viewplanning)
include_directories(${VPL_SOURCE_DIR}/partialmodel)
link_directories(${VPL_SOURCE_DIR}/lib)
set(LIBS_VIEW_PLANNING ViewPlanning PartialModel)


# --------------- Octomap --------------------
include_directories(${RELATED_DIRS_DIRECTORY}/octomap-devel/octomap/include)
link_directories(${RELATED_DIRS_DIRECTORY}/octomap-devel/lib)
set(LIBS_OCTOMAP octomap octomath octovis)


# ---------------- MRPT ---------------------
 find_package(MRPT REQUIRED)
 include_directories(${MRPT_INCLUDE_DIRS})
 link_directories(${MRPT_LINK_DIRECTORIES})
# include_directories(/usr/include/mrpt/gui/include)
 set(LIBS_MRPT mrpt-base)
 
set(CORE_LIBS ${LIBS_INIPARSER} ${LIBS_VIEW_PLANNING} ${LIBS_OCTOMAP} ${LIBS_MRPT})
 
 
if(OCTREE_RANGE_SIMULATOR)
#---------------- range Simulator ---------
  message("APPS will compile with octree range simulator")
  include_directories(${VPL_SOURCE_DIR}/rangesimulator)
  link_directories(${VPL_SOURCE_DIR}/lib)
  set(LIBS_RANGE_SIMULATOR RangeSimulator)
endif()


#------------------ MotionPlanning---------------
include_directories(${RELATED_DIRS_DIRECTORY}/nbvmotionplanner)
link_directories(${RELATED_DIRS_DIRECTORY}/nbvmotionplanner/lib)
set(LIBS_MOTION_PLANNING MotionPlanning)


# #########################################################
# # PQP
# #########################################################
include_directories(${RELATED_DIRS_DIRECTORY}/pqp-1.3/PQP_v1.3/include)
link_directories(${RELATED_DIRS_DIRECTORY}/pqp-1.3/PQP_v1.3/lib)
set(PQP_LIBRARY PQP)


# # #########################################################
# # # FOX LIBRARY
# # #########################################################
include_directories(/usr/include/fox-1.6)
link_directories(/usr/lib)
set(FOX_LIBRARY FOX-1.6)

 
# #########################################################
# # FIND GLUT
# #########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
     message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)


# #########################################################
# # FIND OPENGL
# #########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)


set (LIBS_MP ${LIBS_MOTION_PLANNING} ${PQP_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${FOX_LIBRARY})

set(LIBS_ALL ${CORE_LIBS} ${LIBS_RANGE_SIMULATOR} ${LIBS_MP})

add_executable(vp_simulation_example vp_simulation_example.cpp ${SRC})
target_link_libraries(vp_simulation_example ${LIBS_ALL})

add_executable(test_sim_rrtnbv test_sim_rrtnbv.cpp ${SRC})
target_link_libraries(test_sim_rrtnbv ${LIBS_ALL})

add_executable(test_sim_rrtnbv_fig test_sim_rrtnbv_fig.cpp ${SRC})
target_link_libraries(test_sim_rrtnbv_fig ${LIBS_ALL})

add_executable(test_sim_rrtnbv_Kriegel test_sim_rrtnbv_Kriegel.cpp ${SRC})
target_link_libraries(test_sim_rrtnbv_Kriegel ${LIBS_ALL})

add_executable(test_sim_rrtnbv_rearside test_sim_rrtnbv_rearside.cpp ${SRC})
target_link_libraries(test_sim_rrtnbv_rearside ${LIBS_ALL})

#if(FREE_FLYER_APP)
#  if(OCTREE_RANGE_SIMULATOR)
    #add_executable(vp_simulation_example vp_simulation_example.cpp ${SRC_ALL} ${SRC_OCTREE_RSIM})
    #target_link_libraries(vp_simulation_example ${ALL_LIBS} ${LIBS_RANGE_SIMULATOR})
  
    #add_executable(test_freeflyer test_freeflyer.cpp ${SRC_EXTENDED} ${SRC_OCTREE_RSIM})
    #target_link_libraries(test_freeflyer ${CORE_LIBS} ${LIBS_RANGE_SIMULATOR})
    
    #add_executable(test_freeflyer_vasquez09 test_freeflyer_vasquez09.cpp ${SRC_EXTENDED} ${SRC_OCTREE_RSIM})
    #target_link_libraries(test_freeflyer_vasquez09 ${CORE_LIBS} ${LIBS_RANGE_SIMULATOR})
#  endif()
#endif()
 
 

#set (ALL_LIBS ${LIBS_INIPARSER} ${LIBS_PARTIAL_MODEL} ${LIBS_OCTOMAP} ${LIBS_MRPT} ${LIBS_MP})



######################################################### 


