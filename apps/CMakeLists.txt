cmake_minimum_required(VERSION 2.6)

project(apps)

# Octomap
include_directories(${OCTOMAP_INCLUDE_PATH})
link_directories(${OCTOMAP_LIB_PATH})
set(LIBS_OCTOMAP octomap octomath)


if(BUILD_PARTIAL_MODEL)
  # Partial model
  include_directories(${VPL_SOURCE_DIR}/partialmodel)
  link_directories(${VPL_SOURCE_DIR}/lib)
  set(LIB_PARTIAL_MODEL PartialModel)
  
  message("Building partial model examples")

  # Programs
  add_executable(random_view_generator test_view_generator.cpp ${SRC_ALL})
  target_link_libraries(random_view_generator ${LIB_PARTIAL_MODEL} ${LIBS_OCTOMAP})

  add_executable(view_sphere_generator test_view_sphere.cpp ${SRC_ALL})
  target_link_libraries(view_sphere_generator  ${LIB_PARTIAL_MODEL} ${LIBS_OCTOMAP})
  
  add_executable(update_partial_model test_update_partial_model.cpp ${SRC_ALL})
  target_link_libraries(update_partial_model  ${LIB_PARTIAL_MODEL} ${LIBS_OCTOMAP})
  
  
  
  if(BUILD_VIEW_PLANNING)
    include_directories(${VPL_SOURCE_DIR}/viewplanning)
    set(LIBS_VIEW_PLANNING ViewPlanning)
    
    # ---------------- MRPT ---------------------
    find_package(MRPT REQUIRED)
    include_directories(${MRPT_INCLUDE_DIRS})
    link_directories(${MRPT_LINK_DIRECTORIES})
    include_directories(/usr/include/mrpt/gui/include)
    set(LIBS_MRPT mrpt-base)
    
    set (CORE_LIBS ${LIBS_VIEW_PLANNING} ${LIBS_OCTOMAP} ${LIBS_MRPT})
    
    if(BUILD_RANGE_SIM)
      # range Simulator 
      message("Building range simulator apps")
      include_directories(${VPL_SOURCE_DIR}/rangesimulator)
      link_directories(${VPL_SOURCE_DIR}/lib)
      set(LIBS_RANGE_SIMULATOR RangeSimulator)
      
      add_executable(test_rangesimulator test_rangesimulator.cpp)
      target_link_libraries(test_rangesimulator ${LIB_PARTIAL_MODEL} ${LIBS_RANGE_SIMULATOR} ${LIBS_OCTOMAP} ${LIBS_VIEW_PLANNING})
      
      add_executable(test_freeflyer test_freeflyer.cpp)
      target_link_libraries(test_freeflyer ${CORE_LIBS} ${LIBS_RANGE_SIMULATOR})
      
      add_executable(test_freeflyer_vasquez09 test_freeflyer_vasquez09.cpp)
      target_link_libraries(test_freeflyer_vasquez09 ${CORE_LIBS} ${LIBS_RANGE_SIMULATOR})
      
    endif()
  endif()
endif()


if(BUILD_NBVS)

endif()



























#set(SRC_EXT_ROBOTS vprobot.cpp vprfreeflyer.cpp)
#set(SRC_ROBOTS vprobot.cpp robotmobile.cpp robotarmmdh.cpp vprfreeflyer.cpp robotcomposed.cpp)
#set(REALROBOT_SRC robotpioneer.cpp robotkatana.cpp)


#set(SRC_EXT_SENSORS)
#set(SRC_SENSORS rangesensor.cpp rskinect.cpp rssimulated.cpp)


#set(SRC_EXT_ROBOTSENSOR robotsensor.cpp)
#set(SRC_ROBOTSENSOR robotsensor.cpp robsennotransformation.cpp)

#set(SRC_EXT_NBVPLANNER nbvplanner.cpp)
#set(SRC_NBVPLANNER nbvplannerrgflt.cpp nbvplanner.cpp  workspacenbvplanner.cpp  rrtnbv.cpp nbvplannerexpectedutility.cpp nbvplannerexpectedutilitynroot.cpp)


#set(SRC_RECONSTRUCTOR reconstructor3d.cpp r3ddirectpositioning.cpp vplutils.cpp)
#set(SRC_EXT_RECONSTRUCTOR reconstructor3d.cpp r3ddirectpositioning.cpp vplutils.cpp)

#set(SRC_EXTENDED ${SRC_EXT_ROBOTS} ${SRC_EXT_SENSORS} ${SRC_EXT_NBVPLANNER} ${SRC_EXT_ROBOTSENSOR} ${SRC_EXT_RECONSTRUCTOR})



 
 
 #------------------ MotionPlanning---------------
# include_directories(${RELATED_DIRS_DIRECTORY}/nbvmotionplanner)
# link_directories(${RELATED_DIRS_DIRECTORY}/nbvmotionplanner/lib)
# set(LIBS_MOTION_PLANNING MotionPlanning)

 
# #########################################################
# # PQP
# #########################################################
# include_directories(${RELATED_DIRS_DIRECTORY}/pqp-1.3/PQP_v1.3/include)
# link_directories(${RELATED_DIRS_DIRECTORY}/pqp-1.3/PQP_v1.3/lib)
# set(PQP_LIBRARY PQP)


# # #########################################################
# # # FOX LIBRARY
# # #########################################################
# include_directories(/usr/include/fox-1.6)
# link_directories(/usr/lib)
# set(FOX_LIBRARY FOX-1.6)

 
# #########################################################
# # FIND GLUT
# #########################################################
# find_package(GLUT REQUIRED)
# include_directories(${GLUT_INCLUDE_DIRS})
# link_directories(${GLUT_LIBRARY_DIRS})
# add_definitions(${GLUT_DEFINITIONS})
# if(NOT GLUT_FOUND)
#     message(ERROR " GLUT not found!")
# endif(NOT GLUT_FOUND)


# #########################################################
# # FIND OPENGL
# #########################################################
# find_package(OpenGL REQUIRED)
# include_directories(${OpenGL_INCLUDE_DIRS})
# link_directories(${OpenGL_LIBRARY_DIRS})
# add_definitions(${OpenGL_DEFINITIONS})
# if(NOT OPENGL_FOUND)
#     message(ERROR " OPENGL not found!")
# endif(NOT OPENGL_FOUND)


#set (LIBS_MP ${LIBS_MOTION_PLANNING} ${PQP_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${FOX_LIBRARY})
#set (ALL_LIBS ${LIBS_INIPARSER} ${LIBS_PARTIAL_MODEL} ${LIBS_OCTOMAP} ${LIBS_MRPT} ${LIBS_MP})



######################################################### 


