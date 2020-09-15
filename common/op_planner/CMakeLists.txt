cmake_minimum_required(VERSION 2.8.3)

project(op_planner)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
		cmake_modules
		geometry_msgs
		roscpp
		op_utility
		lanelet2_extension
)

if(EXISTS "/usr/include/proj_api.h")
  include(FindProj.cmake)
endif()  
find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  op_planner
  CATKIN_DEPENDS op_utility
  DEPENDS TinyXML
)
 
 ## set(CMAKE_CXX_FLAGS " -g -Wall -Wno-unused-result ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${TinyXML_INCLUDE_DIRS}
	${PROJ_INCLUDE_DIRS}
)


set(PLANNERH_SRC
	src/BehaviorPrediction.cpp 
	src/BehaviorStateMachine.cpp
	src/DecisionMaker.cpp
	src/LocalPlannerH.cpp
	src/MappingHelpers.cpp
	src/MatrixOperations.cpp
	src/PassiveDecisionMaker.cpp
	src/PlannerH.cpp		
	src/PlanningHelpers.cpp				
	src/SimuDecisionMaker.cpp
	src/TrajectoryCosts.cpp
	src/TrajectoryDynamicCosts.cpp
	src/TrajectoryEvaluator.cpp
	src/KmlMapLoader.cpp
	src/VectorMapLoader.cpp
	src/Lanelet2MapLoader.cpp
	src/RoadNetwork.cpp
	src/hmi/HMIMSG.cpp
	src/control/op_controller.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME}
	${PLANNERH_SRC} 	
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${PROJ_LIBRARY}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS op_planner
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#if (CATKIN_ENABLE_TESTING)
	#
	#roslint_add_test()
#find_package(rostest REQUIRED)
#
#add_rostest_gtest(test-op_planner_hmi test/test_op_planner_hmi.test test/src/test_op_planner_hmi.cpp )
#add_dependencies(test-op_planner_hmi ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test-op_planner_hmi ${catkin_LIBRARIES} ${PROJECT_NAME})
#
#endif ()
