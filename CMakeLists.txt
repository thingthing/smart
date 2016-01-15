CMAKE_MINIMUM_REQUIRED(VERSION 2.1 FATAL_ERROR)

SET(PROJECT_NAME agent)

PROJECT(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-Wall -Wextra -pthread -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")



add_custom_target(clean-cmake-files
  COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
  )

add_custom_target(fclean
  COMMAND ${CMAKE_BUILD_TOOL} clean
  COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
  )

add_custom_target(re
  COMMAND ${CMAKE_BUILD_TOOL} fclean
  COMMAND ${CMAKE_BUILD_TOOL} all
  )


SET(SOURCES)

SET(SUBDIR
  core
  network
  protocol
  slam
  utils
  movement
  )

#Change the path to your implements files
SET (IMPLEMENT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/example/include")
SET (IMPLEMENT_SRC_DIR "${PROJECT_SOURCE_DIR}/example/src")
SET (IMPLEMENT_SUBDIR
  core
  network
  )

SET(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
SET(PCL_VERSION "1.7")


FIND_PACKAGE(PCL ${PCL_VERSION} REQUIRED)
INCLUDE_DIRECTORIES("${PCL_INCLUDE_DIRS}/pcl-${PCL_VERSION}")
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})


include(LibFindMacros)
set(Jsoncpp_PROCESS_INCLUDES Jsoncpp_INCLUDE_DIR)
set(Jsoncpp_PROCESS_LIBS Jsoncpp_LIBRARY)
FIND_PACKAGE(JsonCpp)
INCLUDE_DIRECTORIES(${Jsoncpp_INCLUDE_DIR})
LINK_DIRECTORIES(${Jsoncpp_LIBRARY})

ADD_SUBDIRECTORY("${PROJECT_SOURCE_DIR}/src")
ADD_SUBDIRECTORY(${INCLUDE_DIR})

ADD_SUBDIRECTORY(${IMPLEMENT_INCLUDE_DIR})
ADD_SUBDIRECTORY(${IMPLEMENT_SRC_DIR})

INCLUDE_DIRECTORIES(${INCLUDE_DIR} ${IMPLEMENT_INCLUDE_DIR})

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Jsoncpp_LIBRARIES})

# -- looking for PCL_COMMON
# -- looking for PCL_KDTREE
# -- looking for PCL_OCTREE
# -- looking for PCL_SEARCH
# -- looking for PCL_SURFACE
# -- looking for PCL_SAMPLE_CONSENSUS
# -- looking for PCL_IO
# -- looking for PCL_FILTERS
# -- looking for PCL_FEATURES
# -- looking for PCL_GEOMETRY
# -- looking for PCL_KEYPOINTS
# -- looking for PCL_REGISTRATION
# -- looking for PCL_SEGMENTATION
# -- looking for PCL_RECOGNITION
# -- looking for PCL_VISUALIZATION
# -- looking for PCL_PEOPLE
# -- looking for PCL_OUTOFCORE
# -- looking for PCL_TRACKING
# -- looking for PCL_APPS
