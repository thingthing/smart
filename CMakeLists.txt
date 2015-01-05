CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)

PROJECT(smartapi)

SET(SOURCES)

FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS common io)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

ADD_SUBDIRECTORY("${PROJECT_SOURCE_DIR}/src")

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/Include)

ADD_EXECUTABLE(smartapi ${SOURCES})

TARGET_LINK_LIBRARIES(smartapi ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})