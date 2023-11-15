cmake_minimum_required(VERSION 3.5)
project(garden)

###########
## Build ##
###########
set(CMAKE_SKIP_BUILD_RPATH  TRUE)

set(RULE_MAKE_DIR  ${CMAKE_SOURCE_DIR}/..)
set(SHARED_LIBRARY_DIR "${RULE_MAKE_DIR}/rulebase/lib")
set(RULE_ROOT_DIR "${RULE_MAKE_DIR}/rulebase")
set(RULE_SOURCE_DIR "${RULE_MAKE_DIR}/rulebase/src")
set(RULE_INCLUDE_DIR "${RULE_MAKE_DIR}/rulebase/include")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${SHARED_LIBRARY_DIR})

include_directories(${RULE_INCLUDE_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# List all rule directories.
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/rule_*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

# Add all rule folders.
SUBDIRLIST(SUBDIRS ${RULE_ROOT_DIR})
FOREACH(subdir ${SUBDIRS})
  add_subdirectory(${subdir})
ENDFOREACH()



