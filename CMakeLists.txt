cmake_minimum_required(VERSION 3.10)
project(tag_graph_nav)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tag_route_graph
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS JSONCPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
link_directories(${JSONCPP_LIBRARY_DIRS})

add_library(tag_route_graph
  src/tag_route_graph.cpp
)
target_link_libraries(tag_route_graph
  ${JSONCPP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(tag_navigator_node
  src/tag_navigator.cpp
  src/tag_navigator_node.cpp
)
target_link_libraries(tag_navigator_node
  tag_route_graph
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_program(PYTEST_EXECUTABLE NAMES pytest-3 pytest)
  if(NOT PYTEST_EXECUTABLE)
    message(FATAL_ERROR "pytest is required to test ${PROJECT_NAME}")
  endif()

  add_executable(tag_route_graph_probe EXCLUDE_FROM_ALL
    test/tag_route_graph_probe.cpp
  )
  target_link_libraries(tag_route_graph_probe
    tag_route_graph
    ${catkin_LIBRARIES}
  )
  add_dependencies(tests tag_route_graph_probe)

  set(PYTEST_RESULT
    "${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/pytest-test_tag_route_graph.xml")
  set(PYTEST_COMMAND
    "\"${CMAKE_COMMAND}\" -E env \"TAG_ROUTE_GRAPH_PROBE=$<TARGET_FILE:tag_route_graph_probe>\" \"${PYTEST_EXECUTABLE}\" -q \"${CMAKE_CURRENT_SOURCE_DIR}/test/test_tag_route_graph.py\" \"--junitxml=${PYTEST_RESULT}\"")
  catkin_run_tests_target(
    "pytest" "test_tag_route_graph" "pytest-test_tag_route_graph.xml"
    COMMAND ${PYTEST_COMMAND}
    DEPENDENCIES tag_route_graph_probe
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

install(TARGETS tag_route_graph tag_navigator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
