cmake_minimum_required(VERSION 3.13.0)
project(planning_core CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native") # have to enable because of gtsam

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(src/common)
add_subdirectory(src/a_star)
add_subdirectory(src/trajectory_optimization)
add_subdirectory(src/map_manager)
add_subdirectory(src/ele_planner)

# Install pybind11 modules to the python package directory
install(TARGETS 
  a_star 
  traj_opt 
  ele_planner 
  py_map_manager
  a_star_search
  gpmp_optimizer
  map_manager
  ele_planner_lib
  common_smoothing
  DESTINATION ${PYTHON_INSTALL_DIR}/pct_planner/lib
)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/__init__.py DESTINATION ${PYTHON_INSTALL_DIR}/pct_planner/lib)

# Install python stubs
install(FILES
  src/a_star/a_star.pyi
  src/trajectory_optimization/traj_opt.pyi
  src/ele_planner/ele_planner.pyi
  src/map_manager/py_map_manager.pyi
  DESTINATION ${PYTHON_INSTALL_DIR}/pct_planner/lib
  OPTIONAL
)
