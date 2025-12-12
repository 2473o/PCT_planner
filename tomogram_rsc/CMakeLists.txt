cmake_minimum_required(VERSION 3.16)
project(tomogram_rsc)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

install(DIRECTORY pcd rviz tomogram
  DESTINATION share/${PROJECT_NAME}
)

# Register a resource in ament index so other packages can discover pct_rsc
# ament_index_register_resource("packages" CONTENT "${PROJECT_NAME}")

ament_package()
