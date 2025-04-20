cmake_minimum_required(VERSION 3.5)
project(vedh_gazebo)
list(APPEND CMAKE_MESSAGE_IGNORE "Manually-specified variables were not used by the project")

find_package(ament_cmake REQUIRED)  # Only this line is needed

install(
  DIRECTORY launch worlds urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  astar_path_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

