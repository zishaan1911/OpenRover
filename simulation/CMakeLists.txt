cmake_minimum_required(VERSION 3.8)
project(openrover_simulation)

find_package(ament_cmake REQUIRED)

# This package has no C++ or Python code of its own (yet) — it's just
# world files, robot models, and launch files. "install(DIRECTORY ...)"
# copies those folders into the installed package share directory, so
# other packages (and ROS 2 launch files) can find them by package
# name instead of by a hardcoded path.
install(DIRECTORY
  worlds
  launch
  models
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
