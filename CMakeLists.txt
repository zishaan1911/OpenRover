cmake_minimum_required(VERSION 3.20)
project(OpenRover LANGUAGES CXX)

# We use C++20 features throughout the core robotics code.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This makes editor tooling (like VS Code's C++ extension) able to
# find headers and give you good autocomplete.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enables CTest, CMake's built-in test runner.
enable_testing()

# This is where the actual robotics algorithms live.
add_subdirectory(core)

# This is where our automated tests live.
add_subdirectory(tests)
