# Builds a small static library containing our shared math code
# (Pose2D, differential-drive kinematics, and more later).
add_library(openrover_math STATIC
    pose2d.cpp
    differential_drive_kinematics.cpp
)

# PUBLIC means: anything that links against openrover_math can also
# #include headers from this directory (e.g. #include "pose2d.hpp").
target_include_directories(openrover_math PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
