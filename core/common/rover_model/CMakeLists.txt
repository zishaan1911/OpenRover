# Builds a small static library containing RoverModel, which wraps
# our math functions with actual rover state.
add_library(openrover_rover_model STATIC
    rover_model.cpp
)

target_include_directories(openrover_rover_model PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# RoverModel is built on top of the math library (Pose2D,
# differential-drive kinematics), so it links against it. PUBLIC
# means anyone who links against openrover_rover_model automatically
# gets openrover_math too.
target_link_libraries(openrover_rover_model PUBLIC
    openrover_math
)