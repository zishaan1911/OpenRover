# --------------------------------------------------------------------
# Downloads GoogleTest automatically the first time you build, so you
# don't need to install it system-wide.
# --------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)

# Keeps GoogleTest's runtime library setting consistent with the rest
# of the project (mainly relevant on Windows).
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

add_subdirectory(unit)
