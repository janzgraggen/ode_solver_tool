cmake_minimum_required(VERSION 3.0)
project(PCSC-project)

set(CMAKE_CXX_STANDARD 17)


# Function to handle library options and subdirectory addition
function(add_optional_library LIB_NAME LIB_DIR)
    option(${LIB_NAME} "activate ${LIB_NAME} support" ON)
    message("${LIB_NAME} option is ${${LIB_NAME}}")
    if(${LIB_NAME})
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${LIB_DIR}")
            message("Activating ${LIB_NAME} support")
            add_subdirectory(${LIB_DIR})
        else()
            message(FATAL_ERROR "Cannot find the ${LIB_DIR} directory")
        endif()
    endif()
endfunction()

## Override ENABLE_OPENMP build option of muparser.(?: muparser is missing things to build with OpenMP)
set(ENABLE_OPENMP OFF CACHE BOOL "Override muparser submodule build option ENABLE_OPENMP to OFF" FORCE)

# Update submodules as needed
set(GIT_EXECUTABLE "git")
option(GIT_SUBMODULE "Check submodules during build" ON)
if(GIT_SUBMODULE)
    message(STATUS "Submodule update")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE GIT_SUBMOD_RESULT
    )
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
endif()

# Check directories of submodules
add_optional_library(GOOGLETEST googletest)
add_optional_library(EIGEN eigen)
add_optional_library(YAML yaml)
add_optional_library(MUPARSER muparser)


# Collect core sources
file(GLOB_RECURSE CORE_SOURCES
    "src/**/*.cc"
    "src/**/*.hh"
)

# Add main executable
add_executable(main src/main.cc ${CORE_SOURCES})
target_link_libraries(main eigen yaml-cpp::yaml-cpp muparser)

# Add test executable
add_executable(test test/test.cc ${CORE_SOURCES})
target_link_libraries(test gtest_main gtest pthread eigen yaml-cpp muparser)

# Custom target to run tests
add_custom_target(run_tests COMMAND ./test DEPENDS test)