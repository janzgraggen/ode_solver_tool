cmake_minimum_required(VERSION 3.5)
project(PCSC-project)

# -------------------------------------------------------------------------------
# Global Settings
# -------------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build libraries as static" FORCE)

# -------------------------------------------------------------------------------
# Helper Functions
# -------------------------------------------------------------------------------

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

# -------------------------------------------------------------------------------
# Build Configuration Overrides (solve issue with muparser)
# -------------------------------------------------------------------------------

set(ENABLE_OPENMP OFF CACHE BOOL "Override muparser submodule build option ENABLE_OPENMP to OFF" FORCE)

# -------------------------------------------------------------------------------
# Submodule Management
# -------------------------------------------------------------------------------

# Special care with googletest, where we want to point to a more recent commit than the default one

set(GIT_EXECUTABLE "git")
option(GIT_SUBMODULE "Check submodules during build" ON)

if(GIT_SUBMODULE)
    message(STATUS "Submodule update")

    # Initialize and update submodules
    execute_process(
            COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE GIT_SUBMOD_RESULT
    )

    # Check if the submodule update was successful
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()

    # Automatically checkout the desired version of GoogleTest submodule
    set(GOOGLETEST_SUBMODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libs/googletest")
    set(GOOGLETEST_DESIRED_VERSION "79219e26e0e36b415a5804b6b017ad6c6cd99ad8")

    execute_process(
            COMMAND ${GIT_EXECUTABLE} -C ${GOOGLETEST_SUBMODULE_PATH} checkout ${GOOGLETEST_DESIRED_VERSION}
            RESULT_VARIABLE GIT_CHECKOUT_RESULT
    )

    if(NOT GIT_CHECKOUT_RESULT EQUAL "0")
        message(FATAL_ERROR "Failed to checkout GoogleTest version ${GOOGLETEST_DESIRED_VERSION}. Please ensure the version exists.")
    endif()

    message(STATUS "GoogleTest submodule updated to version ${GOOGLETEST_DESIRED_VERSION}.")
endif()

add_optional_library(GOOGLETEST libs/googletest)
add_optional_library(EIGEN libs/eigen)
add_optional_library(YAML libs/yaml)
add_optional_library(MUPARSER libs/muparser)

# -------------------------------------------------------------------------------
# Core Sources
# -------------------------------------------------------------------------------

file(GLOB_RECURSE CORE_SOURCES
        "src/**/*.cc"
        "src/**/*.hh"
)

# -------------------------------------------------------------------------------
# Main Executable
# -------------------------------------------------------------------------------

add_executable(main src/main.cc ${CORE_SOURCES})
target_link_libraries(main eigen yaml-cpp::yaml-cpp muparser)

# -------------------------------------------------------------------------------
# Test Executable
# -------------------------------------------------------------------------------

add_executable(test test/test.cc ${CORE_SOURCES})
target_link_libraries(test gtest_main gtest pthread eigen yaml-cpp muparser)

# -------------------------------------------------------------------------------
# Documentation Generation
# -------------------------------------------------------------------------------

option(DOCUMENTATION "Activate documentation" ON)

if (DOCUMENTATION)
    # Try to find Doxygen executable
    find_package(Doxygen)

    if (DOXYGEN_FOUND)
        message(STATUS "Doxygen found: ${DOXYGEN_EXECUTABLE}")

        # Define the Doxyfile path
        set(DOXYFILE "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile")

        # Define a custom target to generate documentation
        add_custom_target(doc_doxygen ALL
                COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE}
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM
        )
    else()
        message(WARNING "Doxygen was not found on your system. Documentation will not be generated.")
    endif()
endif()