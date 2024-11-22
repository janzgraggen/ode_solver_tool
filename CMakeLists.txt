
cmake_minimum_required (VERSION 3.0)
project (PCSC-project)

set(CMAKE_CXX_STANDARD 17)

add_library(pcsc
  src/pcsc.cc
)

# Check directories of submodules
option(GOOGLETEST "activate googletest  support" ON)
message("googletest  option is ${GOOGLETEST}")
if(GOOGLETEST)
  if(EXISTS "googletest ")
    message("activating googletest support")
    add_subdirectory(googletest)
  else()
    message(FATAL_ERROR "Cannot find the googletest directory" )
  endif()
endif()

option(EIGEN "activate eigen support" ON)
message("eigen option is ${EIGEN}")
if(EIGEN)
  if(EXISTS "eigen")
    message("activating eigen support")
    add_subdirectory(eigen)
  else()
    message(FATAL_ERROR "Cannot find the eigen directory" )
  endif()
endif()

# Update submodules as needed
set(GIT_EXECUTABLE "git")
option(GIT_SUBMODULE "Check submodules during build" ON)
if(GIT_SUBMODULE)
  message(STATUS "Submodule update")
  execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --
    recursive WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE GIT_SUBMOD_RESULT)
  if(NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "git submodule update --init --recursive failed 
      with ${GIT_SUBMOD_RESULT}, please checkout submodules")
  endif()
endif()


add_executable(main src/main.cc)
target_link_libraries(main pcsc)

add_executable(test_pcsc test/test.cc)
target_link_libraries(test_pcsc gtest_main gtest pthread pcsc)

add_custom_target(test ./test_pcsc DEPENDS test_pcsc)
