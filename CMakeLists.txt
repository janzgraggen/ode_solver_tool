
cmake_minimum_required (VERSION 3.0)
project (PCSC-project)

set(CMAKE_CXX_STANDARD 17)



# Check directories of submodules
option(GOOGLETEST "activate googletest  support" ON)
message("googletest  option is ${GOOGLETEST}")
if(GOOGLETEST)
  if(EXISTS "../googletest") ## With respect to build directory
    message("activating googletest support")
    add_subdirectory(googletest)
  else()
    message(FATAL_ERROR "Cannot find the googletest directory" )
  endif()
endif()

option(EIGEN "activate eigen support" ON)
message("eigen option is ${EIGEN}")
if(EIGEN)
  if(EXISTS "../eigen") ## With respect to build directory
    message("activating eigen support")
    add_subdirectory(eigen)
  else()
    message(FATAL_ERROR "Cannot find the eigen directory" )
  endif()
endif()

option(YAML "activate yaml support" ON)
message("yaml option is ${YAML}")
if(YAML)
  if(EXISTS "../yaml") ## With respect to build directory
    message("activating yaml support")
    add_subdirectory(yaml)
  else()
    message(FATAL_ERROR "Cannot find the yaml directory" )
  endif()
endif()

# Update submodules as needed
set(GIT_EXECUTABLE "git")
option(GIT_SUBMODULE "Check submodules during build" ON)
if(GIT_SUBMODULE)
  message(STATUS "Submodule update")
  execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive 
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE GIT_SUBMOD_RESULT)
  if(NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "git submodule update --init --recursive failed 
      with ${GIT_SUBMOD_RESULT}, please checkout submodules")
  endif()
endif()


add_executable(main src/main.cc
        src/Reader/Reader.hh
        src/Reader/Reader.cc

        src/Ode/OdeSolver.hh
        src/Ode/OdeSolver.cc

        src/Ode/Explicit.hh
        src/Ode/Explicit.cc
        src/Ode/ForwardEuler.hh
        src/Ode/ForwardEuler.cc
        src/Ode/RungeKutta.hh
        src/Ode/RungeKutta.cc
        src/Ode/AdamsBashforth.hh
        src/Ode/AdamsBashforth.cc

        src/Ode/Implicit.hh
        src/Ode/Implicit.cc
        src/Ode/BackwardEuler.hh
        src/Ode/BackwardEuler.cc

        src/LinSysStruct.hh

        src/LinSysSolver/LinSysSolver.hh
        src/LinSysSolver/LinSysSolver.cc
        src/LinSysSolver/GaussElimSolve.hh
        src/LinSysSolver/GaussElimSolve.cc

        src/RootFinder/RootFinder.hh
        src/RootFinder/RootFinder.cc
        src/RootFinder/NewtonRaphson.hh
        src/RootFinder/NewtonRaphson.cc
        )

target_link_libraries(main eigen yaml-cpp::yaml-cpp)

add_executable(test_pcsc test/test.cc)
target_link_libraries(test_pcsc gtest_main gtest pthread)

add_custom_target(test ./test_pcsc DEPENDS test_pcsc)
