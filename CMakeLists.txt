cmake_minimum_required(VERSION 3.26)
project(DynamicRouting VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_ROUTER "Build the SUMO/MPI routing executable" ON)
include(CTest)
if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME regression
        COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tests -v)
    set_tests_properties(regression PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1;CXX=${CMAKE_CXX_COMPILER}"
        TIMEOUT 120)
endif()

if(NOT BUILD_ROUTER)
    return()
endif()

set(SUMO_HOME "$ENV{SUMO_HOME}" CACHE PATH "SUMO source tree or installation prefix")
set(SUMO_LIBRARY_DIR "" CACHE PATH "Directory containing libtracicpp")
set(XGBOOST_ROOT "$ENV{XGBOOST_ROOT}" CACHE PATH "XGBoost source tree or installation prefix")
set(XGBOOST_LIBRARY_DIR "" CACHE PATH "Directory containing libxgboost")
find_package(MPI REQUIRED COMPONENTS CXX)
find_path(SUMO_INCLUDE_DIR libsumo/libtraci.h HINTS "${SUMO_HOME}/src" "${SUMO_HOME}/include" REQUIRED)
find_library(TRACICPP_LIBRARY NAMES tracicpp libtracicpp
    HINTS "${SUMO_LIBRARY_DIR}" "${SUMO_HOME}/bin" "${SUMO_HOME}/lib" REQUIRED)
find_path(XGBOOST_INCLUDE_DIR xgboost/c_api.h HINTS "${XGBOOST_ROOT}/include" REQUIRED)
find_library(XGBOOST_LIBRARY NAMES xgboost
    HINTS "${XGBOOST_LIBRARY_DIR}" "${XGBOOST_ROOT}/lib" REQUIRED)

add_executable(MASHierarchicalSystem main.cpp CoverEdge.cpp CoverGraph.cpp Router.cpp
    TrafficEstimator.cpp SpeedEstimator.cpp FuelEstimator.cpp)
target_include_directories(MASHierarchicalSystem PRIVATE "${SUMO_INCLUDE_DIR}" "${XGBOOST_INCLUDE_DIR}")
target_link_libraries(MASHierarchicalSystem PRIVATE MPI::MPI_CXX "${TRACICPP_LIBRARY}" "${XGBOOST_LIBRARY}")
if(UNIX)
    target_compile_definitions(MASHierarchicalSystem PRIVATE UNIX)
endif()

if(BUILD_TESTING)
    add_executable(model_loading tests/model_loading.cpp)
    target_include_directories(model_loading PRIVATE "${XGBOOST_INCLUDE_DIR}")
    target_link_libraries(model_loading PRIVATE "${XGBOOST_LIBRARY}")
    add_test(NAME model_loading COMMAND model_loading "${CMAKE_CURRENT_SOURCE_DIR}/scenarios")
    add_test(NAME cli_help COMMAND MASHierarchicalSystem --help)
    add_test(NAME cli_missing_arguments COMMAND MASHierarchicalSystem)
    set_tests_properties(cli_missing_arguments PROPERTIES WILL_FAIL TRUE)
    option(ENABLE_SUMO_SMOKE_TEST "Run a short xgrid simulation with two MPI processes" OFF)
    if(ENABLE_SUMO_SMOKE_TEST)
        find_program(SUMO_EXECUTABLE sumo HINTS "${SUMO_HOME}/bin" REQUIRED)
        add_test(NAME xgrid_smoke COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_example.py"
            --binary $<TARGET_FILE:MASHierarchicalSystem>
            --sumo "${SUMO_EXECUTABLE}" --mpiexec "${MPIEXEC_EXECUTABLE}"
            --steps 30 --timeout 150 --output "${CMAKE_CURRENT_BINARY_DIR}/smoke-output")
        set_tests_properties(xgrid_smoke PROPERTIES TIMEOUT 180)
        add_test(NAME xgrid_cooperative_smoke COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_example.py"
            --binary $<TARGET_FILE:MASHierarchicalSystem>
            --sumo "${SUMO_EXECUTABLE}" --mpiexec "${MPIEXEC_EXECUTABLE}"
            --agents 2 --steps 30 --timeout 150 --output "${CMAKE_CURRENT_BINARY_DIR}/smoke-output")
        set_tests_properties(xgrid_cooperative_smoke PROPERTIES TIMEOUT 180)
    endif()
endif()
