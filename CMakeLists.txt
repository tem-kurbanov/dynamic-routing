cmake_minimum_required(VERSION 3.26)
project(MASHierarchicalSystem)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message(STATUS "Configuring on/for a Unix-like system")
add_definitions(-DUNIX)

SET(TARGET "MASHierarchicalSystem")
PROJECT(${TARGET} LANGUAGES CXX)
add_executable(${TARGET} main.cpp
        CoverEdge.h
        CoverEdge.cpp
        CoverGraph.h
        CoverGraph.cpp
        Edge.h
        Node.h
        TrafficEstimator.h
        TrafficEstimator.cpp
        Graph.h
        Router.cpp
        Router.h
        serializations.h
        SpeedEstimator.h
        SpeedEstimator.cpp
        FuelEstimator.h
        FuelEstimator.cpp
)


find_package(OpenMP REQUIRED)


message("Building for Unix")
find_package(MPI REQUIRED)
set(SUMO_HOME $ENV{SUMO_HOME})

set(XGBOOST_ROOT "/home/tim/Work/own/phd/phdproject/xgboost")
find_library(XGBOOST_LIB xgboost PATHS ${XGBOOST_ROOT}/lib NO_DEFAULT_PATH)

include_directories(${Boost_INCLUDE_DIRS} ${SUMO_HOME}/src ${MPI_INCLUDE_PATH} ${XGBOOST_ROOT}/include)
set(CMAKE_C_FLAGS "${CMAKE_FLAGS} ${MPI_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MPI_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MPI_EXE_LINKER_FLAGS}")
link_directories(${SUMO_HOME}/bin)
target_link_libraries(${TARGET} PRIVATE MPI::MPI_CXX sumocpp tracicpp ${XGBOOST_ROOT}/lib/libxgboost.so OpenMP::OpenMP_CXX)
