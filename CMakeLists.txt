cmake_minimum_required(VERSION 2.6)
project (ivmm)


find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python unit_test_framework program_options filesystem system)

add_compile_options(-std=c++11 -fPIC)


add_subdirectory(cppformat)
include_directories(cppformat)

set(IVMM_SRC 
    util.cpp
    road.cpp
    network.cpp
    ivmm.cpp
    sample_generator.cpp
    evaluation.cpp
)


include_directories(
    ${PYTHON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    /home/syh/Developer/ThirdLibrary/util
)

link_directories(
    ${Boost_LIBRARY_DIRS}
)

add_executable (test
    test.cpp
)

target_link_libraries(test
    ivmm_static
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    format
)

add_library(pyivmm SHARED
    pyivmm.cpp
)

target_link_libraries( pyivmm
    ivmm_static
   ${Boost_PYTHON_LIBRARY}
   ${Boost_SYSTEM_LIBRARY}
   ${PYTHON_LIBRARIES}
)

add_library(ivmm_static
    ${IVMM_SRC}
)
target_link_libraries(ivmm_static
    shp
    ${Boost_SYSTEM_LIBRARY}
)
set_target_properties(ivmm_static PROPERTIES OUTPUT_NAME "ivmm")

add_executable(ivmm_main
    ivmm_main.cpp
)
target_link_libraries(ivmm_main
    ivmm_static
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    format
)

add_executable(smooth_traj
    smooth_traj.cpp
)

target_link_libraries(smooth_traj
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

add_executable( edge_time_collector
    edge_time_collector.cpp
)

target_link_libraries(edge_time_collector
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(gpsReformat
    gpsReformat.cpp
)

target_link_libraries(gpsReformat
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    proj
    format
)

add_executable(gpsPrepare
    gpsPrepare.cpp
)

target_link_libraries(gpsPrepare
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ivmm_static
    format
)

add_executable(gpsFileSplit
    gpsFileSplit.cpp
)
target_link_libraries(gpsFileSplit
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(stat
    statistics.cpp
)
target_link_libraries(stat
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

add_executable(candidateCrossEdgeGraphGenerator
    candidateCrossGraphGenerator.cpp
)
target_link_libraries(candidateCrossEdgeGraphGenerator
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ivmm_static
)

add_executable(candidateRoadGraphGenerator
    candidateRoadGraphGenerator.cpp
)
target_link_libraries(candidateRoadGraphGenerator
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ivmm_static
)

add_executable(drawCPNet
    drawcpn.cpp
    transitionGraph.cpp
)

target_link_libraries(drawCPNet
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ivmm_static
)

add_executable(dumpCross
    dumpCross.cpp
)

target_link_libraries(dumpCross
    ivmm_static
)
