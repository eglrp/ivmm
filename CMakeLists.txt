cmake_minimum_required(VERSION 2.6)
project (ivmm)


find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python unit_test_framework program_options filesystem system)

set(IVMM_SRC 
    util.cpp
    road.cpp
    network.cpp
    ivmm.cpp
    sample_generator.cpp
    evaluation.cpp
)

add_compile_options(-std=c++11 -fPIC)

include_directories(
    ${PYTHON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    /usr/include/jsoncpp
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
)

add_executable(modify
    modify.cxx
)

target_link_libraries(modify
    ivmm_static
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
    libshp.a
    jsoncpp
)
set_target_properties(ivmm_static PROPERTIES OUTPUT_NAME "ivmm")

add_executable(main
    main.cxx
)
target_link_libraries(main
    ivmm_static
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)
#add_library(ivmm_shared SHARED
#    ${IVMM_SRC}
#)
#target_link_libraries(ivmm_shared
#    libshp.a
#)
#set_target_properties(ivmm_shared PROPERTIES OUTPUT_NAME "ivmm")
