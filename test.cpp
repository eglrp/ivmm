#define BOOST_TEST_MODULE ivmm_test

#include  <boost/test/unit_test.hpp>
#include  <boost/timer.hpp>
#include  <vector>
#include  <iostream>
#include  <cstdio>
#include    "road.h"
#include    "range_extend.hpp"
#include    "network.h"
#include    "key_visitor.hpp"
#include    "sample_generator.h"
#include    "ivmm.h"
#include    "key_visitor.hpp"
#include    "evaluation.h"
#include    "format.h"
#include    "io.h"
#include    "pathendpointwalker.hpp"

void b(){}
using namespace std;
BOOST_AUTO_TEST_SUITE(basic_classes)

BOOST_AUTO_TEST_CASE( io_test ){
    b();
    Network network;
    BOOST_REQUIRE( network.load("../Date/map/bj-road-epsg3785") );
    RawTraj traj = loadTraj("../Date/traj-1-900/20121113/300694/300694-4.txt");
    BOOST_REQUIRE( ! traj.empty() );
    Path path = trajToPath(network, traj);

    boost::timer timer;
    auto p = network.shortest_path(73184, 73050);
    cout << timer.elapsed() << "s" << endl;
    cout << p.length << endl;
}

BOOST_AUTO_TEST_SUITE_END()

#include<boost/test/included/unit_test.hpp>
