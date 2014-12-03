#define BOOST_TEST_MODULE ivmm_test

#include  <boost/test/unit_test.hpp>
#include  <boost/algorithm/string.hpp>
#include  <vector>
#include  <boost/assign.hpp>
#include  <boost/range/numeric.hpp>
#include  <boost/tuple/tuple.hpp>
#include  <boost/algorithm/string.hpp>
#include  <boost/range/algorithm.hpp>
#include  <iostream>
#include  <iomanip>
#include  <ctime>
#include  <type_traits>
#include  <shapefil.h>
#include  <cstdio>
#include  <fstream>
#include    "road.h"
#include    "range_extend.hpp"
#include    "network.h"
#include    "key_visitor.hpp"
#include    "sample_generator.h"
#include    "ivmm.h"
#include    "key_visitor.hpp"
#include    "evaluation.h"
#include    "debuger.hpp"
#include    "format.h"
using namespace std;
using namespace boost::adaptors;
void break_point(){}

vector<GpsPoint> loadGps(string const& filename){
    ifstream ins(filename);
    string line;
    vector<GpsPoint> gps;
    vector<string> splitStr;

    while ( getline( ins, line) ){
        boost::split(splitStr, line, boost::is_any_of(","));
        double x = atof( splitStr[2].c_str());
        double y = atof( splitStr[3].c_str());
        std::tm time;
        strptime(splitStr[1].c_str(), "%Y-%m-%d %H:%M:%S", &time);
        GpsPoint g(x, y, mktime(&time));
        gps.push_back(g);
    }

    return gps;
}

BOOST_AUTO_TEST_SUITE(basic_classes)

    Network network;
/*
    BOOST_AUTO_TEST_CASE(roadsegment_test){
        Cross begin(0, "", 121.24661, 31.02633);
        Cross end(1, "", 121.25261,31.02765);

        vector<Point> roadpoints = {begin,{121.24997,31.02772},end};

        double dist_sum = boost::accumulate(roadpoints | boost::adjacented, 0.0,
                [](double acc, boost::tuple<Point, Point>const & points){
                return acc + points.get<0>().gis_dist(points.get<1>());
                });


        RoadSegment roadseg(0, "", 80, true, begin, end, roadpoints);

        BOOST_CHECK_EQUAL(roadseg.begin, begin);
        BOOST_CHECK_EQUAL(roadseg.end, end);
        BOOST_CHECK_EQUAL(roadseg.points.front().where, 0.0);
        BOOST_CHECK_EQUAL(roadseg.points.back().where, 1.0);
        BOOST_CHECK_CLOSE(roadseg.points[1].where, roadpoints[1].gis_dist(roadpoints[0]) / dist_sum, 1e-6);

        BOOST_CHECK_EQUAL(roadseg.candidate_at_normal(0.0), roadseg.candidate_at_dist(0.0));
        BOOST_CHECK_EQUAL(roadseg.candidate_at_dist(0.0), begin);


        BOOST_CHECK_EQUAL(roadseg.candidate_at_normal(1.0), roadseg.candidate_at_dist(dist_sum));
        BOOST_CHECK_EQUAL(roadseg.candidate_at_dist(dist_sum), end);

        double dist = roadpoints[1].gis_dist(roadpoints[0]);
        BOOST_CHECK_EQUAL(roadseg.candidate_at_dist(dist), roadseg.candidate_at_normal(dist / dist_sum));
        BOOST_CHECK_EQUAL(roadseg.candidate_at_dist(dist), roadseg.points[1]);

        Point p = (roadseg.points[1] - roadseg.points[0]) * 0.5 + roadseg.points[0];
        BOOST_CHECK_EQUAL(roadseg.candidate_at_dist(dist/2.0), p);


        vector<CandidatePoint> path = roadseg.path_follow(0,1);
        BOOST_CHECK_EQUAL_COLLECTIONS(path.begin(), path.end(), roadseg.points.begin(), roadseg.points.end());

        path = roadseg.path_follow(1,0);
        auto rev = roadseg.points | boost::adaptors::reversed;
        BOOST_CHECK_EQUAL_COLLECTIONS(path.begin(), path.end(), rev.begin(), rev.end());

        path = roadseg.path_follow(0, dist / dist_sum);
        BOOST_CHECK_EQUAL_COLLECTIONS(path.begin(), path.end(), roadseg.points.begin(), roadseg.points.begin()+2);

        path = roadseg.path_follow(0,0);
        //BOOST_CHECK_EQUAL_COLLECTIONS(path.begin(), path.end(), roadseg.points.begin(), roadseg.points.begin()+1);
        BOOST_CHECK_EQUAL( path.size(), 0 );

        path = roadseg.path_follow(roadseg.points[1].where / 2, roadseg.points[1].where / 2 + 0.5);
        BOOST_CHECK_EQUAL(path.size(), 3);
        BOOST_CHECK_CLOSE(path[0].where , roadseg.points[1].where/2.0, 1e-6);
        BOOST_CHECK_EQUAL(path[1], roadseg.points[1]);
        BOOST_CHECK_CLOSE(path[2].where, roadseg.points[1].where/2.0 + 0.5, 1e-6);

        BOOST_CHECK_EQUAL(network.load("/home/syh/Desktop/南京地图修正/road"), true);

        BOOST_CHECK_EQUAL(network.road(0)->points[0].belong, network.road(0));
        BOOST_CHECK_EQUAL(network.road(0)->points.back().belong, network.road(0));
        BOOST_CHECK_EQUAL(network.road(0)->points[network.road(0)->points.size()/2].belong, network.road(0));
    }
*/
/*
    BOOST_AUTO_TEST_CASE(path_smooth_traj){

        BOOST_CHECK_EQUAL( network.load("/home/syh/Desktop/北京地图修正/road"), true);
        CandidateGraph graph;
        vector<size_t> best_index;
        IVMM ivmm(&network);
        IVMMParamEX param;
        param.candidate_limit = 5;
        param.candidate_query_radious = 100;
        param.project_dist_mean = 5;
        param.project_dist_stddev = 10;
        param.window = 50;
        param.beta = 5000;

        vector<GpsPoint> gps = loadGps("/home/syh/Desktop/final-part/8662(2008-02-05)-0-3.txt");
        Path path = ivmm.ivmm_ex(gps, param, &graph, &best_index);

        auto traj = path.smooth_trajectory();
        SHPHandle shp = SHPCreate("traj", SHPT_ARC);
        DBFHandle dbf = DBFCreate("traj");
        DBFAddField(dbf, "enter", FTString, 20, 0);
        DBFAddField(dbf, "leave", FTString, 20, 0);

        char str[21];
        for(auto& seg: traj){
            size_t nv = seg.road->points.size();
            double x[nv];
            double y[nv];
            boost::transform(seg.road->points, x, key_of(&Point::x));
            boost::transform(seg.road->points, y, key_of(&Point::y));
            SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, nv, x, y, nullptr);
            int newID = SHPWriteObject(shp, -1, obj);
            strftime(str, sizeof(str), "%Y-%m-%d %H:%M:%S", localtime(&seg.enter_timestamp));
            DBFWriteStringAttribute(dbf, newID, 0, str);
            strftime(str, sizeof(str), "%Y-%m-%d %H:%M:%S", localtime(&seg.leave_timestamp));
            DBFWriteStringAttribute(dbf, newID, 1, str);
            SHPDestroyObject(obj);
        }

        DBFClose(dbf);
        SHPClose(shp);


        vector<CandidatePoint> bestCandidates;
        for ( size_t i = 0; i < best_index.size(); ++i){
            bestCandidates.push_back( graph.candidates.at(i).at(best_index.at(i)));
        }

        shp = SHPCreate("bestCandidate", SHPT_POINT);
        dbf = DBFCreate("bestCandidate");
        DBFAddField(dbf, "vote", FTInteger, 10, 0);
        DBFAddField(dbf, "fvalue", FTDouble, 10, 6);
        DBFAddField(dbf, "timestamp", FTInteger, 20,0);
        DBFAddField(dbf, "data", FTString, 20, 0);
        for ( auto& c : bestCandidates){
            SHPObject * obj = SHPCreateSimpleObject(SHPT_POINT, 1, & c.x, & c.y, nullptr);
            int newID = SHPWriteObject(shp, -1, obj);
            DBFWriteIntegerAttribute( dbf, newID, 0, c.vote);
            DBFWriteDoubleAttribute( dbf, newID, 1, c.fvalue);
            DBFWriteIntegerAttribute( dbf, newID, 2, c.timestamp);
            tm time;
            strftime( str, sizeof str, "%Y-%m-%d %H:%M:%S", localtime( & c.timestamp));
            DBFWriteStringAttribute( dbf, newID, 3, str);
            SHPDestroyObject(obj);
        }
        SHPClose(shp);
        DBFClose(dbf);


        vector< boost::tuple<PathPoint, size_t , size_t, size_t > > cross_pathpoint;
        for ( size_t i = 0; i < path.points.size(); ++i ) {
            if ( path.points.at(i).cid != -1 ) {
                cross_pathpoint.push_back( {path.points.at(i), i, 0, 0} );
            }
        }

        auto find_pre = [&path](size_t i) {
            for (; i >= 0 and path.points[i].timestamp == -1; --i)
                ;
            return i;
        };
        auto find_next = [&path](size_t i ){
            for (; i < path.points.size() and path.points[i].timestamp == -1; ++i)
                ;
            return i;
        };

        for ( auto& tup : cross_pathpoint ){
            tup.get<2>() = find_pre( tup.get<1>() );
            tup.get<3>() = find_next( tup.get<1>() );
        }

        shp = SHPCreate("cross", SHPT_POINT);
        dbf = DBFCreate("cross");
        DBFAddField(dbf, "prevIdx", FTInteger , 10, 0);
        DBFAddField(dbf, "succIdx", FTInteger, 10, 0);
        DBFAddField(dbf, "distToPrev", FTDouble, 10, 3);
        DBFAddField(dbf, "distToSucc", FTDouble, 10, 3);
        DBFAddField(dbf, "prev date", FTString, 20, 0);
        DBFAddField(dbf, "succ date", FTString , 20, 0);
        DBFAddField(dbf, "prev timetamp", FTInteger, 10, 0);
        DBFAddField(dbf, "succ timestamp", FTInteger, 10, 0);

        for( auto& tup: cross_pathpoint ){
            PathPoint const& cross = tup.get<0>();
            PathPoint const& prev = path.points.at(tup.get<2>());
            PathPoint const& succ = path.points.at(tup.get<3>());

            SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, (double*)& cross.x, (double*)& cross.y, nullptr);
            const int append = -1;
            int newID = SHPWriteObject( shp, append, obj);
            SHPDestroyObject(obj);
            DBFWriteIntegerAttribute( dbf, newID, 0, tup.get<2>());
            DBFWriteIntegerAttribute( dbf, newID, 1, tup.get<3>());
            DBFWriteDoubleAttribute( dbf, newID, 2, cross.dist_of_path - prev.dist_of_path);
            DBFWriteDoubleAttribute( dbf, newID, 3, succ.dist_of_path - cross.dist_of_path);
            strftime( str, sizeof str, "%Y-%m-%d %H:%M:%S", localtime( & prev.timestamp) );
            DBFWriteStringAttribute( dbf, newID, 4, str);
            strftime( str, sizeof str, "%Y-%m-%d %H:%M:%S", localtime( & succ.timestamp) );
            DBFWriteStringAttribute( dbf, newID, 5, str);
            DBFWriteIntegerAttribute( dbf, newID, 6, prev.timestamp);
            DBFWriteDoubleAttribute( dbf, newID, 7, succ.timestamp);
        }

        SHPClose(shp);
        DBFClose(dbf);
    }*/
/*
    BOOST_AUTO_TEST_CASE(network_candidate_query){
        Point center(118.881350, 32.050881);
        Point center2(118.892355,32.046942);
        auto candidates = network.query(center, 100);
        auto candidates_2 = network.query(center2, 30);

        SimpleDebugDump debug;
        debug.add_point(center, "center");
        debug.add_points(candidates, "candidates");
        debug.add_point(center2, "center2").add_points(candidates_2, "candidates2");


        debug
            .add_point( network.cross(359), "cross 359" )
            .add_point( network.cross(429), "cross 429" );

        Path path = network.shortest_path_Astar(359, 429);
        debug.add_path_as_line(path, "shortest path from 359 to 429");


        path = network.shortest_path( 121, 105 );
        debug.add_path_as_line(path, "shortest path from 121 ti 105");

        path = network.shortest_path_Astar(2646, 2901);
        debug.add_path_as_line(path, "shortest path from 2646, 2901");


        Point p1(118.86024, 32.01356);
        Point p2(118.86034, 32.01461);

        debug.add_point(p1, "unbound point 1");
        debug.add_point(p2, "unbound point 2");

        CandidatePoint cp1 = network.project(p1);
        CandidatePoint cp2 = network.project(p2);
        debug.add_point(cp1, "candidate of p1")
            .add_point(cp2, "candidate of p2")
            .add_path_as_line(network.shortest_path_Astar(cp1, cp2), "shortest path from cp1 to cp2");
        debug.write("network_candidate_query.geojson");

        SimpleDebugDump debug2;
        //CandidatePoint a = network.project(Point(118.734162789, 32.0315508));
        CandidatePoint a = network.project(Point(118.7343721, 32.0308756));
        CandidatePoint b = network.project(Point(118.734388339641, 32.030857));
        debug2.add_point(a).add_point(b);
        break_point();
        path = network.shortest_path_Astar(a, b);
        debug2.add_path_as_line(path).write("shortest_path_test.geojson");

    }
    BOOST_AUTO_TEST_CASE(network_shortest_path){
        CandidatePoint c1 = network.project(Point(118.93078,31.67995));
        CandidatePoint c2 = network.project(Point(118.93638, 31.68304));
        break_point();
        network.shortest_path(c1, c2);

    }*/

BOOST_AUTO_TEST_SUITE_END()
/*
#include  <boost/timer.hpp>

BOOST_AUTO_TEST_SUITE(ivmm_ex)
Network network;

vector<GpsPoint> loadGps(string const& filename){
    vector<GpsPoint> gps;
    ifstream ins(filename);
    string line;
    vector<string> split_result;
    while ( getline(ins, line) ){
        split_result.clear();
        boost::split(split_result, line, boost::is_any_of(","), boost::token_compress_off);
        tm datetm;
        strptime(split_result[1].c_str(), "%Y-%m-%d %H:%M:%S", &datetm);
        time_t timestemp = mktime(&datetm);
        double x = atof(split_result[2].c_str());
        double y = atof(split_result[3].c_str());
        gps.emplace_back(x, y, (long)timestemp);
    }
    ins.close();
    return gps;
}

BOOST_AUTO_TEST_CASE(ivmmex){
    break_point();
    boost::timer timer;
    BOOST_ASSERT( network.load("/home/syh/Desktop/北京地图修正/road"));
    fmt::print("build network cost : {}\n", timer.elapsed() );
    IVMM ivmm(&network);
    vector<GpsPoint> gps = loadGps("/home/syh/Desktop/366-out.txt");
    IVMMParamEX param;
    param.project_dist_mean = 5;
    param.project_dist_stddev = 10;
    param.candidate_query_radious = 100;
    param.candidate_limit = 5;
    param.beta = 5000;
    param.window = 100;

    timer.restart();
    Path path = ivmm.ivmm_ex( gps, param);
    fmt::print("ivmm ex cost : {}\n", timer.elapsed());
    ofstream o("/home/syh/Desktop/path.geojson");
    o << path.geojson_feature() << endl;
    o.close();

    IVMMParam param2;
    param2.project_dist_mean = 5;
    param2.project_dist_stddev = 10;
    param2.candidate_query_radious = 100;
    param2.candidate_limit = 5;
    param2.beta = 5000;
    timer.restart();
    path = ivmm.ivmm(gps, param2);
    o.open("/home/syh/Desktop/path2.geojson");
    fmt::print("ivmm cost:{}\n", timer.elapsed() );
    o << path.geojson_feature() << endl;
    o.close();
}

BOOST_AUTO_TEST_SUITE_END()
*/

    /*

#include  <boost/tuple/tuple.hpp>

void draw_point(Point const& p, const string& name){
    string base = boost::algorithm::erase_last_copy(name, ".shp");
    string shp_file = base + ".shp";
    SHPHandle shp = SHPCreate(shp_file.c_str(), SHPT_POINT);
    SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, (double*)&p.x, (double*)&p.y, nullptr);
    SHPWriteObject(shp,-1,obj);
    SHPDestroyObject(obj);
    SHPClose(shp);
}

template<typename Seq>
typename enable_if<is_base_of<Point, typename Seq::value_type>::value,void>::type
draw_path(Seq const& seq, string const& name){
    string base = boost::algorithm::erase_last_copy(name, ".shp");
    string shp_file = base + ".shp";
    SHPHandle shp = SHPCreate(shp_file.c_str(), SHPT_ARC);
    double x[seq.size()];
    double y[seq.size()];
    boost::transform(seq, x, key_of(&Point::x));
    boost::transform(seq, y, key_of(&Point::y));
    SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, seq.size(), x,y,nullptr);
    SHPWriteObject(shp, -1, obj);
    SHPDestroyObject(obj);
    SHPClose(shp);
}

template<typename Seq>
typename enable_if<is_same<Path, typename Seq::value_type>::value, void> ::type
draw_paths(Seq const& seq, string const& name){
   string shp_file = boost::algorithm::erase_last_copy(name, ".shp") + ".shp";
  SHPHandle shp = SHPCreate(shp_file.c_str(), SHPT_ARC);
  for (auto& pth : seq){
    double x[pth.points.size()];
    double y[pth.points.size()];
    boost::transform(pth.points, x, key_of(&Point::x));
    boost::transform(pth.points, y, key_of(&Point::y));
    SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, pth.points.size(), x,y,nullptr);
    SHPWriteObject(shp, -1, obj);
    SHPDestroyObject(obj);
  }
  SHPClose(shp);
}

#include  <boost/timer.hpp>
#include  <boost/format.hpp>
BOOST_AUTO_TEST_SUITE(network_test)

    Network network("./../shp/cross", "./../shp/road");
    SampleGenerator generator(&network);


BOOST_AUTO_TEST_CASE(network_kdtree_and_shortest_path_test){
    boost::timer t;
    draw_points(network.query(Point (121.18265,30.89549), 150), "query1");
    t.restart();
    auto pth = network.shortest_path(6891,3182);
    draw_path(pth.points,"sp_1");
    cout << "shortest path from 6891 to 3182 cost : " << t.elapsed() << endl;

    vector<PathPoint> points;
    for(int i = 0;  i < 10; ++i){
        points.push_back(pth.point_of_dist(i*100));
    }
    draw_points(points, "path_of_test");

    auto cp1 = network.query(Point(121.43548,31.01197), 100, 1);
    auto cp2 = network.query(Point(121.4389,31.012517), 100, 1);
    pth = network.shortest_path(cp1[0], cp2[0]);
    draw_path(pth.points, "sp_c_1");

    cp2 = network.query(Point(121.43319,31.011249), 100 , 1);

    pth = network.shortest_path(cp1[0], cp2[0]);
    draw_path(pth.points, "sp_c_2");

    cp2 = network.query(Point(121.43480,31.01894), 100, 1);
    pth = network.shortest_path(cp1[0], cp2[0]);
    draw_path(pth.points, "sp_c_3");

    t.restart();
    pth = network.shortest_path(16034, 6641);
    cout << "shortest path from 16034 to 6641 cost : " << t.elapsed() << endl;

    t.restart();
    auto pth2 = network.shortest_path_Astar(16034, 6641);
    cout << "shortest path A* from 16034 to 6641 cost : " << t.elapsed() << endl;

    BOOST_CHECK_CLOSE(pth.length, pth2.length, 1e-6);
    BOOST_CHECK_EQUAL_COLLECTIONS(pth.points.begin(), pth.points.end(), pth2.points.begin(), pth2.points.end());

}

BOOST_AUTO_TEST_CASE(k_shortest_path_test){
    auto paths = network.k_shortest_path_Yen(1176, 8292, 10);
    for(size_t i = 0; i < paths.size(); ++i){
        draw_path(paths[i].points, (boost::format("k_sp_%d")%i).str());
    }
    //auto pth = network.shortest_path_Astar(7570, 2660);
    //draw_path(pth.points, "sp_7570_2660");
}

BOOST_AUTO_TEST_CASE(sample_generator_test){
    LaunchParam param;
    param.speed_white_error_stddev = 3;
    param.time_sample_mean = 60;
    param.time_sample_stddev = 15;
    param.speed_confidence = 0.6;
    param.speed_similar = 0.7;
    param.gps_sample_stddev = 50;
    auto path = network.shortest_path_Astar(2641, 5886);
    auto result = generator.sample(path, param);
    draw_path(path.points, "sp");
    draw_points(result, "org");
    auto gps = generator.create(result, param);
    draw_points(gps, "gps");

    vector<PathPoint> pathpoints;
    boost::transform(gps, back_inserter(pathpoints), [&path](Point const& p){
            return path.project(p);
            });
    draw_points(pathpoints,"projects");
    //cout << "shortest path cost : " << t.elapsed() << endl;
    //draw_path(path.points, "sp");


    //auto results = generator.launch(13432, 1137, 5, param);
    //network.k_shortest_path_Yen(13432, 1137,5);
    draw_points(vector<Cross>{network.cross(13432), network.cross(1137)}, "13432-1137");

    RoadSegment const* r = network.road(5257);
    Point testp(121.395,31.0959);
    auto cptest = r->candidate_of(testp);
    draw_points(vector<Point>{testp, cptest}, "road_segment_project_test");
}

static vector<GpsPoint> load_gps_points(string const& file){
    ifstream f(file);
    string line;
    vector<string> each_element;
    vector<GpsPoint> gps;
    while(getline(f, line)){
        if (boost::starts_with(line,"#") )
                continue;
        boost::trim_if(line, boost::is_any_of(" ,"));
        boost::split(each_element, line, boost::is_any_of(", "), boost::token_compress_on);

        if(each_element.size() >= 3){
            double x;
            double y;
            long timestamp;
            sscanf(each_element[0].c_str(), "%lf",&x);
            sscanf(each_element[1].c_str(), "%lf", &y);
            sscanf(each_element[2].c_str(), "%ld", &timestamp);
            gps.emplace_back(x, y, timestamp);
        }
        each_element.clear();
    }

    return gps;
}
#include  <boost/range/algorithm.hpp>
#include  <boost/algorithm/cxx11/copy_if.hpp>

BOOST_AUTO_TEST_CASE(ivmm_test){

    IVMM ivmm(&network);

    IVMMParam param;
    param.candidate_limit = 5;
    param.project_dist_mean = 40.0;
    param.project_dist_stddev = 35.0;
    param.candidate_query_radious = 100;
    param.beta = 5000;
    auto gps = load_gps_points("./outs/sample/1793_1.log");

    break_point();
    auto path = ivmm.ivmm(gps, param);

    draw_path(path.points, "ivmm_path_01");
    draw_points(path.points, "ivmm_path_points_01");
    path.points.resize(boost::algorithm::copy_if(path.points, path.points.begin(), [](PathPoint const& p){ return p.cid != -1; })
            - path.points.begin());
    path.update();
    draw_path(path.points, "ivmm_path_02");
    draw_points(path.points, "ivmm_path_points_02");

}


static void draw_segments(PathInfo::SegmentVector const& segments, string const& name){
    string shp_file_name = boost::erase_last_copy(name ,".shp") + ".shp";
    SHPHandle shp = SHPCreate(shp_file_name.c_str(), SHPT_ARC);
    for(auto& seg : segments ){
        double x[2] = { seg.first->x, seg.second->x };
        double y[2] = { seg.first->y, seg.second->y };
        SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
        SHPWriteObject(shp, -1, obj);
        SHPDestroyObject( obj );
    }
    SHPClose(shp);
}


BOOST_AUTO_TEST_CASE (path_info_test){
    auto path = network.shortest_path(11474, 10856);
    draw_path( path.points, "shortest_path_for_path_info_test" );
    PathInfo pathinfo(path);
    vector<Point> sample_points;
    PathInfo::SampleVector segments;
    for( auto & each : pathinfo.samples ){
        sample_points.push_back(each.first);
    }

    BOOST_CHECK_EQUAL( pathinfo.count() , pathinfo.samples.size() );

    draw_points( sample_points, "point_for_path_info_test" );
    draw_segments( pathinfo.segments, "segments_for_path_info_test" );

    Point p(121.473667, 31.221415);
    break_point();
    auto sample = pathinfo.nearlest_sample( p);
    draw_point(p, "nearlest_point_for_path_info_test");
    vector<Point> seg{*sample.second->first, *sample.second->second};
    draw_point(sample.first, "sample_for_path_info_test");
    draw_path(seg, "sample_segment_for_path_info_test");

//    Point p2(121.4704349, 31.22053927) ;
    Point p2(121.470434985, 31.220539144);
    auto pps = pathinfo.on_path_point(p2);
    draw_point(p2, "path_info_on_path_point");
    cout << pps.size() << endl;
    if ( not pps.empty() )draw_points(pps, "path_info_on_path_point_result");

}

BOOST_AUTO_TEST_CASE(evaluation_test){
    LaunchParam param;
    param.speed_white_error_stddev = 3;
    param.time_sample_mean = 60;
    param.time_sample_stddev = 15;
    param.speed_confidence = 0.6;
    param.speed_similar = 0.7;
    param.gps_sample_stddev = 50;
    //SampleGenerator generator(&network, 3125187216);
    SampleGenerator generator(&network);
    cout << "seed is " << generator.seed() << endl;
    vector<SampleResult> samples = generator.launch(2641, 5886, 1, param);


    IVMMParam ivmm_param;
    ivmm_param.candidate_limit = 5;
    ivmm_param.project_dist_mean = 40.0;
    ivmm_param.project_dist_stddev = 35.0;
    ivmm_param.candidate_query_radious = 100;
    ivmm_param.beta = 5000;

    IVMM ivmm(&network);
    CandidateGraph graph;
    vector<size_t> best_point_index;
    Path ivmm_path = ivmm.ivmm( samples[0].sample, ivmm_param , &graph, & best_point_index);


    vector<Path> graph_paths;
    vector<Point> best_points;
    for(size_t i = 0; i < best_point_index.size(); ++i){
        best_points.push_back(graph.candidates[i][best_point_index[i]]);
    }

    for(size_t i = 0; i < best_point_index.size() - 1; ++i){
        graph_paths.push_back(
                graph.paths[i][best_point_index[i]][best_point_index[i+1]]
                );
    }

    draw_paths( graph_paths, "best_segments_of_candidate_debug" );
    draw_points(best_points, "best_points_of_candidate_debug");

    PathInfo::SegmentVector not_match_segments;
    PathInfo::SegmentVector match_segments;
    Evaluation evaluation(&network);
    PathInfo labeld_info(samples[0].path);
    PathInfo ivmm_info(ivmm_path);
    evaluation.walk(labeld_info, ivmm_info,
            [&not_match_segments](Segment const& not_match){
                not_match_segments.push_back(not_match);
            },
            [&match_segments](Segment const& matched){
                //match_segments.push_back(matched);
            });

    evaluation.walk(ivmm_info, labeld_info,
            [&not_match_segments](Segment const& not_match){
                not_match_segments.push_back(not_match);
            },
            [&match_segments](Segment const& matched){
                //match_segments.push_back(matched);
            });
    draw_path(samples[0].path.points, "evaluation_test_org_path");
    draw_path(ivmm_path.points,"evaluation_test_ivmm_path");
    draw_segments(not_match_segments,"evaluation_test_not_match_segments");
    draw_segments(match_segments, "evaluation_test_match_segments");
    cout << evaluation.correct_rate_by_distance(labeld_info, ivmm_info) << endl;


    / *
    SHPHandle shp = SHPCreate("label_points_debug", SHPT_POINT);
    DBFHandle dbf = DBFCreate("label_points_debug");
    DBFAddField(dbf, "id", FTInteger, 10, 0);
    DBFAddField(dbf, "roadsegment_id", FTInteger, 10, 0);
    for (auto & each_point :samples[0].path.points){
        SHPObject* point = SHPCreateSimpleObject(SHPT_POINT, 1, & each_point.x, & each_point.y, nullptr);
        int id = SHPWriteObject(shp, -1, point);
        DBFWriteIntegerAttribute(dbf, id, 0, id);
        DBFWriteIntegerAttribute(dbf, id, 1, each_point.belong->id);
        SHPDestroyObject(point);
    }
    DBFClose(dbf);
    SHPClose(shp);
    draw_path(samples[0].path.points, "labeld_path_debug");

    shp = SHPCreate("ivmm_points_debug", SHPT_POINT);
    dbf = DBFCreate("ivmm_points_debug");
    DBFAddField(dbf, "id", FTInteger, 10, 0);
    DBFAddField(dbf, "roadsegment_id", FTInteger, 10, 0);
    for (auto & each_point :ivmm_path.points){
        SHPObject* point = SHPCreateSimpleObject(SHPT_POINT, 1, & each_point.x, & each_point.y, nullptr);
        int id = SHPWriteObject(shp, -1, point);
        DBFWriteIntegerAttribute(dbf, id, 0, id);
        DBFWriteIntegerAttribute(dbf, id, 1, each_point.belong->id);
        SHPDestroyObject(point);
    }
    DBFClose(dbf);
    SHPClose(shp);
    draw_path(ivmm_path.points, "ivmm_path_debug");


    cout << "labeld point test" << endl;
    boost::foreach_adjacent(samples[0].path.points, [](PathPoint const& first, PathPoint const& second){
                if (not first.pos_equal(second)){
                    BOOST_CHECK_EQUAL( first.belong->id, second.belong->id );
                }
            });
    cout << "ivmm point test " << endl;
    boost::foreach_adjacent(ivmm_path.points, [](PathPoint const& first, PathPoint const& second){
                if (not first.pos_equal(second)){
                    BOOST_CHECK_EQUAL( first.belong->id, second.belong->id );
                }
            });
    auto debug_pth = network.shortest_path_Astar(541, 17780);
    draw_path( debug_pth.points, "debug_541,17780" );* /
}

BOOST_AUTO_TEST_SUITE_END()*/

#include<boost/test/included/unit_test.hpp>
