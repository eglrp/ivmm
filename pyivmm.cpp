#include  <boost/python.hpp>
#include  <boost/range/algorithm.hpp>
#include  <algorithm>
#include    "road.h"
#include    "network.h"
#include    "boost_python_converter.hpp"
#include    "sample_generator.h"

namespace py = boost::python;

py::tuple point_dist_to_sement(Point const& self, Point const& a, Point const& b){
    Point mid;
    double rst;
    rst = self.dist_to_segment(a, b, mid);
    py::tuple tup = py::make_tuple(rst, mid);
    return tup;
}

double point_getitem(Point const& self, int index){
    return self[index];
}

void point_setitem(Point & self, int index, double value){
    self[index] = value;
}
void export_points(){
    py::class_<Point>("Point")
        .def(py::init<double, double>())
        .def_readwrite("x", &Point::x)
        .def_readwrite("y", &Point::y)
        .def("sqr_dist", (double (Point::*)(Point const&)const)&Point::sqr_dist)
        .def("sqr_dist", (double (Point::*)(double, double)const)&Point::sqr_dist)
        .def("dist_to_segment", &point_dist_to_sement)
        .def("gis_dist", &Point::gis_dist)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def(py::self += py::self)
        .def(py::self + py::self)
        .def(py::self -= py::self)
        .def(py::self - py::self)
        .def(py::self *= py::other<double>())
        .def(py::self * py::other<double>())
        .def(py::other<double>() * py::self)
        .def(str(py::self))
        .def(repr(py::self));
}

void export_gpspoint(){
    py::class_<GpsPoint, py::bases<Point> >("GpsPoint")
        .def(py::init<double, double, double>())
        .def_readwrite("timestamp", &GpsPoint::timestamp)
        .def(py::self == py::self)
        .def(py::self != py::self);
}

void export_cross(){
    py::class_<Cross, py::bases<Point> >("Cross", py::no_init)
        .def_readonly("id", &Cross::id)
        .def(py::self == py::self)
        .def(py::self != py::self);
}

void export_candidatepoint(){
    py::class_<CandidatePoint, py::bases<Point> >("CandidatePoint", py::no_init)
        .def_readonly("belong", &CandidatePoint::belong)
        .def("distance_from_begin", &CandidatePoint::distance_from_begin)
        .def_readonly("where", &CandidatePoint::where)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def(str(py::self))
        .def(repr(py::self));
}
#include  <boost/python/suite/indexing/vector_indexing_suite.hpp>
void export_roadsegment(){

    py::class_<std::vector<CandidatePoint> >("CandidateVector",py::no_init)
        .def(py::vector_indexing_suite<std::vector<CandidatePoint> >());
    py::class_<RoadSegment, RoadSegment *>("RoadSegment",py::no_init)
        .def_readonly("id", &RoadSegment::id)
        .def_readonly("dbID",& RoadSegment::dbId )
        .def_readonly("bidir", &RoadSegment::bidir)
        .def_readonly("begin", &RoadSegment::begin)
        .def_readonly("end", &RoadSegment::end)
        .def_readonly("points", &RoadSegment::points)
        .def("path_follow", &RoadSegment::path_follow)
        .def("candidate_at_dist", &RoadSegment::candidate_at_dist)
        .def("candidate_of", &RoadSegment::candidate_of)
        .def("candidate_at_normal", &RoadSegment::candidate_at_normal)
        .def(py::self == py::self)
        .def(py::self != py::self);
}
#include  <boost/range/adaptors.hpp>


void export_network(){
    py::class_<PathPoint, py::bases<CandidatePoint> >("PathPoint")
        .def_readonly("dist_of_path", &PathPoint::dist_of_path)
        .def_readonly("cid", &PathPoint::cid);


    py::class_<std::vector<PathPoint> >("PathPointVector", py::no_init)
        .def(py::vector_indexing_suite<std::vector<PathPoint> >());

    py::class_<Path>("Path", py::no_init)
        .def_readonly("length", &Path::length)
        .def_readonly("points", &Path::points)
        .def("update", &Path::update)
        .def("append", &Path::append)
        .def("project", &Path::project)
        .def("empty", &Path::empty)
        .def("infinity", &Path::infinity)
        .def("empty_path", &Path::empty_path).staticmethod("empty_path")
        .def("inf_path", &Path::inf_path).staticmethod("inf_path");

    to_python_converter()
        .to_python<std::vector<Path> >();

    //py::class_<Network, Network*>("Network",py::init<std::string const&, std::string const&>())
    py::class_<Network, Network*>("Network")
        .def("load", (bool (Network::*)(std::string const&))& Network::load)
        .def("load", (bool (Network::*)(std::string const&, std::string const&))&Network::load)
        .def("query", (std::vector<CandidatePoint>(Network::*)(Point const&, double)const)&Network::query)
        .def("query", (std::vector<CandidatePoint>(Network::*)(Point const&, double, int)const)&Network::query)
        .def("shortest_path", (Path(Network::*)(int, int)const)&Network::shortest_path)
        .def("shortest_path", (Path(Network::*)(Cross const&, Cross const&)const)&Network::shortest_path)
        .def("shortest_path", (Path(Network::*)(CandidatePoint const&, CandidatePoint const&)const)&Network::shortest_path)
        .def("shortest_path_Astar",(Path(Network::*)(int, int)const)&Network::shortest_path_Astar)
        .def("shortest_path_Astar",(Path(Network::*)(Cross const&, Cross const&)const)&Network::shortest_path_Astar)
        .def("k_shortest_path", (std::vector<Path>(Network::*)(Cross const&, Cross const&, int)const)
                &Network::k_shortest_path)
        .def("k_shortest_path", (std::vector<Path>(Network::*)(int, int, int)const)
                &Network::k_shortest_path)
        .def("k_shortest_path_Yen", &Network::k_shortest_path_Yen)
        .def("cross_bound", &Network::cross_bound)
        .def("roadsegment_bound", &Network::roadsegment_bound)
        .def("cross", (Cross const& (Network::*)(int)const)&Network::cross, py::return_value_policy<py::copy_const_reference>())
        .def("cross", (Cross const& (Network::*)(std::string const&)const)&Network::cross, py::return_value_policy<py::copy_const_reference>())
        .def("road", (RoadSegment const* (Network::*)(int)const)&Network::road, py::return_value_policy<py::reference_existing_object>())
        .def("road", (RoadSegment const* (Network::*)(std::string const&)const)&Network::road, py::return_value_policy<py::reference_existing_object>())
        .def("project", &Network::project)
        .def("save_cross_to_map", &Network::save_cross_to_map);
}


SampleResult launch(SampleGenerator& s, py::list const& list, LaunchParam const& p){
    std::vector<int> crosses;
    py::container_utils::extend_container(crosses, list);
    return s.launch(crosses,p );

}
void export_sample_generator(){
    
    py::class_<LaunchParam>("LaunchParam")
        .def_readwrite("gps_sample_stddev", &LaunchParam::gps_sample_stddev)
        .def_readwrite("speed_white_error_stddev", &LaunchParam::speed_white_error_stddev)
        .def_readwrite("time_sample_mean",&LaunchParam::time_sample_mean)
        .def_readwrite("time_sample_stddev", &LaunchParam::time_sample_stddev)
        .def_readwrite("speed_confidence", &LaunchParam::speed_confidence)
        .def_readwrite("speed_similar", &LaunchParam::speed_similar);

    py::class_<SampleOriginalPoint, py::bases<PathPoint> >("SampleOriginalPoint")
        .def_readonly("timestamp", &SampleOriginalPoint::timestamp);

    py::class_<std::vector<SampleOriginalPoint> >("SampleOriginalPointVector")
        .def(py::vector_indexing_suite<std::vector<SampleOriginalPoint> >());

    py::class_<std::vector<GpsPoint> >("GpsPointVector")
        .def(py::vector_indexing_suite<std::vector<GpsPoint> >());


    py::class_<SampleResult>("SampleResult")
        .def_readonly("path",&SampleResult::path)
        .def_readonly("origin", &SampleResult::origin)
        .def_readonly("sample", &SampleResult::sample);

    py::class_<std::vector<SampleResult> >("SampleResultVector")
        .def(py::vector_indexing_suite<std::vector<SampleResult> >());

    py::class_<SampleGenerator>("SampleGenerator",py::init<Network*>())
        .def("launch", (std::vector<SampleResult>(SampleGenerator::*)(int ,int ,int , LaunchParam const&))&SampleGenerator::launch)
        .def("launch", &launch);
}

BOOST_PYTHON_MODULE(pyivmm){
    export_points();
    export_cross();
    export_gpspoint();
    export_candidatepoint();
    export_roadsegment();
    export_network();
    export_sample_generator();
}
