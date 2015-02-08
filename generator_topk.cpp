#include  <boost/program_options.hpp>
#include  <boost/algorithm/string.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <boost/range/algorithm.hpp>
#include  <functional>
#include  <utility>

#include  <iostream>
#include  <shapefil.h>

#include    "network.h"
#include    "key_visitor.hpp"

template<typename H>
struct Handle{
    H handle = nullptr;
    std::function<void(H)> close;
    operator H(){
        return handle;
    }

    Handle(H h, std::function<void(H)> close):handle(h),close(close){}

    ~Handle(){
        if (handle){
            close(handle);
        }
    }
};


using namespace std;
int main(int argc, char *argv[])
{
    namespace po = boost::program_options;
    namespace fs = boost::filesystem;


    fs::path road;
    fs::path output;
    fs::path input;
    long topk;
    po::options_description config("Config");
    config.add_options()
        ("help,h", "show help")
        ("road,r", po::value(&road), "road shp file")
        ("output,o", po::value(&output)->default_value("topk"), "topk road dir")
        ("topk,k", po::value(&topk)->default_value(5000), "topk")
        ("input-file,I", po::value(&input), "input file");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, config), vm);
    po::notify(vm);

    if ( vm.find("road") == vm.end() ){
        cerr << config << endl;
        return 1;
    }

    if ( vm.find("input-file") == vm.end() ){
        cerr << config << endl;
        return 1;
    }


    if ( vm.find("help") != vm.end() ){
        cout << config<< endl;
        return 0;
    }

    Network network;
    if ( not network.load(road.string()) ){
        cerr << "can not load network " << endl;
        return 1;
    }

    fs::ifstream ins(input);
    if (not ins){
        cerr << "can not open " << input << endl;
        return 1;
    }

    if ( not fs::exists(output) ){
        try{
            fs::create_directories(output);
        }catch(fs::filesystem_error const& err){
            cerr << err.what() << endl;
            return 1;
        }
    }

    Handle<SHPHandle> shp{ SHPCreate((output/"topk").c_str(), SHPT_ARC), &SHPClose};
    Handle<DBFHandle> dbf{ DBFCreate((output/"topk").c_str()), &DBFClose};
    if (not shp or not dbf){
        return 1;
    }

    DBFAddField(dbf, "ID", FTString, 11, 0);
    DBFAddField(dbf, "count", FTInteger, 10, 0);

    string line;
    vector<string> meta;
    for (int i = 0; i < topk and getline(ins, line); ++i){
        meta.clear();
        boost::split(meta, line, boost::is_any_of(",") );
        RoadSegment const* road = network.road(meta.at(0));
        long count = atol(meta.at(1).c_str());
        double x[road->points.size()];
        double y[road->points.size()];
        boost::transform(road->points, x, key_of(&Point::x));
        boost::transform(road->points, y, key_of(&Point::y));
        SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, road->points.size(), x, y, nullptr);
        int newId = SHPWriteObject(shp, -1, obj);
        DBFWriteStringAttribute(dbf, newId, 0, meta.at(0).c_str());
        DBFWriteIntegerAttribute(dbf, newId, 1, count);
        SHPDestroyObject(obj);
    }
    return 0;
}
