#include  <iostream>
#include  <fstream>
#include  <boost/program_options.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/range.hpp>
#include  <boost/range/algorithm.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <shapefil.h>
#include    "network.h"
#include    "key_visitor.hpp"

using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


void drawBeginEnd(Network const& network, fs::path const& out, string const& begin, string const& end){
    SHPHandle shp = SHPCreate(out.c_str(), SHPT_POINT);
    DBFHandle dbf = DBFCreate(out.c_str());
    DBFAddField(dbf, "cross", FTString, 11, 0);
    double x = network.cross(begin).x;
    double y = network.cross(begin).y;
    SHPObject * obj = SHPCreateSimpleObject(SHPT_POINT, 1, &x , &y, nullptr);
    DBFWriteStringAttribute(dbf, SHPWriteObject(shp, -1, obj), 0, begin.c_str());
    SHPDestroyObject(obj);
    x = network.cross(end).x;
    y = network.cross(end).y;
    obj = SHPCreateSimpleObject(SHPT_POINT, 1 , &x, &y, nullptr);
    DBFWriteStringAttribute(dbf, SHPWriteObject(shp, -1, obj), 0, end.c_str());
    SHPClose(shp);
    DBFClose(dbf);
}
void drawPoints(Network const& network, fs::path const& in, fs::path const& out, string const& begin, string const& end){
    SHPHandle shp = SHPCreate(out.c_str(), SHPT_POINT);
    DBFHandle dbf = DBFCreate(out.c_str());
    DBFAddField(dbf, "cross", FTString, 11, 0);
    DBFAddField(dbf, "time", FTString, 20, 0);
    DBFAddField(dbf, "timestamp", FTInteger, 10, 0);
    fs::ifstream ifs(in);
    string cross;
    string time;
    long timestamp;
    bool beginToDraw = false;
    while ( getline(ifs, cross, ',') && getline(ifs, time, ',') && ifs >> timestamp && ifs.ignore() ){
        if ( ! beginToDraw && cross == begin )
            beginToDraw = true;
        if ( beginToDraw  ){
            double x = network.cross(cross).x;
            double y = network.cross(cross).y;
            SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
            int const insert = -1;
            int newID = SHPWriteObject(shp, insert, obj);
            DBFWriteStringAttribute(dbf, newID, 0 , cross.c_str());
            DBFWriteStringAttribute(dbf, newID, 1, time.c_str());
            DBFWriteIntegerAttribute(dbf, newID, 2, timestamp);
            SHPDestroyObject(obj);
        }

        if ( cross == end )
            break;
    }
    SHPClose(shp);
    DBFClose(dbf);
}

void drawPath(Network const& network, fs::path const& in, fs::path const& out, string const& begin, string const& end){
    SHPHandle shp = SHPCreate(out.c_str(), SHPT_ARC);
    DBFHandle dbf = DBFCreate(out.c_str());
    DBFAddField(dbf, "length", FTDouble, 11, 6);
    DBFAddField(dbf, "cost", FTInteger, 11, 0);
    fs::ifstream ifs(in);
    string cross;
    long timestamp;
    string preCross;
    long preTimestamp;
    bool beginToDraw = false;
    getline(ifs, cross, ',') && ifs.ignore(numeric_limits<streamsize>::max(), ',') && ifs >> timestamp && ifs.ignore();
    while ( getline(ifs, cross, ',') && ifs.ignore(numeric_limits<streamsize>::max(), ',')  && ifs >> timestamp && ifs.ignore() ){
        if ( ! beginToDraw && cross == begin )
            beginToDraw = true;

        if ( beginToDraw  && cross != preCross){
            adjacent_edge const* edge = network.edge(preCross, cross);
            size_t sz = edge->road->points.size();
            double x[sz];
            double y[sz];
            boost::transform(edge->road->points, x, key_of(&Point::x));
            boost::transform(edge->road->points, y, key_of(&Point::y));
            SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, sz, x, y, nullptr);
            int const insert = -1;
            int newID = SHPWriteObject(shp, insert, obj);
            DBFWriteDoubleAttribute(dbf, newID, 0, edge->road->length);
            DBFWriteIntegerAttribute(dbf, newID, 1, timestamp - preTimestamp);
        }

        if ( cross == end )
            break;
        preCross = std::move(cross);
        preTimestamp = timestamp;
    }
    SHPClose(shp);
    DBFClose(dbf);

}

int main(int argc, char *argv[])
{
    string road;
    fs::path output;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("road,r", po::value(&road)->default_value("../Date/map/bj-road-epsg3785"), "specify the road shp file")
        ("output,o", po::value(&output)->required(), "specify the outout dir");
    po::variables_map vm;

    try{
        po::store(po::parse_command_line(argc, argv, desc), vm);
        vm.notify();
    }catch(std::exception const& err){
        cerr << err.what() << endl;
        cerr << desc << endl;
        return 1;
    }
    
    if ( vm.count("help") ){
        cout << desc << endl;
        return 0;
    }

    if ( ! fs::exists(output) ){
        try{
            fs::create_directories(output);
        }catch(fs::filesystem_error const& err){
            cerr << err.what() << endl;
            return 1;
        }
    }
    Network network;
    if (! network.load(road)){
        cerr << "can not load map " << road << endl;
    }


    string begin, end;
    if (cin >> begin >> end && cin.ignore() )
        drawBeginEnd(network, output/"beginEnd", begin, end);
    string line;
    while( getline(cin,  line) ){
        fs::path trajPath(line);
        string name  = trajPath.filename().stem().string();
        fs::path pointOuput = output / ( name + "-point");
        fs::path pathOuput = output / ( name + "-path" );
        drawPoints(network, line , pointOuput, begin, end);
        drawPath(network, line, pathOuput, begin, end);
    }
    return 0;
}
