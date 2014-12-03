#include <boost/range/algorithm.hpp>
#include  <boost/algorithm/string.hpp>
#include  <boost/program_options.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <boost/property_tree/ptree.hpp>
#include  <boost/property_tree/ini_parser.hpp>
#include  <boost/timer.hpp>
#include <iterator>
#include  <string>
#include  <iostream>
#include  <vector>
#include  <ctime>
#include  <shapefil.h>

#include    "road.h"
#include    "ivmm.h"
#include    "format.h"
#include "range_extend.hpp"
#include "simple_guard.hpp"
#include "simple_progress.hpp"
#include "pathendpointwalker.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace pt = boost::property_tree;
using namespace std;

void generator_default_ini(char const* filename) throw (pt::ini_parser_error);
IVMMParam load_ivmm_param(pt::ptree const& ptree);
IVMMParamEX load_ivmm_param_ex( pt::ptree const& ptree);
vector<GpsPoint> loadGps(fs::path const& gpsFile);



void savePath(Path const& path, const char* file)
{
    SHPHandle shp = SHPCreate(file, SHPT_ARC);
    DBFHandle dbf = DBFCreate(file);
    DBFAddField(dbf, "length", FTDouble, 10, 6);
    for (auto p : path.points | boost::adjacented)
    {
        Point const& p1 = p.get<0>();
        Point const& p2 = p.get<1>();
        if (p1.pos_equal(p2)) continue;
        double x[2] = {p1.x, p2.x};
        double y[2] = {p1.y, p2.y};
        SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
        const int insert = -1;
        int id = SHPWriteObject(shp, insert, obj);
        DBFWriteDoubleAttribute(dbf, id, 0, p1.gis_dist(p2));
        SHPDestroyObject(obj);
    }
    SHPClose(shp);
    DBFClose(dbf);
}

void savePoints(Path const& path, const char* file)
{
    SHPHandle shp = SHPCreate(file, SHPT_POINT);
    DBFHandle dbf = DBFCreate(file);
    DBFAddField(dbf, "timestamp", FTInteger, 10,0);
    DBFAddField(dbf, "time", FTString, 20, 0);
    auto writeToShp = [&](PathPoint const& p)
    {
        double x = p.x;
        double y = p.y;
        SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, &x,&y,nullptr);
        int const insert = -1;
        int newId = SHPWriteObject(shp, insert, obj);
        DBFWriteIntegerAttribute(dbf, newId, 0, p.timestamp);
        char timeStr[20];
        tm t = *localtime(&p.timestamp);
        strftime(timeStr, sizeof timeStr, "%Y-%m-%d %H:%M:%S", &t);
        DBFWriteStringAttribute(dbf, newId, 1, timeStr);
        SHPDestroyObject(obj);
    };
    boost::foreach_adjacent<boost::with_prepare>(path.points, [&](PathPoint const& first){
                writeToShp(first);
            },
            [&](PathPoint const& first, PathPoint const& second){
                if ( second.pos_equal(first) && second.timestamp == first.timestamp && second.timestamp != -1)
                {
                    writeToShp(second);
                }
            });
    SHPClose(shp);
    DBFClose(dbf);
}

void saveEndpoint(Network const& network, Path const& path, const char* file)
{
    SHPHandle shp = SHPCreate(file, SHPT_POINT);
    DBFHandle dbf = DBFCreate(file);
    DBFAddField(dbf, "cid", FTInteger, 10, 0);
    DBFAddField(dbf, "roadDBID", FTString, 11, 0);
    DBFAddField(dbf, "timestamp", FTInteger, 10, 0);
    DBFAddField(dbf, "time", FTString, 20, 0);
    DBFAddField(dbf, "crossDBID", FTString, 20,0);
    PathEndpointWalker<Path const> walker(path);
    char timeStr[20];
    while ( not walker.isEnd() )
    {
        double x, y;
        if ( walker.first != path.points.begin() - 1 and walker.first->cid != -1)
        {
            x = walker.first->x;
            y = walker.first->y;
            long t = walker.first->timestamp;
            SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
            const int insert = -1;
            int newID = SHPWriteObject(shp, insert, obj);
            DBFWriteIntegerAttribute(dbf, newID, 0, walker.first->cid);
            DBFWriteStringAttribute(dbf, newID, 1, walker.first->belong->dbId.c_str());
            DBFWriteIntegerAttribute(dbf, newID, 2, t);
            tm tt = * localtime(&t);
            strftime(timeStr, sizeof timeStr, "%Y-%m-%d %H:%M:%S", &tt);
            DBFWriteStringAttribute(dbf, newID,3, timeStr);
            DBFWriteStringAttribute(dbf, newID, 4, network.cross(walker.first->cid).dbId.c_str());
            SHPDestroyObject(obj);
        }else if ( walker.second != path.points.end() and walker.second->cid != -1)
        {
            long t = walker.second->timestamp;
            x = walker.second->x;
            y = walker.second->y;
            SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
            const int insert = -1;
            int newID = SHPWriteObject(shp, insert, obj);
            DBFWriteIntegerAttribute(dbf, newID, 0, walker.second->cid);
            DBFWriteStringAttribute(dbf, newID, 1, walker.second->belong->dbId.c_str());
            DBFWriteIntegerAttribute(dbf, newID, 2, t);
            tm tt = * localtime(&t);
            strftime(timeStr, sizeof timeStr, "%Y-%m-%d %H:%M:%S", &tt);
            DBFWriteStringAttribute(dbf, newID,3, timeStr);
            DBFWriteStringAttribute(dbf, newID, 4, network.cross(walker.second->cid).dbId.c_str());
            SHPDestroyObject(obj);

        }
        ++walker;
    }
    SHPClose(shp);
    DBFClose(dbf);
}

int main(int argc, char *argv[]) { string configFile;
    fs::path outputDirs;
    po::options_description desc("ivmm main");
    desc.add_options()
        ("config,c", po::value<string>(&configFile)->default_value(string("default.ini")), "load ivmm config file")
        ("output,o", po::value(&outputDirs), "output dir")
        ("help,h", "show help");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if ( vm.find("help") != vm.end()){
        cout << desc << endl;
        return 0;
    }

    if ( vm.count("output") == 0)
    {
        cerr << "need output dir " << endl;
        cerr << desc << endl;
        return 1;
    }


    pt::ptree ptree;
    if (fs::exists(configFile) ){
        try{
            fs::ifstream ins(configFile);
            pt::read_ini(ins, ptree);
        }catch ( pt::ini_parser_error const& err){
            fmt::print(cerr, "while read config {}", err.what());
            return 1;
        }
    }else{
        try{
            generator_default_ini(configFile.c_str());
            fmt::print("{} not exists generator a default one\n", configFile);
        }catch ( pt::ini_parser_error const& err){
            fmt::print(cerr, "while generator a default config {} \n", err.what());
            return 1;
        }
        return 0;
    }

    string road = ptree.get<string>("Network.road", "road");
    Network network;

    boost::timer timer;
    if ( not network.load(road) ){
        fmt::print("load network from {} fail!\n", road);
        return 1;
    }

    fmt::print("load network within {}s.\n", timer.elapsed());

    if (not fs::exists(outputDirs) ){
        boost::system::error_code err;
        fs::create_directories(outputDirs, err);
        if ( err){
            fmt::print("can not create {}\n {}\n", outputDirs, err.message());
            return 1;
        }
    }
    IVMMParamEX param = load_ivmm_param_ex(ptree);
    IVMM ivmm(&network);

    string line;
    list<string> inputLines;
    while( getline(cin, line))
        inputLines.push_back(std::move(line));

    size_t allLines = inputLines.size();
    while( not inputLines.empty() ){
        line = std::move( inputLines.front() );
        inputLines.pop_front();
        fs::path path(line);
        if (not fs::exists(path) ){
            continue;
        }
        fmt::print("input:{},", path);
        vector<GpsPoint> gps = loadGps( path );
        if ( gps.size() < 10)
        {
            cout << "too few records" <<endl;
            continue;
        }

        fmt::print(" with {} records ", gps.size());
        cout .flush();
        boost::timer timer;
        CandidateGraph g;
        Path matched_path = ivmm.ivmm_ex(gps, param);
        //Path matched_path = ivmm.ivmm(gps,  param);
        if ( matched_path.empty() or matched_path.infinity() )
        {
            fmt::print(" ignored path is :{} ", matched_path.empty()?"empty":"infinity");
        } else
        {
            string filename = path.filename().stem().string();
            string id = boost::copy_range<string>(boost::find_if<boost::return_begin_found>(filename, boost::is_any_of("-")));
            fs::path dir = outputDirs/id;
            if ( not fs::exists(dir))
            {
                try
                {
                    fs::create_directories(dir);
                }catch(fs::filesystem_error const& e)
                {
                    cerr << e.what() << endl;
                    continue;
                }
            }
            //cdbid, rdbid, timestamp, x,y, where, dist of path
            string crossDBID;
            long timestamp;
            char timeStr[20];
            //savePath(matched_path,(outputDirs/id/filename).c_str());
            //savePoints(matched_path, (outputDirs/id/(filename+"-points")).c_str());
            matched_path.estimate_time_at_cross();
            //saveEndpoint(network, matched_path, (outputDirs/id/(filename+"-endpoints")).c_str());
            fs::ofstream os(dir/path.filename());
            for (PathEndpointWalker<Path const> walker(matched_path);not walker.isEnd(); ++walker)
            {
                int cid;
                if ( walker.isFirst())
                {
                    if (walker.second->cid == -1) continue;
                    cid = walker.second->cid;
                    timestamp = walker.second->timestamp;
                } else if ( walker.isLast() )
                {
                    if ( walker.first->cid == -1) continue;
                    cid = walker.first->cid;
                    timestamp = walker.first->timestamp;
                }else
                {
                    cid = walker.first->cid;
                    assert(cid == walker.second->cid);
                    timestamp = walker.first->timestamp;
                    assert(timestamp == walker.second->timestamp);
                }
                crossDBID = network.cross(cid).dbId;
                tm t = *localtime(&timestamp);
                strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &t);
                os << crossDBID << ","<<timeStr<<","<<timestamp<<"\n";
            }
            /*
            for ( PathPoint const& p : matched_path.points){
                crossDBID = p.cid == -1 ? "           ":network.cross(p.cid).dbId;
                }
                x = p.x;
                y = p.y;
                paramOfRoad = p.where;
                distOfPath = p.dist_of_path;
                fmt::print( os, "{},{},{},{},{:.6f},{:.6f},{:.4f},{:.3f}\n",crossDBID, roadDBID, timestamp,timeStr, x, y, paramOfRoad, distOfPath);
            }*/
        }
        fmt::print(" cost {}s {:.2f}%\n", timer.elapsed(), (double(allLines - inputLines.size()) / allLines )*100);

    }
    return 0;
}

void generator_default_ini(char const* filename)throw(pt::ini_parser_error){
    pt::ptree ini;
    ini.put("Network.road", "road");
    ini.put("IVMM.candidate_limit", 5);
    ini.put("IVMM.candidate_query_radious", 100);
    ini.put("IVMM.project_dist_mean", 5);
    ini.put("IVMM.project_dist_stddev", 10.0);
    ini.put("IVMM.beta", 5000);
    ini.put("IVMMEX.window", 100);
    pt::write_ini(filename, ini);
}

IVMMParam load_ivmm_param(pt::ptree const& ptree){
    IVMMParam ivmm_param;
    ivmm_param.candidate_limit = ptree.get<int>("IVMM.candidate_limit", 5);
    ivmm_param.project_dist_mean = ptree.get<double>("IVMM.project_dist_mean", 40);
    ivmm_param.project_dist_stddev = ptree.get<double>("IVMM.project_dist_stddev", 35.0);
    ivmm_param.candidate_query_radious = ptree.get<double>("IVMM.candidate_query_radious",100);
    ivmm_param.beta = ptree.get<double>("IVMM.beta", 5000);
    return ivmm_param;
}

IVMMParamEX load_ivmm_param_ex( pt::ptree const& ptree){
    IVMMParamEX ivmm_param;
    ivmm_param.candidate_limit = ptree.get<int>("IVMM.candidate_limit", 5);
    ivmm_param.project_dist_mean = ptree.get<double>("IVMM.project_dist_mean", 40);
    ivmm_param.project_dist_stddev = ptree.get<double>("IVMM.project_dist_stddev", 35.0);
    ivmm_param.candidate_query_radious = ptree.get<double>("IVMM.candidate_query_radious",100);
    ivmm_param.beta = ptree.get<double>("IVMM.beta", 5000);
    ivmm_param.window = ptree.get<int>("IVMMEX.window", 100);
    return ivmm_param;
}


vector<GpsPoint> loadGps(fs::path const& gpsFile){
    vector<GpsPoint> gps;
    fs::ifstream ins(gpsFile);
    string line;
    vector<string> meta;
    while ( getline( ins, line) ){
        meta.clear();
        boost::split(meta, line, boost::is_any_of(","));
        double x;// = atof(meta[2].c_str());
        double y;// = atof(meta[3].c_str());
        tm time;
        if ( meta.size() == 9 ){
            x = atof(meta[4].c_str());
            y = atof(meta[5].c_str());
            strptime(meta[3].c_str(), "%Y%m%d%H%M%S", &time);
        }
        else if ( meta.size() == 6 )
        {
            x = atof(meta[4].c_str());
            y = atof(meta[5].c_str());
            strptime(meta[1].c_str(), "%Y-%m-%d %H:%M:%S", &time);
        }else{
            x = atof(meta[2].c_str());
            y = atof(meta[3].c_str());
            strptime(meta[1].c_str(), "%Y-%m-%d %H:%M:%S", &time);
        }
        long timestamp = mktime(&time);
        gps.emplace_back(x, y, timestamp);
    }
    return gps;
}
