//auto-include {{{
#include <map>
#include <iostream>
#include <string>
#include  <boost/program_options.hpp>
#include  <boost/date_time.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include "network.h"
#include "io.h"
#include    "pathendpointwalker.hpp"
//}}}
using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
int main(int argc, char *argv[])
{

    Network network;
    string line;
    string road;
    fs::path output;

    po::options_description desc(argv[0]);
    desc.add_options()
        ("road,r", po::value(&road)->default_value("../Date/map/bj-road-epsg3785"), "specify the road shp file")
        ("output,o", po::value(&output)->default_value("time-slot"), "specify the time collections output directories")
        ("help,h", "show help");
    po::variables_map vm;

    try{
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if ( vm.count("help") ){
            cout << desc << endl;
            return 1;
        }
        vm.notify();
    }catch(std::exception const& e){
        cerr << e.what() << endl;
        return 1;
    }

    if (! network.load(road) ){
        cerr << "can not load shp file"<< endl;
        return 1;
    }
    cout << "load shp file." << endl;
    if ( ! fs::exists(output) ){
        try{
            fs::create_directories(output);
        }catch( fs::filesystem_error const& err ){
            cerr << err.what() << endl;
            return 1;
        }
    }

    map<pair<string, string> , map<int, vector<long> > >  timeCollections;
    while ( getline(cin, line) ){
        RawTraj traj = loadTraj(line);
        if ( traj.empty()) 
            continue;
        Path path = trajToPath(network, traj);
        path.estimate_time_at_cross();
        PathEndpointWalker<Path> walker1(path), walker2(path);
        ++walker2;
        while ( ! walker2.isEnd() ){
            PathPoint p1 = walker1.pathpoint();
            PathPoint p2 = walker2.pathpoint();
            adjacent_edge const* edge = network.edge(p1.cid, p2.cid);
            if ( edge ){
                Cross const& from = network.cross(edge->begin);
                Cross const& to = network.cross(edge->end);
                ptime time1 = from_time_t(p1.timestamp);
                ptime time2 = from_time_t(p2.timestamp);
                int slot = time1.time_of_day().total_seconds() / (60 * 30);
                int cost = (time2-time1).total_seconds();
                if ( cost != 0 ) timeCollections[{from.dbId,to.dbId}][slot].push_back(cost);
            }
            ++walker1;
            ++walker2;
        }
        cout << "collect:" << line << "\n";
    }
    cout << "dumping..." << endl;
    fs::ofstream outs(output/"all-edge.txt");
    for ( auto& each : timeCollections ){
        auto& edge = each.first;
        string head = edge.first + "-" + edge.second + ".txt";
        outs <<  head << "\n";
        if ( outs ){
            for(auto& eachSlot : each.second){
                int slotId = eachSlot.first;
                auto& times = eachSlot.second;
                outs << slotId << ":";
                for(int cost : times){
                    outs << cost << " ";
                }
                outs << "\n";
            }
        }
    }
    return 0;
}
