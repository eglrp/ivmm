#include  <boost/program_options.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <iostream>
#include  <map>
#include  <shapefil.h>
#include    "network.h"
#include    "io.h"
#include    "key_visitor.hpp"
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;



double getSortIndicator(map<string, int> const& m){
    vector<int> totalCount;
    int sum = 0;
    for(auto& each : m){
        totalCount.push_back(each.second);
        sum += each.second;
    }

    double entropy = 0;
    double stat = 0;
    for(int count : totalCount){
        double p = (double)count / (double)sum;
        entropy += - p * log2(p);
        stat += sqrt((double)count);
    }

    stat *= stat;
    stat /= totalCount.size();
    return entropy * stat;
}
int main(int argc, char *argv[])
{

    int topk;
    fs::path output;
    string road;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("topk,k", po::value(&topk)->required()->default_value(4000), "specify the top k")
        ("output,o", po::value(&output)->required(), "specify the output directory")
        ("road,r", po::value(&road)->required()->default_value("../Date/map/bj-road-epsg3785"));


    po::variables_map vm;

    try{
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if ( vm.count("help") ){
            cout << desc << endl;
            return 0;
        }
        vm.notify();
    }catch(std::exception const& err){
        cerr << err.what() << endl;
        cerr << desc << endl;
        return 1;
    }
    Network network;
    if (! network.load(road) ){
        cerr << " can not load " << road << endl;
        return 1;
    }

    map<string, map<string, int> > crossStat;
    string line;
    if ( ! fs::exists(output) ){
        try{
            fs::create_directories(output);
        }catch(fs::filesystem_error const& err){
            cerr << err.what() << endl;
            return 1;
        }
    }

    while ( getline(cin, line) ){
        RawTraj traj = loadTraj(line);
        for(int i = 0; i < traj.size(); ++i){
            if ( i > 0 && traj[i].crossDBID != traj[i-1].crossDBID){
                ++crossStat[traj[i].crossDBID][traj[i-1].crossDBID];
            }
            if ( i + 1 < traj.size() && traj[i].crossDBID != traj[i+1].crossDBID){
                ++crossStat[traj[i].crossDBID][traj[i+1].crossDBID];
            }
        }
    }

    vector<pair<string, double> > sorted;
    sorted.reserve(crossStat.size());
    for ( auto& p : crossStat ){
        sorted.push_back({p.first, getSortIndicator(p.second)});
    }
    crossStat.clear();
    sort(begin(sorted), end(sorted), binary_of<greater>(&pair<string, double>::second));
    fs::ofstream ofs(output/"cross_stat.txt");
    if ( ! ofs ){
        cerr << "can not create " << output / "cross_stat" << endl;
        return 1;
    }
    for(auto& each : sorted){
        ofs << each.first << "," << each.second << endl;
    }

    if ( sorted.size() > topk ) sorted.resize(topk);
    SHPHandle shp = SHPCreate((output/"cross_stat").c_str(), SHPT_POINT);
    if ( shp == NULL )
    {
        return 1;
    }
    DBFHandle dbf = DBFCreate((output/"cross_stat").c_str());
    if ( dbf == NULL ){
        SHPClose(shp);
        return 1;
    }

    DBFAddField(dbf, "cross", FTString, 11, 0);
    DBFAddField(dbf, "indicator", FTDouble, 10, 6);

    for(auto & p : sorted){
        if (  network.contain_cross(p.first) ){
            Cross const& c = network.cross(p.first);
            double x = c.x;
            double y = c.y;
            SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, NULL);
            int const insert = -1;
            int newID = SHPWriteObject(shp, insert, obj);
            DBFWriteStringAttribute(dbf, newID, 0, p.first.c_str());
            DBFWriteDoubleAttribute(dbf, newID, 1, p.second);
            SHPDestroyObject(obj);
        }
    }
    SHPClose(shp);
    DBFClose(dbf);
    return 0;
}
