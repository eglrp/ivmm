#include  <iostream>
#include  <string>
#include  <map>
#include  <shapefil.h>
#include  <memory>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <boost/program_options.hpp>
#include    "network.h"
#include    "simple_guard.hpp"
#include    "transitionGraph.h"

typedef std::map<std::string, int> CrossMap;
typedef std::map<std::pair<std::string, std::string>, int> ForwardEdgeMap;
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

void parseCommandLine(int argc, char const* const* argv, po::options_description& desc)
{
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc),vm);
    }catch(std::exception const& e)
    {
        cerr << e.what() << endl;
        exit(EXIT_FAILURE);
    }
    if ( vm.count("help") )
    {
        cout << desc << endl;
        exit(EXIT_SUCCESS);
    }
    try
    {
        vm.notify();
    }catch(std::exception const& e)
    {
        cerr << e.what() << endl;
        exit(EXIT_FAILURE);
    }
}

void generatorMap(fs::path const& outputDir, Network const& network, CrossMap const& cmap, ForwardEdgeMap const& emap)
{
    SHPHandle shp = SHPCreate((outputDir/"cpnet").c_str(), SHPT_ARC);
    SimpleGuard shpG([&](){ if (shp) SHPClose(shp); });
    DBFHandle dbf = DBFCreate((outputDir/"cpnet").c_str());
    SimpleGuard dbfG([&](){ if (dbf) DBFClose(dbf);});
    DBFAddField(dbf, "edge", FTString, 30,0);
    DBFAddField(dbf, "pro", FTDouble, 10, 8);

    auto as_c_str = [](pair<string, string> const& e){
        static string buf;
        buf = e.first+","+e.second;
        return buf.c_str();
    };
    for(auto& p : emap)
    {
       auto const& edge =  p.first;
       int feq = p.second;
       double pro = 0.0;
       try
       {
            pro = (double)feq / cmap.at(edge.first);
       }catch(std::out_of_range const& e)
       {
           cerr << e.what() << endl;
           cerr << edge.first << " not in cmap " << endl;
           exit(EXIT_FAILURE);
       }
       Cross  const& c1 = network.cross(edge.first);
       Cross  const& c2 = network.cross(edge.second);
       double x[2] { c1.x, c2.x };
       double y[2] { c1.y, c2.y };
       SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
       int insertPos = -1;
       int newID = SHPWriteObject(shp, insertPos, obj);
       DBFWriteStringAttribute(dbf,newID, 0, as_c_str(edge));
       DBFWriteDoubleAttribute(dbf, newID, 1, pro);
       SHPDestroyObject(obj);
    }
}

int main(int argc, char *argv[])
{
    fs::path crossFrequenceTxT;
    fs::path forwardFrequenceTxT;
    fs::path outputDir;
    string road;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("road,r", po::value(&road)->required(), "specify the road shp file")
        ("cross,c", po::value(&crossFrequenceTxT)->required(), "specify the cross frequence table")
        ("forward,f", po::value(&forwardFrequenceTxT)->required(), "specify the forward edge table")
        ("output,o", po::value(&outputDir)->required(), "specify the output directory");

    parseCommandLine(argc, argv, desc);
    
    //if (!fs::exists(outputDir)) fs::create_directories(outputDir);
    Network network;
    if ( ! network.load(road) )
    {
        cerr << "can not load road shp file" << endl;
        return 1;
    }

    TransitionGraph tg(network);
    if ( ! tg.load(crossFrequenceTxT.string(), forwardFrequenceTxT.string()) )
    {
        cerr << "load cross, forward edge data fail " << endl;
        return 1;
    }
    if (! tg.dump(outputDir.string()))
    {
        cerr << "dump graph fail." << endl;
        return 1;
    }
    /*
    CrossMap cmap;
    fs::ifstream crossIns(crossFrequenceTxT);
    if ( ! crossIns )
    {
        cerr << "can not open " << crossFrequenceTxT << endl;
        return 1;
    }
    string crossID;
    int freq;
    while (getline(crossIns, crossID, ',') && crossIns >> freq && crossIns.ignore())
        cmap[std::move(crossID)] = freq;
    crossIns.close();

    ForwardEdgeMap edgeMap;
    fs::ifstream forwardEdgeIns(forwardFrequenceTxT);
    if ( ! forwardEdgeIns  )
    {
        cerr << "can not open " << forwardFrequenceTxT << endl;
        return 1;
    }
    string crossID1, crossID2;
    while(getline(forwardEdgeIns, crossID1, ',') && getline(forwardEdgeIns, crossID2, ',') && 
            forwardEdgeIns>>freq && forwardEdgeIns.ignore())
        edgeMap[{std::move(crossID1),std::move(crossID2)}] = freq;
    forwardEdgeIns.close();
    generatorMap(outputDir, network, cmap, edgeMap);*/
    return 0;
}
