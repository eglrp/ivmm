#include  <iostream>
#include  <unordered_set>
#include  <unordered_map>
#include  <string>
#include  <shapefil.h>
#include  <boost/range.hpp>
#include  <boost/range/algorithm.hpp>
#include  <boost/program_options.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <boost/algorithm/string.hpp>

#include    "network.h"
#include    "simple_guard.hpp"
#include    "simple_progress.hpp"
using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef unordered_map<string, int> HotCrossSet;
typedef pair<string, string> Edge;

template<typename EDGE>
auto cross_from(EDGE&& edge)->decltype((edge.first))
{
    return edge.first;
}
template<typename EDGE>
auto cross_to(EDGE&& edge)->decltype((edge.second))
{
    return edge.second;
}

typedef unordered_map<Edge, vector<pair<long, long> > > EdgeFrequenceMap;
namespace std
{
    template<>
    struct hash<Edge>
    {
        size_t operator()(Edge const & e)const
        { 
            hash<string> hasher;
            return hasher(cross_from(e)) ^ ( hasher(cross_to(e)) << 1);
        }    
    };
}

struct Record
{
    string crossDBID;
    long timestamp;
};



void 
saveCandidateEdge( vector<EdgeFrequenceMap::const_pointer> const& vec, fs::path const& outputDir, size_t sigma )
{
    string filename;
    auto hourOfDay = [](long timestamp)
    {
        tm const* atm = localtime(&timestamp);
        return atm->tm_hour + (atm->tm_min * 60.0 + atm->tm_sec) / 3600.0;
    };
    for ( auto & p : vec  )
    {
        auto& edgeCrossPair = p->first;
        auto& timeColloection = p->second;
        if ( timeColloection.size() < sigma ) break;
        filename = cross_from(edgeCrossPair)+ "-" + cross_to(edgeCrossPair) + ".txt";
        fs::ofstream o(outputDir / filename);
        for (auto& timePair : timeColloection )
        {
            o << timePair.first << "," << timePair.second << ","<< 
                hourOfDay(timePair.first) << "," << hourOfDay(timePair.second) << ","
                << timePair.second - timePair.first << "\n";
        }
        o.close();
    }
}

void
generatorSHPFile(Network const& network, HotCrossSet const& hotSet, vector<EdgeFrequenceMap::const_pointer> const& vec, 
        fs::path const& outputDir, size_t sigma)
{
    SHPHandle shp = SHPCreate((outputDir/"hotCross").c_str(), SHPT_POINT);
    DBFHandle dbf = DBFCreate((outputDir/"hotCross").c_str());
    SimpleGuard guard([&](){ SHPClose(shp);DBFClose(dbf); });
    if ( shp == nullptr or dbf == nullptr )
    {
        cerr << "can not create shp file " << endl;
        return;
    }
    DBFAddField(dbf, "crossDBID", FTString, 11, 0);
    DBFAddField(dbf, "frequence", FTInteger, 10,0);
    for( auto& p: hotSet )
    {
        Cross c = network.cross(p.first);
        SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, & c.x,  &c.y, nullptr);
        int const insert = -1;
        int id = SHPWriteObject(shp, insert, obj );
        DBFWriteStringAttribute(dbf, id, 0, p.first.c_str());
        DBFWriteIntegerAttribute(dbf, id , 1, p.second);
        SHPDestroyObject(obj);
    }

    SHPHandle edgeShp = SHPCreate((outputDir/"edge").c_str(), SHPT_ARC);
    DBFHandle edgeDbf = DBFCreate((outputDir/"edge").c_str());
    SimpleGuard edgeGuard([&](){ SHPClose(edgeShp); DBFClose(edgeDbf); });
    if ( edgeShp == nullptr or edgeDbf == nullptr )
    {
        cerr << "can not create edge shp file " << endl;
        return ;
    }
    DBFAddField(edgeDbf, "from", FTString, 11, 0);
    DBFAddField(edgeDbf, "to", FTString, 11, 0);
    DBFAddField(edgeDbf, "frequcne", FTInteger, 10, 0);
    for( auto p : vec  )
    {
        Edge const& edge = p->first;
        string const& from = cross_from(edge);
        string const& to = cross_to(edge);
        size_t freq = p->second.size();
        if ( freq < sigma )
            break;
        Cross const& cfrom = network.cross(from);
        Cross const& cto = network.cross(to);
        double x[2] = {cfrom.x, cto.x};
        double y[2] = {cfrom.y, cto.y};
        SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
        int const insert = -1;
        int id = SHPWriteObject(edgeShp, insert, obj);
        DBFWriteStringAttribute(edgeDbf, id, 0, from.c_str());
        DBFWriteStringAttribute(edgeDbf, id , 1, to.c_str());
        DBFWriteIntegerAttribute(edgeDbf, id, 2, freq);
        SHPDestroyObject(obj);
    }
}

int main(int argc, char *argv[])
{
    fs::path outputDir;
    fs::path roadPath;
    fs::path crossFreqPath;
    size_t sigma;
    size_t topk;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("road,r", po::value(&roadPath)->required(), "specify the road shpfile")
        ("crossFreq,c", po::value(&crossFreqPath)->required(), "specify the cross frequence txt file")
        ("topk,t", po::value(&topk)->default_value(1000), "specify the topk cross as candidate cross")
        ("sigma,s",po::value(&sigma)->default_value(300), "specify the sigma, the edge frequence >= sigma will be a candidate edge")
        ("output,o", po::value(&outputDir)->required(), "specify the output directory");
    po::variables_map vm;
    try
    { 
        po::store(po::parse_command_line(argc, argv, desc), vm);
        vm.notify();
    }catch ( exception const& e )
    {
        if ( vm.count("help") )
        {
            cout << desc << endl;
            return 0;
        }
        cerr << e.what() << endl;
        return 1;
    }

    if ( vm.count("help") )
    {
        cout << desc << endl;
        return 0;
    }

    if ( not fs::exists(outputDir) )
    {
        try
        {
            fs::create_directories(outputDir);
        }catch(fs::filesystem_error const& e)
        {
            cerr << e.what() << endl;
            return 1;
        }
    }

    fs::path timeColloectionOutputDir = outputDir/"time";
    if ( not fs::exists(timeColloectionOutputDir) )
    {
        try
        {
            fs::create_directories(timeColloectionOutputDir);
        }catch(fs::filesystem_error const& e)
        {
            cerr << e.what() << endl;
            return 1;
        }
    }


    Network network;
    if ( not network.load( roadPath.string() ) )
    {
        cerr << "can not load " << roadPath << endl;
        return 1;
    }
    cout << "load network success!" << endl;

    HotCrossSet hotCross;
    fs::ifstream crossFreqInStream(crossFreqPath);
    if ( not crossFreqInStream )
    {
        cerr << "can not open " << crossFreqPath << endl;
        return 1;
    }

    string line;
    size_t count = 0;
    vector<string> sp;
    while( getline(crossFreqInStream, line) && count < topk )
    {
        boost::split(sp, line, boost::is_any_of(","));
        hotCross.insert( { std::move(sp[0]), atoi(sp[1].c_str()) } );
        ++count;
    }
    cout << "load hot cross " << endl;

    list<string> lines;
    while ( getline(cin, line) )
    {
        lines.push_back(std::move(line));
    }

    SimpleProgress trajReadProgress(lines.size());

    EdgeFrequenceMap edgeFreq;
    vector<Record> records;
    cout << "generator edge " << endl;
    while ( not lines.empty() )
    {
        SimpleGuard guard([&trajReadProgress](){ ++trajReadProgress; });
        line = std::move(lines.front());
        lines.pop_front();
        fs::ifstream trajInStream(line);
        if ( not trajInStream  )
        {
            continue;
        }
        records.clear();
        while ( getline(trajInStream, line) )
        {
            boost::split(sp, line, boost::is_any_of(",")); 
            Record r;
            r.crossDBID = std::move(sp.at(0));
            r.timestamp = atol(sp.at(2).c_str());
            records.push_back(std::move(r));
        }
        vector<Record>::iterator lastHotCrossIter = records.end();
        for(auto it = records.begin(); it != records.end(); ++it)
        {
            if ( hotCross.count(it->crossDBID) )
            {
                if ( lastHotCrossIter != records.end()  )
                { 
                    edgeFreq[ {lastHotCrossIter->crossDBID, it->crossDBID} ]
                        .push_back( { lastHotCrossIter->timestamp, it->timestamp } );
                }
                lastHotCrossIter = it;

            }
        }
    }
    trajReadProgress.done();
    vector < EdgeFrequenceMap::const_pointer > freqSorterVec;
    boost::transform(edgeFreq, back_inserter(freqSorterVec), []( EdgeFrequenceMap::const_reference p ){ return &p; });
    boost::sort(freqSorterVec, [](EdgeFrequenceMap::const_pointer a, EdgeFrequenceMap::const_pointer b)
    {
        return a->second.size() > b->second.size();
    });
    cout << "save candiate edge " << endl;
    cout << "generator shp file " << endl;
    saveCandidateEdge( freqSorterVec, timeColloectionOutputDir, sigma );
    generatorSHPFile(network, hotCross, freqSorterVec, outputDir, sigma);
    return 0;
}
