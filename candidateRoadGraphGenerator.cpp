#include  <iostream>
#include  <vector>
#include  <unordered_map>
#include  <shapefil.h>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <boost/program_options.hpp>
#include  <boost/range.hpp>
#include  <boost/range/algorithm.hpp>
#include  <boost/algorithm/string.hpp>
#include  <boost/range/algorithm.hpp>

#include    "network.h"
#include    "simple_progress.hpp"
#include    "simple_guard.hpp"
#include    "range_extend.hpp"
#include    "key_visitor.hpp"
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;


struct ForwardEdge
{
    string from;
    string to;
    bool operator==(ForwardEdge const& e)const
    {
        return from == e.from and to == e.to;
    }
};
typedef unordered_map<ForwardEdge, int> ForwardEdgeFrequenceTable;

typedef pair<ForwardEdge, ForwardEdge> CandidateEdge;
namespace std{
    template<>
    struct hash<ForwardEdge>
    {
        size_t operator()( ForwardEdge const& e )const
        {
            hash<string> hasher;
            return hasher(e.from) ^ ( hasher(e.to) << 1 );
        }
    };
    template<>
    struct hash<CandidateEdge>
    {
        size_t operator()(CandidateEdge const& r)const
        {
            hash<ForwardEdge>  hasher;
            return hasher(r.first) ^ ( hasher(r.second) << 1);
        }
    };
    template<>
    struct equal_to<CandidateEdge>
    {
        bool operator()(CandidateEdge const& e1, CandidateEdge const& e2)const
        {
            return e1.first == e2.first && e1.second == e2.second;
        }   
    };
}
typedef pair<long, long> TimePair;
typedef vector<TimePair> TimePairCollections;
typedef unordered_map<CandidateEdge, TimePairCollections> CandidateEdgeTable;
typedef vector<pair<ForwardEdge, TimePair> > RoadTraj;
typedef vector< CandidateEdgeTable::value_type const* > CandidateEdgesVeiw;


void saveHotRoadSegmentToShp(Network const& network ,ForwardEdgeFrequenceTable const& table, fs::path const& outputDir)
{
    SHPHandle shp = SHPCreate((outputDir/"hotRoad").c_str(), SHPT_ARC);
    DBFHandle dbf = DBFCreate((outputDir/"hotRoad").c_str());
    SimpleGuard guard([&](){ SHPClose(shp); DBFClose(dbf); });
    if (shp == nullptr or dbf == nullptr)
    {
        return;
    }
    DBFAddField(dbf, "frequence", FTInteger, 10, 0);
    for(auto& p : table)
    {
        string const& from = p.first.from;
        string const& to = p.first.to;
        adjacent_edge const* edge = network.edge(from, to);
        size_t n = edge->road->points.size();
        double x[n];
        double y[n];
        boost::transform(edge->road->points, x, key_of(&Point::x));
        boost::transform(edge->road->points, y, key_of(&Point::y));
        SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, n, x, y, nullptr);
        int const insert = -1;
        int id = SHPWriteObject(shp, insert, obj);
        DBFWriteIntegerAttribute(dbf, id , 0 , p.second);
        SHPDestroyObject(obj);
    }
}
void saveCandidateGraphEdgeToShp(Network const& network, CandidateEdgesVeiw const& view, fs::path const& outputDir, size_t sigma)
{
   SHPHandle shp = SHPCreate((outputDir/"candiateGraph").c_str(), SHPT_ARC);
   DBFHandle dbf = DBFCreate((outputDir/"candidateGraph").c_str());
   SimpleGuard guard([&](){ SHPClose(shp); DBFClose(dbf); });
   DBFAddField(dbf, "from", FTString, 11, 0);
   DBFAddField(dbf, "to", FTString, 11 , 0);
   DBFAddField(dbf, "frequence", FTInteger, 10, 0);
   for (auto p : view )
   {
       if ( p->second.size() < sigma )
           break;
       auto & f1 = p->first.first;
       auto & f2 = p->first.second;
       adjacent_edge const* r1 = network.edge(f1.from, f1.to);
       adjacent_edge const* r2 = network.edge(f2.from, f2.to);
       CandidatePoint p1 = r1->road->candidate_at_normal(0.5);
       CandidatePoint p2 = r2->road->candidate_at_normal(0.5);
       double x[] = {p1.x, p2.x};
       double y[] = {p1.y, p2.y};
       SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, 2, x, y, nullptr);
       int const insert = -1;
       int id = SHPWriteObject(shp, insert, obj);
       DBFWriteStringAttribute(dbf, id , 0, r1->road->dbId.c_str());
       DBFWriteStringAttribute(dbf, id , 1 , r2->road->dbId.c_str());
       DBFWriteIntegerAttribute(dbf, id , 2, p->second.size());
       SHPDestroyObject(obj);
   }
}
void saveCandidateEdge(Network const& network, CandidateEdgesVeiw const& view, fs::path const& outputDir, size_t sigma)
{
    
    auto hourOfDay = [](long timestamp){
        tm* pt = localtime(&timestamp);
        return pt->tm_hour + (pt->tm_min * 60 + pt->tm_sec) / 3600.0;
    };
    for(auto p : view )
    {
        auto &timePairCollections = p->second;
        auto & edge = p->first;
        if ( timePairCollections.size() < sigma )
            break;
        auto e1 = network.edge(edge.first.from, edge.first.to);
        auto e2 = network.edge(edge.second.from, edge.second.to);
        fs::path filename = outputDir/(e1->road->dbId + "-" + e2->road->dbId + ".txt");
        fs::ofstream outs(filename);
        if (not outs)
        {
            continue;
        }

        for(auto timePair : timePairCollections)
        {
            outs << timePair.first  << "," << timePair.second << "," 
                << hourOfDay(timePair.first) << "," << hourOfDay(timePair.second) << ","
                << timePair.second - timePair.first << "\n";
        }
        outs.close();
    }
}

int main(int argc, char *argv[])
{
    fs::path outputDir;
    fs::path roadPath;
    fs::path forwardEdgePath;
    fs::path edgeOutputDir;
    size_t topk;
    size_t sigma;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("output,o", po::value(&outputDir)->required(), "special output directory")
        ("road,r", po::value(&roadPath)->required(), "special the road shp file")
        ("forwardEdge,f", po::value(&forwardEdgePath)->required(), "special the forward edge txt file")
        ("topk,t", po::value(&topk)->default_value(1000), "special the topk")
        ("sigma,s",po::value(&sigma)->default_value(300), "the edge frequence > sigma will be the candidate edge");

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        vm.notify();
    }catch(po::error const& e)
    {
        if ( vm.count("help")  )
        {
            cout << desc << endl;
            return 0;
        }
        cerr << e.what() << endl;
        return 1;
    }
    if ( not fs::exists(outputDir) )
    {
        try
        {
            fs::create_directories(outputDir);
        }
        catch(fs::filesystem_error const & e)
        {
            cerr << e.what() <<endl;
            return 1;
        }
    }
    edgeOutputDir = outputDir/"edge";
    if ( not fs::exists(edgeOutputDir) )
    {
        try
        {
            fs::create_directories(edgeOutputDir);
        }
        catch(fs::filesystem_error const & e)
        {
            cerr << e.what() <<endl;
            return 1;
        }
    }

    
    Network network;
    if ( not network.load(roadPath.string()) )
    {
        cerr << "can not load road network : " << roadPath << endl;
        return 1;
    }
    cout << "load network success." << endl;

    fs::ifstream forwardEdgeIns(forwardEdgePath);
    if ( not forwardEdgeIns )
    {
        cerr << "can not open " << forwardEdgePath << endl;
        return 1;
    }

    string line;
    vector<string> sp;
    ForwardEdgeFrequenceTable hotForwardEdge;
    for(size_t i = 0;i < topk && getline(forwardEdgeIns, line); ++i )
    {
        boost::split(sp, line, boost::is_any_of(","));
        hotForwardEdge[ { std::move(sp[0]),std::move(sp[1])} ] = atoi(sp[2].c_str());
    }
    cout << "load forward edge frequence table success." << endl;

    list<string> lines;
    while(getline(cin, line))
    {
        lines.push_back(std::move(line));
    }

    SimpleProgress trajProgress(lines.size());
    CandidateEdgeTable candidateEdgeTable;
    cout << "count candidate edge frequcne ." << endl;
    while(not lines.empty() )
    {
        SimpleGuard guard([&trajProgress](){ ++trajProgress; });
        line = std::move(lines.front());
        lines.pop_front();
        ifstream trajIns(line);
        if ( not trajIns )
        {
            cerr << "can not open " << line << endl;
            continue;
        }
        vector<pair<string, long> > traj;
        while( getline(trajIns, line) )
        {
            boost::split(sp, line, boost::is_any_of(","));
            traj.push_back( { std::move(sp.at(0)), atol( sp.at(2).c_str() ) } );
        }
        RoadTraj roadTraj;
        boost::copy_adjacent(traj, back_inserter(roadTraj), [](pair<string ,long> const& t1, pair<string, long> const& t2){
                ForwardEdge e{ t1.first, t2.first };
               return make_pair(std::move(e), make_pair(t1.second, t2.second) );
                });
        RoadTraj::iterator lastHotRoadIter = roadTraj.end();
        for(auto it = roadTraj.begin(); it != roadTraj.end(); ++it)
        {
            if( hotForwardEdge.count(it->first) )
            {
                if ( lastHotRoadIter != roadTraj.end() )
                {
                    candidateEdgeTable[{lastHotRoadIter->first, it->first}]
                        .push_back({lastHotRoadIter->second.first, it->second.second});
                }
                lastHotRoadIter = it;
            }
        }
    }
    trajProgress.done();
    CandidateEdgesVeiw view;
    view.reserve(candidateEdgeTable.size());
    cout << "transofrm" << endl;
    boost::transform(candidateEdgeTable, back_inserter(view), [](CandidateEdgeTable::value_type const& v){ return &v; });
    cout << "sort" << endl;
    boost::sort(view, [](CandidateEdgesVeiw::value_type const& a, CandidateEdgesVeiw::value_type const& b)
            {
                return a->second.size() > b->second.size();
            });
    cout << "save hot roadsegment to shp" << endl;
    saveHotRoadSegmentToShp(network, hotForwardEdge, outputDir);
    cout << "save candidate graph edge to shp " << endl;
    saveCandidateGraphEdgeToShp(network, view, outputDir, sigma);
    cout << "save candidate edge " << endl;
    saveCandidateEdge(network, view, edgeOutputDir, sigma);
}
