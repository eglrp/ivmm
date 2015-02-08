#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <boost/program_options.hpp>
#include  <boost/range.hpp>
#include  <boost/range/algorithm.hpp>
#include  <boost/archive/xml_iarchive.hpp>
#include  <boost/archive/xml_oarchive.hpp>
#include  <boost/archive/text_iarchive.hpp>
#include  <boost/archive/text_oarchive.hpp>
#include  <boost/archive/binary_iarchive.hpp>
#include  <boost/archive/binary_oarchive.hpp>
#include  <boost/serialization/map.hpp>
#include  <boost/serialization/vector.hpp>
#include  <boost/serialization/set.hpp>
#include  <stack>
#include  <iostream>
#include  <boost/timer.hpp>
#include  <shapefil.h>

#include    "network.h"


namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ar = boost::archive;
using namespace std;

struct TrajPathNode
{
    string cur;
    TrajPathNode* pre = nullptr;
    map<string, TrajPathNode*> childs;

    template<class Archive>
    void serialize(Archive& ar, unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(cur);
        ar & BOOST_SERIALIZATION_NVP(pre);
        ar & BOOST_SERIALIZATION_NVP(childs);
    }
};
class TrajPathTree
{
public:
    TrajPathNode* insert(fs::path const& path) 
    {
        TrajPathNode* root = & root_;

        for(fs::path item : path)
        {
            string sitem = item.string();
            if ( root->childs.count(sitem) == 0 )
            {
                TrajPathNode* node = new TrajPathNode;
                root->childs.insert( { sitem, node } );
                node->cur = sitem;
                node->pre = root;
            }
            root = root->childs[sitem];
        }
        return root == &root_ ? nullptr : root;
    }
    string find(TrajPathNode const* node){
        fs::path path;
        stack<string const*> stk;
        while ( node != &root_ ){
            stk.push( &node->cur );
            node = node->pre;
        }
        while ( !stk.empty() ){
            path /= * stk.top();
            stk.pop();
        }
        return path.string();
    }

    template<class Archive>
    void serialize(Archive& ar, unsigned int const version)
    {
        ar & BOOST_SERIALIZATION_NVP(root_);
    }
private:
    TrajPathNode root_;
};
/*
struct SuffixTrajNode{
    vector< pair <TrajPathNode*, long> > trajs;
    map<int, SuffixTrajNode*> childs;
    template<class Archive>
    void serialize(Archive& ar, unsigned int const version)
    {
        ar & BOOST_SERIALIZATION_NVP(trajs);
        ar & BOOST_SERIALIZATION_NVP(childs);
    }
}; 
class SimpleSuffixTrajTree
{
public:
    void insertTraj(vector<pair<int, long> > const& traj, TrajPathNode* path){
        for(auto it = traj.begin(); it != traj.end() - 2; ++it){
            insertSuffix(it, traj.end(), path);
        }
    }
    
    void insertSuffix( vector<pair<int, long> >::const_iterator begin, vector<pair<int, long> >::const_iterator end, 
            TrajPathNode* path){
        SuffixTrajNode* root = & root_;
        int beginTimeStamp = begin->second;
        for( ; begin != end; ++begin ){
            int crossID = begin->first;
            if ( root->childs.count(crossID)  == 0){
                root->childs.insert({ crossID, new SuffixTrajNode });
            }
            root = root->childs[crossID];
        }
        root->trajs.push_back({path, beginTimeStamp});
    }
    template<class Archive>
    void serialize(Archive & ar, unsigned int const version)
    {
        ar & BOOST_SERIALIZATION_NVP(root_);
    }
    vector<TrajPathNode const*> find(int begin, int end)const{
        if (  root_.childs.count(begin) == 0 ) return {};
        SuffixTrajNode const* node = find(root_.childs.at(begin), end);
    }
private:
    void find(SuffixTrajNode const* node, int cross, vector<SuffixTrajNode const*>& results)const{
        if ( node == nullptr ) return;
        if ( node->childs.count(cross) ){
        }

    }

    SuffixTrajNode root_;
};*/


void generateOutputFile(string const& roadshp, fs::path const& crossFrequence, int topk, int use, fs::path const& outputFile);
void runServer(fs::path const& file);

int main(int argc, char *argv[])
{
    bool generate;
    fs::path crossFrequenceFile;
    fs::path file;
    string roadShp;
    int topk;
    int use;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("generate,g","generate server data")
        ("cross_frequence,c", po::value<fs::path>(&crossFrequenceFile), "specify the cross frequence table")
        ("road,r", po::value<string>(&roadShp), "generate need the road shp ")
        ("topk,k", po::value<int>(&topk)->required()->default_value(4000), "use the topk k to generator hot cross")
        ("use,u", po::value<int>(&use)->default_value(50), "generate how manay random node")
        ("file,f", po::value<fs::path>(&file)->default_value("default_server_data.bin"), "use the file init the server or the output of generated data")
        ("help,h", "show help");

    po::variables_map vm;
    try{
        po::store(po::parse_command_line(argc, argv, desc), vm);
    }catch(std::exception const& err){
        cerr << err.what() << endl;
        cout << desc << endl;
        return 1;
    }

    try{
        vm.notify();
    }catch(std::exception const& err){
        cerr << err.what() << endl;
        cout << desc << endl;
        return 1;
    }
    if ( vm.count("help") ){
        cout << desc << endl;
        return 1;
    }

    generate = vm.count("generate") == 1;
    if( generate  ){
        if (vm.count("cross_frequence") == 0){
            cerr << "need special cross frequence file" << endl;
            return 1;
        }
        if ( vm.count("road") == 0 ){
            cerr << "when generate need the road shp file" << endl;
            return 1;
        }
        generateOutputFile(roadShp, crossFrequenceFile, topk, use, file);
    }else{
        runServer(file);
    }
    return 0;
}

void dumpCross(string const& roadshp, vector<string> const& hotCross){
    Network network;
    if ( network.load(roadshp) == false ){
        cerr << "can not load network " << roadshp << endl;
        exit(EXIT_FAILURE);
    }
    SHPHandle shp = SHPCreate("selected_cross", SHPT_POINT);
    DBFHandle dbf = DBFCreate("selected_cross");
    DBFAddField(dbf, "cross", FTString, 11, 0);
    for(string const& s : hotCross){
        Cross const& c = network.cross(s);
        double x = c.x;
        double y = c.y;
        SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
        int id = SHPWriteObject(shp, -1, obj);
        DBFWriteStringAttribute(dbf, id, 0 , s.c_str());
    }
    SHPClose(shp);
    DBFClose(dbf);
}


void generateOutputFile(string const& roadshp, fs::path const& crossFrequence, int topk, int use, fs::path const& outputFile){
    boost::timer timer;
    TrajPathTree pathTree;
    map<string, int> hotCrossIndexMap;
    vector<string> hotCross;
    fs::ifstream ifs(crossFrequence);
    if ( ! ifs  ) {
        cerr << "can not open " << crossFrequence << endl;
        exit(EXIT_FAILURE);
    }

    hotCross.reserve(topk);
    string cross;
    while( topk-- && getline(ifs, cross, ',') && ifs.ignore(numeric_limits<streamsize>::max() , '\n' )){
        hotCross.push_back(std::move(cross));
    }
    random_shuffle(hotCross.begin(), hotCross.end());
    hotCross.resize(use);
    for(int i = 0; i < (int)hotCross.size(); ++i){
        hotCrossIndexMap.insert({ hotCross[i], i });
    }
    dumpCross(roadshp, hotCross);

    map<TrajPathNode*, map<int, long> > trajCrossMap;
    map<int, set<TrajPathNode *> > crossTrajMap;

    string line;
    while( getline(cin, line) ){
        fs::path filePath(line);
        fs::ifstream ifs(filePath);
        if ( ifs ){
            vector<pair<int, long> > traj;
            long timestamp;
            while( getline(ifs,cross, ',') &&  ifs.ignore(numeric_limits<streamsize>::max(), ',') && ifs >> timestamp && ifs.ignore()){
                if ( hotCrossIndexMap.count(cross) ){
                    traj.push_back( { hotCrossIndexMap[cross], timestamp } );
                }
            }
            if ( traj.size() >= 2 ){
                TrajPathNode* node = pathTree.insert(filePath);
                cout << filePath << " insert" << endl;
                for(auto p : traj ){
                    trajCrossMap[node][p.first] = p.second;
                    crossTrajMap[p.first].insert(node);
                }
            }
        }
    }

    fs::ofstream ofs(outputFile);
    cout << "output to file..." << endl;
    ar::binary_oarchive oar(ofs);
    oar << BOOST_SERIALIZATION_NVP(hotCross);
    oar << BOOST_SERIALIZATION_NVP(hotCrossIndexMap);
    oar << BOOST_SERIALIZATION_NVP(pathTree);
    oar << BOOST_SERIALIZATION_NVP(trajCrossMap);
    oar << BOOST_SERIALIZATION_NVP(crossTrajMap);
    cout << "cost : " << timer.elapsed() << "sec " << endl;
}


void runServer(fs::path const& file){
    vector<string> hotCross;
    map<string, int> hotCrossIndexMap;
    TrajPathTree pathTree;
    map<TrajPathNode*, map<int, long> > trajCrossMap;
    map<int, set<TrajPathNode *> > crossTrajMap;
    fs::ifstream ifs(file);
    if ( ! ifs ){
        cerr << "can not open " << file << endl;
        exit(EXIT_FAILURE);
    }

    ar::binary_iarchive iar(ifs);
    iar >> BOOST_SERIALIZATION_NVP(hotCross);
    iar >> BOOST_SERIALIZATION_NVP(hotCrossIndexMap);
    iar >> BOOST_SERIALIZATION_NVP(pathTree);
    iar >> BOOST_SERIALIZATION_NVP(trajCrossMap);
    iar >> BOOST_SERIALIZATION_NVP(crossTrajMap);
    string begin, end;
    int ibegin, iend;
    while( cin >> begin >> end ){
        if ( hotCrossIndexMap.count(begin) && hotCrossIndexMap.count(end) ){
            ibegin = hotCrossIndexMap[begin];
            iend = hotCrossIndexMap[end];
            vector<TrajPathNode*> commonTraj;
            commonTraj.reserve(min(crossTrajMap[ibegin].size(),crossTrajMap[iend].size()));
            boost::set_intersection(crossTrajMap[ibegin], crossTrajMap[iend], back_inserter(commonTraj));
            for ( TrajPathNode *  node : commonTraj){
                long tbegin = trajCrossMap[node][ibegin];
                long tend = trajCrossMap[node][iend];
                assert(tbegin != 0 && tend != 0);
                if ( tend - tbegin > 0 ){
                    cout << pathTree.find(node) << endl;
                }
            }
        }
    }
}
