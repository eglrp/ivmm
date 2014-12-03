#include    "transitionGraph.h"

#include  <shapefil.h>
#include  <fstream>
#include  <boost/filesystem.hpp>
#include  <iostream>
#include  <boost/range/algorithm.hpp>
#include    "simple_guard.hpp"
#include    "key_visitor.hpp"
#include  <queue>
using namespace std;
namespace fs = boost::filesystem;

TransitionGraph::TransitionGraph(Network& network)
    :network_(network)
{
    crossInfo_.resize(network.cross_bound());
    for(int i = 0; i < network.cross_bound(); ++i)
        for(adjacent_edge & edge : network.adjacent_table().at(i))
        {
            crossInfo_[i].id = i;
            crossInfo_[i].adjacent.push_back({&edge, 0.0, 0});
            crossInfo_[i].outs.push_back(&crossInfo_[i].adjacent.back());
        }

    for(CrossInfo & cinfo : crossInfo_)
        for(Transition & t : cinfo.adjacent )
            crossInfo_[t.source->end].ins.push_back(&t);
}


bool TransitionGraph::load(std::string const& crossFile, std::string const& forwardFile)
{
    ifstream crossFrequenceIns(crossFile);
    if ( ! crossFrequenceIns )
        return false;

    ifstream forwardEdgeFrequenceIns(forwardFile);

    if ( ! forwardEdgeFrequenceIns )
        return false;

    string crossID;
    int freq;

    while( getline(crossFrequenceIns, crossID, ',') && crossFrequenceIns >> freq && crossFrequenceIns.ignore() )
    {
       if (! network_.contain_cross(crossID))
           continue;
       int cidx = network_.cross(crossID).id;
       crossInfo_[cidx].freq = freq;
    }

    string crossID2;
    while( getline(forwardEdgeFrequenceIns , crossID, ',') && getline(forwardEdgeFrequenceIns, crossID2, ',')
        && forwardEdgeFrequenceIns >> freq && forwardEdgeFrequenceIns.ignore() )
    {
        if ( ! network_.contain_cross(crossID) || ! network_.contain_cross(crossID2) )
            continue;

        int cidx1 = network_.cross(crossID).id;
        int cidx2 = network_.cross(crossID2).id;
        Transition* t = const_cast<Transition*>(transition(cidx1, cidx2));
        t->freq = freq;
        t->p = (double)freq / (double)crossInfo_[t->source->begin].freq;
    }
    return true;
}

bool TransitionGraph::dump(std::string const& p)const
{
    fs::path path(p);
    if ( path.has_parent_path() ){
        fs::path parent_path = path.parent_path();
        if ( ! fs::exists(parent_path) ){
            try{
                fs::create_directories(parent_path);
            }catch(fs::filesystem_error const& err){
                cerr << err.what() << endl;
                return false;
            }
        }
    }

    SHPHandle shp = SHPCreate(p.c_str(), SHPT_ARC);
    SimpleGuard shpG([&](){ if (shp) SHPClose(shp); });
    DBFHandle dbf = DBFCreate(p.c_str());
    SimpleGuard dbfG([&](){ if (dbf) DBFClose(dbf);});
    DBFAddField(dbf, "edge", FTString, 30,0);
    DBFAddField(dbf, "pro", FTDouble, 10, 8);

    for(CrossInfo const & info : crossInfo_)
    {
        for ( Transition const & t : info.adjacent )
        {
            size_t pointSZ = t.source->road->points.size();
            double x[pointSZ];
            double y[pointSZ];
            boost::transform(t.source->road->points, x, key_of(&Point::x));
            boost::transform(t.source->road->points, y, key_of(&Point::y));
            SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, pointSZ, x, y, nullptr);
            const int insert = -1;
            int nShp = SHPWriteObject(shp, insert, obj);
            SHPDestroyObject(obj);
            DBFWriteStringAttribute(dbf, nShp, 0, 
                    (network_.cross(t.source->begin).dbId+","+network_.cross(t.source->end).dbId).c_str()
                    );
            DBFWriteDoubleAttribute(dbf, nShp, 1, t.p);
        }
    }
    return true;
}



struct TNode
{
    TNode* parent;
    Transition const* t;
    double accP;
    int cID;
};
struct TNodeAlloc
{
    TNode* alloc(TNode* parent, Transition const* t, double accP, int cId)
    {
        TNode node = {parent , t, accP, cId};
        buf.push_back(node);
        return &buf.back();
    }
    vector<TNode> buf;
};
namespace std
{
    template<>
    struct less<TNode*>
    {
        bool operator()(TNode const* a, TNode const* b)const{
            return a->accP < b->accP;
        }
    };
}
vector<Transition const*> TransitionGraph::max_probabiliy_sequence(int begin, int end)const
{
    priority_queue<TNode*> q;
    TNodeAlloc nodeBuf;
    unordered_set<int> closed;
    q.push(nodeBuf.alloc(nullptr, nullptr, 1.0, begin));
    vector<Transition const*> result;
    if ( ! network_.contain_cross(begin) || !network_.contain_cross(end) )
        return result;
    while( ! q.empty() )
    {
        TNode* top = q.top();
        q.pop();
        if ( top->cID == end )
        {
            while ( top->t)
            {
                result.push_back(top->t);
                top = top->parent;
            }
            boost::reverse(result);
            return result;
        }

        if ( closed.count(top->cID) )
            continue;

        for (Transition const* t : crossInfo_[top->cID].outs)
        {
            if ( closed.count(t->source->end) ) continue;
            q.push(nodeBuf.alloc(top, t, top->accP * t->p, t->source->end));
        }
    }
    return result;
}

struct CrossWrap{
    CrossWrap(vector<Transition const*> & seq):seq(seq){}
    vector<Transition const*> & seq;
    int operator[](size_t idx)const{
        if ( idx == seq.size() ) return seq.back()->source->end;
        return seq.at(idx)->source->begin;
    }
    size_t size()const{
        if ( seq.size() == 0 ) return 0;
        return seq.size() + 1;
    }
};

#include  <boost/algorithm/string.hpp>


bool stillHasOutEdgs(CrossInfo const& source, unordered_set<Transition const*> const& deleted_edge, unordered_set<int> const& deleted_cross)
{
    for(Transition const* t: source.outs){
        if ( deleted_edge.count(t)  == 0 && deleted_cross.count(t->source->end) == 0 ){
            return true;
        }
    }
    return true;
}
SequencePair const* MaxProbabilitySequenceGenerator::next()
{
    if ( tops_.empty() ) return nullptr;
    topk_.push_back(std::move(tops_.top()));
    tops_.pop();
    SequencePair &kth = topk_.back();
    unordered_set<Transition const*> deleted_edge;
    unordered_set<int> deleted_cross;
    //iter root
    vector<Transition const*> root;
    for (size_t i = 0; i < kth.first.size(); ++i){
        //deviate at edge i (a, b), mean deviate at node a
        deleted_edge.clear();

        //check all the top k path, if has same prefix ( the root ), delete
        //the edge of (root:last:end, q) q is the next node of each path
        for (auto& each : topk_ ){
            if ( boost::starts_with(each.first, root) ){
                deleted_edge.insert( each.first.at(root.size()) );
            }
        }

        int node = kth.first.at(i)->source->begin;
        if ( stillHasOutEdgs( tgrap_.crossInfo_[node], deleted_edge, deleted_cross ) )
        {
            //vector<Transition const*> supr 
        }

        deleted_cross.insert(kth.first.at(i)->source->begin);
        root.push_back(kth.first.at(i));
    }
    return &topk_.back();
}
