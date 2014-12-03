#ifndef  TRANSITIONGRAPH_H
#define  TRANSITIONGRAPH_H

#include    "road.h"
#include    "network.h"
#include  <unordered_map>
#include  <unordered_set>
#include  <string>
#include  <map>
#include  <queue>




struct Transition
{
    adjacent_edge const* source;
    double p;
    int freq;
};
struct CrossInfo
{
    int id;
    int freq;
    std::vector<Transition const*> outs;
    std::vector<Transition const*> ins;
    std::vector<Transition> adjacent;
};


class TransitionGraph
{
public:

    TransitionGraph(Network & network);

    bool load(std::string const& crossFile, std::string const& frowardFile);

    bool dump(std::string const& path)const;

    Transition const* transition(int begin, int end)const{
        if ( network_.contain_cross(begin) && network_.contain_cross(end) )
        {
            for(Transition const & t : crossInfo_[begin].adjacent )
            {
                if ( t.source->end == end )
                    return &t;
            }
            return nullptr;
        }
        return nullptr;
    }

    Transition const* transition(std::string const& begin, std::string const& end)const
    {
        if ( network_.contain_cross(begin) && network_.contain_cross(end) )
        {
            return transition(network_.cross(begin).id, network_.cross(end).id);
        }
        return nullptr;
    }

    std::vector<Transition const*> max_probabiliy_sequence(int begin, int end)const;
    std::vector<Transition const*> max_probabiliy_sequence(Cross const& begin, Cross const& end)const{
        return max_probabiliy_sequence(begin.id, end.id);
    }

    std::vector<Transition const*> max_probabiliy_sequence(std::string const& begin, std::string const& end)const{
        if ( network_.contain_cross(begin) && network_.contain_cross(end) )
            return max_probabiliy_sequence(network_.cross(begin), network_.cross(end));
        return {};
    }
private:
    void modifyTransition(){}
    friend class MaxProbabilitySequenceGenerator;
    std::vector<Transition const*> max_probabiliy_with_delete_edge_and_cross(int begin, int end, 
         std::unordered_set<Transition const*> const& deleted_edge, std::unordered_set<int> const& deleted_cross)const;
    Network& network_;
    std::vector<CrossInfo> crossInfo_;
};


typedef std::pair<std::vector<Transition const*>, double> SequencePair;
class MaxProbabilitySequenceGenerator{
public:
    MaxProbabilitySequenceGenerator(TransitionGraph const& grap, int begin, int end):tgrap_(grap),begin_(begin), end_(end){}
    SequencePair const* next();
private:
    //std::unordered_set<int> deleted_cross_;
    //sequence, prob
    std::priority_queue<SequencePair> tops_;
    std::vector<SequencePair> topk_;
    TransitionGraph const& tgrap_;
    int begin_;
    int end_;
};

#endif  /*TRANSITIONGRAPH_H*/
