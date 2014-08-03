#ifndef  EVALUATION_H
#define  EVALUATION_H


#include  <vector>

#include    "road.h"
#include    "network.h"


struct Segment{
    PathPoint const* first;
    PathPoint const* second;
    double length;
};

struct path_kdtree_node{
    Point const* point;
    Segment * segment;
    path_kdtree_node* left = nullptr;
    path_kdtree_node* right = nullptr;
    size_t split;
};

class PathInfo{
public:
    typedef std::vector<Segment> SegmentVector;
    typedef std::vector<std::pair<Point, Segment*> > SampleVector;
    PathInfo()=delete;
    PathInfo(Path const& path);
    //PathInfo(PathInfo const&)=delete;
    //PathInfo(PathInfo&& other);
    //PathInfo& operator=(PathInfo const& other)=delete;
    //PathInfo& operator=(PathInfo && other);
    //virtual ~PathInfo();

    SampleVector query ( Point const& point, double radious )const;
    std::vector<PathPoint> on_path_point(Point const& point , double distance_mask = 0.0)const;
    SampleVector::value_type nearlest_sample(Point const& point)const;

    SegmentVector segments;
    SampleVector samples; 
    Path const* path;
    size_t count()const;

    bool has_exception = false;

private:
    path_kdtree_node* _root;
};

struct EvaluationResult{
    double test_error;
    double labeled_error;
    double all_error;
    friend std::ostream& operator<<(std::ostream& o, EvaluationResult const& rst){
        return o << "test error : " << rst.test_error << "\n"
          << "label error: " << rst.labeled_error << "\n"
          << "all error  : " << rst.all_error;
    }
};
class Evaluation{
public:
    Evaluation(Network* network):_network(network){}


    EvaluationResult correct_rate_by_distance(PathInfo const& labeld, PathInfo const& test)const;


    void walk(PathInfo const& labeld, PathInfo const& test, 
            std::function<void(Segment const&)> not_match_segment_callback,
            std::function<void(Segment const&)> match_segment_callback
            )const;

    //double correct_rate_by_roadsegment_count(PathInfo const& labeled, PathInfo const& test)const;
    //double correct_rate_by_roadsegment_distance(PathInfo const& labeled, PathInfo const& test)const;

private:

    Network* _network;
    
};


#endif  /*EVALUATION_H*/
