#ifndef  NETWORK_H
#define  NETWORK_H

#include  <string>
#include  <vector>
#include  <limits>
#include  <memory>
#include  <unordered_set>
#include  <unordered_map>
#include    "road.h"

struct kdtree_node{
    CandidatePoint point;
    int split = 0;
    kdtree_node* parent = nullptr;
    kdtree_node* left = nullptr;
    kdtree_node* right = nullptr;
};

struct adjacent_edge{
    int begin;
    int end;
    RoadSegment* road;
    adjacent_edge()=default;
    adjacent_edge(int b, int e, RoadSegment const* r):
        begin(b),end(e),road((RoadSegment*)r){};

    std::vector<CandidatePoint>  points()const;
};


struct PathPoint: public CandidatePoint{
    PathPoint()=default;
    PathPoint(CandidatePoint const& c):CandidatePoint(c){}
    Cross const* cross()const{
        if ( cid == -1 ) return nullptr;
        if ( cid == belong->begin.id ){
            return & belong->begin;
        }
        return & belong->end;
    }
    double dist_of_path = 0.0;
    int cid = -1;
};



struct SmoothTrajectorySegment{
    RoadSegment const* road;
    Cross const* begin;
    Cross const* end;
    long enter_timestamp;
    long leave_timestamp;
};
class Path: boost::equality_comparable<Path>{
public:
    Path()=default;//empty one
    Path(std::vector<CandidatePoint> const& p);
    std::vector<PathPoint> points;
    double length = 0.0;
    bool operator==(Path const& other)const;
    void update();

    bool empty()const{
        return points.empty() and length == 0.0;
    }

    bool infinity()const{
        return points.empty() and std::isinf(length);
    }

    PathPoint point_of_dist(double where)const;
    PathPoint project(Point const& p)const;

    Path static empty_path(){
        Path p;
        return p;
    }

    Path static inf_path(){
        Path p;
        p.length = std::numeric_limits<double>::infinity();
        return p;
    }

    void estimate_time_at_cross();

    std::vector<SmoothTrajectorySegment> smooth_trajectory()const;


    bool append(Path const& other);

};


class Network{
public:
    Network()=default;

    bool load( std::string const& shp);
    bool load( std::string const& road, std::string const& cross );

    virtual ~Network();
    size_t kdtree_deep()const;

    std::vector<CandidatePoint> query(GpsPoint const&, double radious)const;
    std::vector<CandidatePoint> query(GpsPoint const&, double radious, int limit)const;
    std::vector<CandidatePoint> query(Point const&, double radious)const;
    std::vector<CandidatePoint> query(Point const&, double radious, int limit)const;


    Path shortest_path(Cross const& begin, Cross const& end)const;
    inline Path shortest_path(int begin, int end)const{
        return shortest_path(_cross[begin], _cross[end]);
    }
    inline Path shortest_path(std::string const& begin, std::string const& end)const{
        if (_cross_db_id_map.count(begin) && _cross_db_id_map.count(end) ){
            return shortest_path(_cross_db_id_map.at(begin), _cross_db_id_map.at(end));
        }
        return {};
    }


    Path shortest_path_Astar(Cross const& begin, Cross const& end)const;
    inline Path shortest_path_Astar(int begin, int end)const{
        return shortest_path_Astar(_cross[begin], _cross[end]);
    }

    inline Path shortest_path_Astar(std::string const& begin, std::string const& end)const{
        if (_cross_db_id_map.count(begin) && _cross_db_id_map.count(end) ){
            return shortest_path_Astar(_cross_db_id_map.at(begin), _cross_db_id_map.at(end));
        }
        return {};
    }


    inline Path shortest_path(CandidatePoint const& begin, CandidatePoint const& end)const{
        return _shortest_path(begin, end, &Network::shortest_path);
    }

    inline Path shortest_path_Astar(CandidatePoint const& begin, CandidatePoint const& end)const{
        return _shortest_path(begin, end, &Network::shortest_path_Astar);
    }

    //k shortest path simple imp but slow
    std::vector<Path> k_shortest_path(Cross const& begin, Cross const& end, int k)const;
    inline std::vector<Path> k_shortest_path(int begin, int end, int k)const{
        return k_shortest_path(_cross[begin], _cross[end], k);
    }

    std::vector<Path> k_shortest_path_Yen(int begin, int end, int k)const;
    inline std::vector<Path> k_shortest_path_Yen(std::string const& begin, std::string const& end, int k)const{
        if (_cross_db_id_map.count(begin) && _cross_db_id_map.count(end) ){
            return k_shortest_path_Yen(_cross_db_id_map.at(begin), _cross_db_id_map.at(end), k);
        }
        return {};
    }


    std::vector<adjacent_edge const*>
    shortest_path_Astar(int begin, int end,
            std::unordered_set<int> const& deleted_cross,
            std::unordered_set<adjacent_edge const*> const& deleted_edge)const;

    inline int cross_bound()const{ return _cross.size(); }
    inline int roadsegment_bound()const{ return _road_segment.size();}
    inline Cross const& cross(int i)const{ return _cross[i]; }
    inline Cross const& cross(std::string const& dbId)const{ return _cross[_cross_db_id_map.at(dbId)]; }
    RoadSegment const* road(int i ) const{ return &_road_segment[i]; }
    RoadSegment const* road(std::string const& dbId)const{ return & _road_segment[_road_db_id_map.at(dbId)]; }

    CandidatePoint project(Point const& p)const;

    //void split( int road_id, int cross_id )const;

    void save_cross_to_map(std::string const& name)const;

    adjacent_edge const* edge(int from, int to)const
    {
        if ( from < 0 || from >= _adjacent.size() )
            return nullptr;
        if ( to < 0 || to >= _adjacent.size() )
            return nullptr;
        for (auto& e : _adjacent.at(from))
        {
            if (e.end == to)
                return &e;
        }
        return nullptr;
    }

    adjacent_edge const* edge(std::string const& from, std::string const& to )const
    {
        if ( _cross_db_id_map.count(from) == 0 ||  _cross_db_id_map.count(to) == 0)
            return nullptr;
        int ifrom = _cross_db_id_map.at(from);
        int ito = _cross_db_id_map.at(to);
        return edge(ifrom ,ito);
    }

    std::vector<std::vector<adjacent_edge> >& adjacent_table(){
        return _adjacent;
    }

    std::vector<std::vector<adjacent_edge> > const& adjacent_table()const{
        return _adjacent;
    }

    bool contain_cross(int i)const{
        if ( i < 0 || i >= cross_bound() )
            return false;
        return true;
    }
    bool contain_cross(std::string const& id)const{
        return _cross_db_id_map.count(id) > 0;
    }
    bool contain_road(int i )const{
        if ( i < 0 || i >= roadsegment_bound() )
            return false;
        return true;
    }
    bool contain_road(std::string const& id)const{
        return _road_db_id_map.count(id) > 0;
    }

    bool dumpTrajPointToFile( std::string const& traj, std::string const& file )const;
    bool dumpTrajPathToFile( std::string const& traj, std::string const& file )const;
private:
    kdtree_node* _root = nullptr;
    std::vector<std::vector<adjacent_edge> > _adjacent;
    std::vector<Cross> _cross;
    std::vector<RoadSegment> _road_segment;
    Path _shortest_path(CandidatePoint const& begin, CandidatePoint const& end,
            Path (Network::*cross2cross_shortest_path)(Cross const&,Cross const&)const)const;

    std::unordered_map<std::string , int>  _cross_db_id_map;
    std::unordered_map<std::string , int > _road_db_id_map;
    std::string _saved_shp_name;
};


#endif  /*NETWORK_H*/
