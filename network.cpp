#include  <boost/algorithm/string.hpp>
#include  <boost/accumulators/accumulators.hpp>
#include  <boost/accumulators/statistics.hpp>
#include  <boost/range/algorithm.hpp>
#include  <boost/range.hpp>
#include  <boost/range/adaptors.hpp>
#include  <algorithm>
#include  <numeric>
#include  <shapefil.h>
#include  <boost/range/numeric.hpp>
#include  <boost/assign.hpp>
#include  <boost/algorithm/cxx11/none_of.hpp>
#include  <boost/filesystem.hpp>
#include  <fstream>
#include  <unordered_map>

#include    "network.h"
#include    "key_visitor.hpp"
#include    "range_extend.hpp"
#include    "evaluation.h"
#include "pathendpointwalker.hpp"

using namespace std;
using namespace boost::assign;

const double sample_step = 200.0;


vector<CandidatePoint> adjacent_edge::points()const{
    vector<CandidatePoint> p;
    if ( road != nullptr ){
        p = road->points;
        if ( begin == road->end.id ){
            boost::reverse(p);
        }
    }
    return p;
}


//=======================================================================
void Path::estimate_time_at_cross()
{
    auto estimateTime = [&](vector<PathPoint>::iterator it)
    {
        if ( it->timestamp != -1) return it->timestamp;
        PathPoint const& p2 = *find_if(it + 1, points.end(), [](PathPoint const& p ){ return p.timestamp != -1;});
        PathPoint const& p1 = *boost::find_if(boost::make_iterator_range(points.begin(), it) | boost::adaptors::reversed,
                [](PathPoint const& p){return p.timestamp != -1;} );
        double d = p2.dist_of_path - p1.dist_of_path;
        double d1 = it->dist_of_path - p1.dist_of_path;
        double d2 = p2.dist_of_path - it->dist_of_path;
        long t1 = p1.timestamp;
        long t2 = p2.timestamp;
        long t = (t1 + t2) / 2;
        if ( d != 0.0)
        {
            t = d1 / d * t2 + d2 /d * t1;
        }
        it->timestamp = t;
        return t;
    };
    PathEndpointWalker<Path> walker(*this);
    while ( not walker.isEnd() )
    {
        if (walker.first != points.begin() -1 and walker.first->cid != -1)
        {
            estimateTime(walker.first);
        }
        if ( walker.second != points.end() and walker.second->cid != -1)
        {
            estimateTime(walker.second);
        }
        ++walker;
    }
}


Path::Path(vector<CandidatePoint> const& cps):points(cps.begin(),cps.end()){
    if (not points.empty())
        update();
}

bool Path::append(Path const& other){
    if (other.empty() or infinity() )
        return true;

    if (other.infinity()){
        points.clear();
        length = std::numeric_limits<double>::infinity();
        return true;
    }

    if ( empty() ){
        points = other.points;
        length = other.length;
        return true;
    }

    PathPoint const& back = points.back();
    PathPoint const& front = other.points.front();
    if ( back.pos_equal( front )
        and (
                back.belong == front.belong
              or
                ( back.cid == front.cid and back.cid != -1  )
            )
        ) {
        boost::copy(other.points, back_inserter(points));
        update();
        return true;
    }

    return false;


    //copy( other.points.begin() + 1, other.points.end(), back_inserter(points) );
    //boost::copy(other.points, back_inserter(points));
    //update();
    //return true;
}

void Path::update(){
    /*points.resize(
            boost::unique<boost::return_begin_found>(points,
                [](Point const& a, Point const& b){ return a.pos_equal(b); }
            ).size()
    );*/
    boost::foreach_adjacent<boost::with_prepare>(points,[](PathPoint& first){
                first.dist_of_path = 0.0;
                if(first.belong->begin.pos_equal(first)){
                    first.cid = first.belong->begin.id;
                }else if(first.belong->end.pos_equal(first)){
                    first.cid = first.belong->end.id;
                }else{
                    first.cid = -1;
                }
            },
            [](PathPoint & first, PathPoint & second){
                second.dist_of_path = first.dist_of_path + first.gis_dist(second);
                if(second.belong->begin.pos_equal(second)){
                    second.cid = second.belong->begin.id;
                }else if(second.belong->end.pos_equal(second)){
                    second.cid = second.belong->end.id;
                }else{
                    second.cid = -1;
                }
            });
    length = points.back().dist_of_path;
}

bool Path::operator==(Path const& other)const{
    return double_nearly_equal(length, other.length)
        and boost::equal(points , other.points, [](Point const& a, Point const& b){ return a.pos_equal(b); }) ;
}

PathPoint Path::project(Point const& p)const{
    using namespace boost::adaptors;
    double acc = 0.0;
    PathPoint best;
    double min_dist = numeric_limits<double>::infinity();
    for(auto tup : points | boost::adjacented){
        auto& first = tup.get<0>();
        auto& second = tup.get<1>();
        if(first.belong == second.belong){
            RoadSegment const* road = first.belong;
            PathPoint pp = road->candidate_of(p);
            pp.dist_of_path = acc;
            if (first.pos_equal(road->begin)){
                pp.dist_of_path += pp.distance_from_begin();
            }else{
                pp.dist_of_path += pp.distance_to_end();
            }
            acc += road->length;
            double d = pp.gis_dist(p);
            if(d < min_dist){
                min_dist = d;
                best = pp;
            }
        }
    }
    return best;
}

PathPoint Path::point_of_dist(double where)const{
    auto iter =  boost::lower_bound(points, where, binary_of<less>(&PathPoint::dist_of_path));
    if(iter == points.end())
        return points.back();

    if(iter == points.begin() or iter->dist_of_path == where)
        return *iter;

    PathPoint const& point = iter[-1];
    PathPoint const& behind = iter[0];
    double distance = behind.dist_of_path - point.dist_of_path;
    double distance_sum = point.dist_of_path;
    double t = (where - distance_sum) / distance;
    Point p = point + (behind - point) * t;
    PathPoint cp = point;
    cp.x = p.x;
    cp.y = p.y;
    assert(point.belong == behind.belong);
    bool ordered = point.where < behind.where;
    if (ordered)
        cp.where += (where - distance_sum) / cp.belong->length;
    else
        cp.where -= (where - distance_sum) / cp.belong->length;
    if(cp.pos_equal(cp.belong->begin)){
        cp.cid = cp.belong->begin.id;
    }else if (cp.pos_equal(cp.belong->end)){
        cp.cid = cp.belong->end.id;
    }else{
        cp.cid = -1;
    }
    return cp;
}

pair< PathPoint const*, PathPoint const* > get_time_bound_point( size_t i,  vector<PathPoint> const& points){
    if ( points.at(i).timestamp != -1 ){
        return { & points.at(i), & points.at(i) };
    }
    int pre, next;
    for(pre = i - 1; pre >= 0 and points.at(pre).timestamp == -1; --pre)
        ;
    for(next = i + 1; next < points.size() and points.at(next).timestamp == -1; ++next)
        ;
    PathPoint const* first = nullptr;
    PathPoint const* last = nullptr;
    if ( pre >= 0) {
        first = &points.at(pre);
    }

    if ( next < points.size() ){
        last = &points.at(next);
    }

    return { first, last};
}

vector<SmoothTrajectorySegment> Path::smooth_trajectory()const{
    vector<SmoothTrajectorySegment> traj;

    vector< pair<PathPoint, size_t> > index;
    for ( size_t i = 0; i < points.size(); ++i ){
        if ( points.at(i).cid != -1 ) {
            index.push_back( {points.at(i), i } );
        }
    }

    for ( auto& element : index ){
        auto time_bound_point = get_time_bound_point(element.second, points);
        if ( time_bound_point.first == nullptr or time_bound_point.second == nullptr)
            continue;
        if ( time_bound_point.first == time_bound_point.second){
            element.first.timestamp = time_bound_point.first->timestamp;
        }else {
            double dist_pre = element.first.dist_of_path - time_bound_point.first->dist_of_path;
            double dist_next = time_bound_point.second->dist_of_path - element.first.dist_of_path;
            double dist = dist_next + dist_pre;
            long timestamp_pre = time_bound_point.first->timestamp;
            long timestamp_next = time_bound_point.second->timestamp;
            long timestamp;
            if ( dist == 0 ) timestamp = ( timestamp_pre + timestamp_next)  / 2;
            else {
                timestamp = timestamp_pre / dist * dist_next + timestamp_next / dist * dist_pre;
            }
            element.first.timestamp = timestamp;
        }
    }

    boost::foreach_adjacent( index | boost::adaptors::transformed(key_of(& pair<PathPoint, size_t>::first) ),
        [&traj](PathPoint const& first, PathPoint const& second) {
            assert(first.cid != -1 and second.cid != -1);
            if ( first.belong == second.belong and first.cid != second.cid ){
                SmoothTrajectorySegment seg;
                seg.road = first.belong;
                seg.begin = first.cid == seg.road->begin.id? & seg.road->begin : & seg.road->end;
                seg.end = second.cid == seg.road->begin.id? & seg.road->begin : & seg.road->end;
                seg.enter_timestamp = first.timestamp;
                seg.leave_timestamp = second.timestamp;
                traj.push_back(seg);
            }
    });
    size_t new_size = boost::unique<boost::return_begin_found>(traj, binary_of<equal_to> (&SmoothTrajectorySegment::road)).size();
    traj.resize(new_size);
    return traj;
}

//=========================================================================================
static vector<Point> point_from_padXY(double* x, double* y, int n){
    vector<Point> points;
    points.resize(n);
    for(int i = 0; i < n; ++i){
        points[i] = Point(x[i],y[i]);
    }
    return points;
}

static int max_variance_dim(vector<CandidatePoint>::iterator begin, vector<CandidatePoint>::iterator end){
    using namespace boost::accumulators;
    accumulator_set<double, stats<tag::lazy_variance> > xacc,yacc;
    for(auto it = begin; it != end; ++it){
        xacc(it->x);
        yacc(it->y);
    }
    return variance(xacc) > variance(yacc)? 0: 1;
}

static kdtree_node* build_kdtree(vector<CandidatePoint>::iterator begin, vector<CandidatePoint>::iterator end){
    if (begin == end){
        return nullptr;
    }else if(begin + 1 == end){
        kdtree_node * node = new kdtree_node;
        node->point = * begin;
        return node;
    }

    int dim = max_variance_dim(begin, end);
    auto sz = end - begin;
    nth_element(begin, begin + sz / 2,  end, [dim](CandidatePoint const& p1, CandidatePoint const& p2){
            return p1[dim] < p2[dim];
            });

    kdtree_node* node = new kdtree_node;
    auto nth = begin + sz / 2;
    node->point = *nth;
    node->split = dim;
    node->left = build_kdtree(begin, nth);
    node->right = build_kdtree(nth, end);
    if(node->left) node->left->parent = node;
    if(node->right) node->right->parent = node;
    return node;
}

static kdtree_node* build_kdtree(vector<RoadSegment> const& road_segment){
    vector<CandidatePoint> sample_points;
    for(auto & road : road_segment){
        int n = ceil(road.length / sample_step);
        double step = road.length / n;
        for(int i = 0; i <= n; ++i){
            sample_points += road.candidate_at_dist(i * step);
        }
    }
    return build_kdtree(sample_points.begin(), sample_points.end());
}


static void kdtree_free( kdtree_node * root ){
    if ( root ) {
        kdtree_free( root->left );
        kdtree_free( root->right );
        delete root;
    }
}
//==============================================================================
bool Network::load(string const& shp){
    _cross.clear();
    _road_segment.clear();
    _adjacent.clear();
    _cross_db_id_map.clear();
    _road_db_id_map.clear();
    if ( _root ) kdtree_free(_root);


    namespace fs = boost::filesystem;
    string name = boost::algorithm::erase_last_copy(shp, ".shp");
    string shp_name = name + ".shp";
    string dbf_name = name + ".dbf";

    SHPHandle hshp = SHPOpen(shp_name.c_str(), "rb");
    if ( not hshp ){
        return false;
    }

    DBFHandle hdbf = DBFOpen(dbf_name.c_str(), "rb");
    if (not hshp ){
        SHPClose(hshp);
        return false;
    }

    int count_of_record = DBFGetRecordCount(hdbf);
    _road_segment.reserve(count_of_record);
    _cross.reserve( count_of_record * 2 );

    const int DBID = 2;
    const int DIRECTION = 6;
    const int SNODEID = 18;
    const int ENODEID = 19;
    const int PATHCLASS = 20;
    string snodeID;
    string enodeID;
    string dbid;
    string direction;
    string path_class;
    vector<Point> points;
    const int SingleForward = 2;
    const int SingleBackward = 3;
    for ( int i = 0; i < count_of_record; ++i ){

        int new_road_id = _road_segment.size();
        dbid = DBFReadStringAttribute(hdbf, i, DBID);
        direction = DBFReadStringAttribute(hdbf, i , DIRECTION);
        snodeID = DBFReadStringAttribute(hdbf, i, SNODEID);
        enodeID = DBFReadStringAttribute(hdbf, i, ENODEID);
        path_class = DBFReadStringAttribute(hdbf,i,PATHCLASS);


        SHPObject* line_points = SHPReadObject(hshp, i);
        points = point_from_padXY( line_points->padfX, line_points->padfY, line_points->nVertices );
        SHPDestroyObject( line_points );

        int direction_id = atoi(direction.c_str());
        int path_class_id = atoi(path_class.c_str());
        double speed = 30 / 3.6;
        switch ( path_class_id ){
            case 4:
                speed = 70 / 3.6;
                break;
            case 3:
                speed = 50 / 3.6;
                break;
            case 2:
                speed = 40 / 3.6;
                break;
        }

        bool bidir = true;
        if ( direction_id == SingleForward ){
            bidir = false;
        }else if ( direction_id == SingleBackward  ){
            boost::reverse(points);
            swap( snodeID, enodeID );
            bidir = false;
        }



        if ( _cross_db_id_map.find( snodeID ) == _cross_db_id_map.end() ){
            assert(_cross.size() == _cross_db_id_map.size());
            int new_cross_id = _cross.size();
            _cross_db_id_map[snodeID] = new_cross_id;
            _cross.emplace_back(new_cross_id, snodeID, points.front().x, points.front().y);
        }
        if ( _cross_db_id_map.find( enodeID ) == _cross_db_id_map.end() ){
            assert(_cross.size() == _cross_db_id_map.size());
            int new_cross_id = _cross.size();
            _cross_db_id_map[enodeID] = new_cross_id;
            _cross.emplace_back(new_cross_id, enodeID, points.back().x, points.back().y);
        }

        int begin_id = _cross_db_id_map.at(snodeID);
        int end_id = _cross_db_id_map.at(enodeID);

        _road_segment.emplace_back(new_road_id, dbid, speed, bidir, _cross[begin_id], _cross[end_id], points);
        _road_db_id_map[dbid] = new_road_id;
    }

    _adjacent.resize(_cross.size());
    for (auto& each_road : _road_segment){
        _adjacent[each_road.begin.id].emplace_back( each_road.begin.id, each_road.end.id, & each_road );
        if ( each_road.bidir ){
            _adjacent[each_road.end.id].emplace_back( each_road.end.id, each_road.begin.id, & each_road );
        }
    }

    _root = build_kdtree ( _road_segment );
    SHPClose(hshp);
    DBFClose(hdbf);

    return true;
}

bool Network::load(string const& road, string const& cross){
    _cross.clear();
    _road_segment.clear();
    _adjacent.clear();
    _road_db_id_map.clear();
    _cross_db_id_map.clear();
    if ( _root ) kdtree_free(_root);

    namespace fs = boost::filesystem;
    string road_name = boost::algorithm::erase_last_copy(road, ".shp");
    string road_shp_name = road_name + ".shp";
    string road_dbf_name = road_name + ".dbf";

    string cross_name = boost::ierase_last_copy(cross, ".shp");
    string cross_shp_name = cross_name + ".shp";
    string cross_dbf_name = cross_name + ".dbf";


    SHPHandle hcross_shp = SHPOpen(cross_shp_name.c_str(), "rb");
    if ( not hcross_shp){
        return false;
    }
    DBFHandle hcross_dbf = DBFOpen(cross_dbf_name.c_str(), "rb");
    if ( not hcross_dbf ){
        SHPClose(hcross_shp);
        return false;
    }

    int cross_size = DBFGetRecordCount(hcross_dbf);
    _cross.resize(cross_size);
    string crossDBID;
    for(int i = 0; i < cross_size; ++i){
        int id = DBFReadIntegerAttribute(hcross_dbf, i, 0);
        crossDBID = DBFReadStringAttribute(hcross_dbf, i , 1);
        SHPObject* point = SHPReadObject(hcross_shp, i);
        double x =  point->padfX[0];
        double y = point ->padfY[0];
        _cross[id] = Cross(id, crossDBID, x, y);
        _cross_db_id_map[crossDBID] = id;
        SHPDestroyObject(point);
    }


    SHPClose(hcross_shp);
    DBFClose(hcross_dbf);

    SHPHandle hshp = SHPOpen(road_shp_name.c_str(), "rb");
    if ( not hshp ){
        return false;
    }

    DBFHandle hdbf = DBFOpen(road_dbf_name.c_str(), "rb");
    if (not hshp ){
        SHPClose(hshp);
        return false;
    }

    int count_of_record = DBFGetRecordCount(hdbf);
    _road_segment.reserve(count_of_record);
    _cross.reserve( count_of_record * 2 );

    const int DBID = 2;
    const int DIRECTION = 6;
    const int SNODEID = 18;
    const int ENODEID = 19;
    const int PATHCLASS = 20;
    string snodeID;
    string enodeID;
    string dbid;
    string direction;
    string path_class;
    vector<Point> points;
    const int SingleForward = 2;
    const int SingleBackward = 3;
    for ( int i = 0; i < count_of_record; ++i ){

        int new_road_id = _road_segment.size();
        dbid = DBFReadStringAttribute(hdbf, i, DBID);
        direction = DBFReadStringAttribute(hdbf, i , DIRECTION);
        snodeID = DBFReadStringAttribute(hdbf, i, SNODEID);
        enodeID = DBFReadStringAttribute(hdbf, i, ENODEID);
        path_class = DBFReadStringAttribute(hdbf,i,PATHCLASS);


        SHPObject* line_points = SHPReadObject(hshp, i);
        points = point_from_padXY( line_points->padfX, line_points->padfY, line_points->nVertices );
        SHPDestroyObject( line_points );

        int direction_id = atoi(direction.c_str());
        int path_class_id = atoi(path_class.c_str());
        double speed = 30 / 3.6;
        switch ( path_class_id ){
            case 4:
                speed = 70 / 3.6;
                break;
            case 3:
                speed = 50 / 3.6;
                break;
            case 2:
                speed = 40 / 3.6;
                break;
        }

        bool bidir = true;
        if ( direction_id == SingleForward ){
            bidir = false;
        }else if ( direction_id == SingleBackward  ){
            boost::reverse(points);
            swap( snodeID, enodeID );
            bidir = false;
        }


        assert( _cross_db_id_map.find(snodeID) != _cross_db_id_map.end() );
        assert( _cross_db_id_map.find(enodeID) != _cross_db_id_map.end() );

        int begin_id = _cross_db_id_map[snodeID];
        int end_id = _cross_db_id_map[enodeID];

        _road_segment.emplace_back(new_road_id, dbid, speed, bidir, _cross[begin_id], _cross[end_id], points);
        _road_db_id_map[dbid] = new_road_id;
    }

    _adjacent.resize(_cross.size());
    for (auto& each_road : _road_segment){
        _adjacent[each_road.begin.id].emplace_back( each_road.begin.id, each_road.end.id, & each_road );
        if ( each_road.bidir ){
            _adjacent[each_road.end.id].emplace_back( each_road.end.id, each_road.begin.id, & each_road );
        }
    }

    _root = build_kdtree ( _road_segment );
    SHPClose (hshp);
    DBFClose(hdbf);
    return true;
}

static size_t kdtree_deep(kdtree_node* node){
    if(node == nullptr) return 0;
    return max(kdtree_deep(node->left) + 1, kdtree_deep(node->right) + 1);
}

size_t Network::kdtree_deep()const{
    return ::kdtree_deep(_root);
}



void kdtree_query(vector<CandidatePoint>& ret, kdtree_node* tree, double radious, Point const& p){
    if(tree == nullptr) return;

    if(p.gis_dist(tree->point) < radious){
        ret += tree->point;
        kdtree_query(ret, tree->left, radious, p);
        kdtree_query(ret, tree->right, radious, p);
    }else{
        Point prj(p);
        prj[tree->split] = tree->point[tree->split];
        double dis = prj.gis_dist(p);
        if(dis < radious){
            kdtree_query(ret, tree->left, radious, p);
            kdtree_query(ret, tree->right, radious, p);
        }else{
            kdtree_query(ret, p[tree->split] < tree->point[tree->split] ? tree->left:tree->right, radious, p);
        }
    }
}


#include  <boost/algorithm/cxx11/copy_if.hpp>
vector<CandidatePoint> Network::query(Point const& point , double radious)const{
    using namespace boost;
    vector<CandidatePoint> rough_rst,rst;
    double more_radious = sqrt(radious * radious + sample_step * sample_step / 4.0);
    kdtree_query(rough_rst, _root, radious + more_radious, point);

    transform(unique<return_begin_found>(sort(rough_rst, binary_of<less>(&CandidatePoint::belong)),
                binary_of<equal_to>(&CandidatePoint::belong)), back_inserter(rst),[&point](CandidatePoint const& c){
            return c.belong->candidate_of(point);
            });
    auto end = boost::algorithm::copy_if(rst, rst.begin(), [radious,&point](CandidatePoint const& p){
            return p.gis_dist(point) <= radious;
            });
    rst.resize(end - rst.begin());
    return rst;
}

vector<CandidatePoint> Network::query(Point const& point, double radious, int limit)const{
    auto result = query(point, radious);
    boost::sort(result, [&point](CandidatePoint const& a, CandidatePoint const& b){
            return a.gis_dist(point) < b.gis_dist(point);
            });
    result.resize(std::min((size_t)limit, result.size()));
    return result;
}


vector<CandidatePoint> Network::query(GpsPoint const& point, double radious)const{
    auto points = query(static_cast<Point const&>(point), radious);
    for(auto& p : points) p.timestamp = point.timestamp;
    return points;
}
vector<CandidatePoint> Network::query(GpsPoint const& point, double radious, int limit)const{
    auto points = query(static_cast<Point const&>(point), radious, limit);
    for(auto& p : points) p.timestamp = point.timestamp;
    return points;
}

#include  <queue>


struct DijkstraNode{
    typedef shared_ptr<DijkstraNode> ptr;
    double acc_length;
    adjacent_edge* edge;
    shared_ptr<DijkstraNode> parent;

    static shared_ptr<DijkstraNode> pointer(double acc, adjacent_edge const* edge, shared_ptr<DijkstraNode> p = nullptr){
        shared_ptr<DijkstraNode> pnode{new DijkstraNode};
        pnode->acc_length = acc;
        pnode->edge = (adjacent_edge*)edge;
        pnode->parent = p;
        return pnode;
    }
};

namespace std{
    template<>
        struct less<DijkstraNode::ptr>{
            bool operator ()(DijkstraNode::ptr const& l, DijkstraNode::ptr const& r)const{
                return l->acc_length > r->acc_length;
            }
        };
}

Path Network::shortest_path(Cross const& begin, Cross const& end)const{
    if(begin == end){
        return Path::empty_path();
    }

    priority_queue<DijkstraNode::ptr>  queue;
    unordered_set<int> min_path_found;
    min_path_found.insert(begin.id);
    for(adjacent_edge const& e: _adjacent[ begin.id ]){
        queue.push(DijkstraNode::pointer(e.road->length, &e));
    }

    while (not queue.empty()){
        DijkstraNode::ptr p = queue.top();
        queue.pop();
        if(min_path_found.count(p->edge->end)){
            continue;
        }else{
            min_path_found.insert(p->edge->end);
        }

        if(p->edge->end == end.id){
            Path pth;
            while(p){
                bool ordered = p->edge->begin == p->edge->road->begin.id;
                if(ordered){
                    copy(p->edge->road->points | boost::adaptors::reversed, back_inserter(pth.points));
                }else{
                    copy(p->edge->road->points, back_inserter(pth.points));
                }
                p = p->parent;
            }
            boost::reverse(pth.points);
            pth.update();
            return pth;
        }

        for(adjacent_edge const& e: _adjacent[ p->edge->end ]){
            if(min_path_found.count(e.end) == 0){
                queue.push(DijkstraNode::pointer(p->acc_length + e.road->length, &e, p));
            }
        }
    }
    return Path::inf_path();//not found, inf path
}


#include  <cassert>
static pair<Cross, Cross> outer(CandidatePoint const& begin, CandidatePoint const& end){
    assert(begin.belong == end.belong);
    RoadSegment* rd = begin.belong;
    if(begin.where < end.where){
        return make_pair(rd->begin, rd->end);
    }else{
        return make_pair(rd->end, rd->begin);
    }
}

static CandidatePoint from_cross(RoadSegment* rd, Cross const& c){
    if(c == rd->begin) return rd->points.front();
    return rd->points.back();
}

static Path& attach_timestamp(Path & path, CandidatePoint const& begin, CandidatePoint const& end){
    if (not path.points.empty()){
        path.points.front().timestamp = begin.timestamp;
        path.points.back().timestamp = end.timestamp;
    }
    return path;
}
#include  <boost/range/algorithm_ext.hpp>
Path Network::_shortest_path(CandidatePoint const& begin, CandidatePoint const& end,
        Path (Network::*cross2cross_shortest_path)(Cross const&,Cross const&)const )const{

    if( begin.pos_equal( end) ){
        return Path::empty_path();
    }

    if(begin.belong == end.belong){
        RoadSegment* rd = begin.belong;
        if(begin.belong->bidir){//bidir
            Path direct_pth(rd->path_follow(begin.where, end.where));
            assert( not direct_pth.empty() and not direct_pth.infinity() );

            auto outer_cross = outer(begin, end);

            Path head = rd->path_follow(begin.where, from_cross(rd, outer_cross.first).where);
            assert ( ! head.infinity() );

            Path tail = rd->path_follow(from_cross(rd,outer_cross.second).where, end.where);
            assert ( ! tail.infinity() );

            if(head.length + tail.length > direct_pth.length){
                return attach_timestamp(direct_pth, begin, end);
            }else{
                //Path body = shortest_path(outer_cross.first, outer_cross.second);

                //maybe inf
                Path body = (this->*cross2cross_shortest_path)(outer_cross.first, outer_cross.second);


                if(direct_pth.length < head.length + body.length + tail.length){
                    return attach_timestamp(direct_pth, begin, end);
                }else{
                    bool should_success = ( head.append(body) );
                    assert(should_success);
                    should_success = ( head.append(tail) );
                    assert(should_success);
                    return attach_timestamp(head, begin, end);//maybe inf
                }
            }
        }else{//single pass

            if(begin.where < end.where){
                Path direct_pth = rd->path_follow(begin.where, end.where);
                assert( not direct_pth.empty() and not direct_pth.infinity() );

                return attach_timestamp(direct_pth, begin,end);
            }else{

                auto outer_cross = outer(begin, end);
                Path head = rd->path_follow(begin.where, from_cross(rd, outer_cross.first).where);
                assert( not head.infinity() );

                Path tail = rd->path_follow(from_cross(rd,outer_cross.second).where, end.where);
                assert( not tail.infinity() );


                //may be inf
                Path body = (this->*cross2cross_shortest_path)(outer_cross.first, outer_cross.second);

                bool should_success = ( head.append(body) );
                assert(should_success);
                should_success = ( head.append(tail) );
                assert(should_success);
                return attach_timestamp(head, begin, end);//maybe inf
            }
        }
    }else {
        vector<Cross> cross_after_begin;
        vector<Cross> cross_before_end;
        Path min = Path::inf_path();//default inf
        min.length = numeric_limits<double>::infinity();

        cross_after_begin += begin.belong->end;
        if(begin.belong->bidir) cross_after_begin += begin.belong->begin;

        cross_before_end += end.belong->begin;
        if(end.belong->bidir) cross_before_end += end.belong->end;

        RoadSegment* begin_rd = begin.belong;
        RoadSegment* end_rd = end.belong;

        for(auto& after_begin: cross_after_begin){
            for(auto& before_end : cross_before_end){
                Path head = begin_rd->path_follow(begin.where, from_cross(begin_rd, after_begin).where);
                assert(not head.infinity() );


                Path tail = end_rd->path_follow(from_cross(end_rd, before_end).where, end.where);
                assert(not tail.infinity() );


                Path body = (this->*cross2cross_shortest_path)(after_begin, before_end);
                bool should_success = ( head.append(body) );
                assert(should_success);
                should_success = ( head.append(tail) );//maybe inf
                assert(should_success);

                if(head.length < min.length){
                    min = std::move(head);
                }
            }
        }
        return attach_timestamp(min, begin, end);//maybe inf
    }
}

struct AstarNode{
    double g;
    double h;
    adjacent_edge * edge;
    typedef shared_ptr<AstarNode> ptr;
    ptr parent;
    static std::shared_ptr<AstarNode> pointer(double g, double h, adjacent_edge const* edge, ptr parent = nullptr){
        ptr p(new AstarNode);
        p->g = g;
        p->h = h;
        p->edge = (adjacent_edge*)edge;
        p->parent = parent;
        return p;
    }
};

namespace std{
    template<>
    struct less<AstarNode::ptr>{
        bool operator()(AstarNode::ptr const& a, AstarNode::ptr const& b)const{
            return not ((a->g + a->h) < (b->g + b->h));
        }
    };
};

Path Network::shortest_path_Astar(Cross const& begin, Cross const& end)const{
    if(begin == end){
        return Path::empty_path();
    }

    priority_queue<AstarNode::ptr>  open;
    unordered_set<int> close;
    close.insert(begin.id);
    for(auto& e : _adjacent[begin.id]){
        open.push(AstarNode::pointer( e.road->length, _cross[e.end].gis_dist(end) , &e));
    }
    while(not open.empty()){
        AstarNode::ptr p = open.top();
        open.pop();
        if(close.count(p->edge->end) == 0){
            close.insert(p->edge->end);
        }else{
            continue;
        }

        if(p->edge->end == end.id){
            Path pth;
            while(p){
                bool ordered = p->edge->begin == p->edge->road->begin.id;
                if(ordered){
                    copy(p->edge->road->points | boost::adaptors::reversed, back_inserter(pth.points));
                }else{
                    copy(p->edge->road->points, back_inserter(pth.points));
                }
                p = p->parent;
            }
            boost::reverse(pth.points);
            pth.update();
            return pth;//fount
        }

        for(auto& e : _adjacent[p->edge->end]){
            if(close.count( e.end ) == 0){
                open.push(AstarNode::pointer(p->g + e.road->length, _cross[e.end].gis_dist(end), &e, p) );
            }
        }
    }
    return Path::inf_path();//not found
}

struct KSPNode{
    typedef shared_ptr<KSPNode> ptr;
    double g;
    double h;
    adjacent_edge * edge;
    ptr parent;
    unordered_set<int> in_path;

    static ptr pointer(double g, double h, adjacent_edge const* e, ptr parent = nullptr){
        ptr p(new KSPNode);
        p->edge = (adjacent_edge*)e;
        p->g = g;
        p->h = h;
        p->parent = parent;
        if(parent == nullptr){
            p->in_path.insert(e->begin);
        }else{
            p->in_path = parent->in_path;
        }
        p->in_path.insert(e->end);
        return p;
    }
};

namespace std{
    template<>
    struct less<KSPNode::ptr>{
        bool operator()(KSPNode::ptr const& l, KSPNode::ptr const& r)const{
            return (l->g + l->h) > (r->g + r->h);
        }
    };
}

vector<Path> Network::k_shortest_path(Cross const& begin, Cross const& end, int k)const{
    vector<Path> paths;

    if(begin == end) return paths;

    priority_queue<KSPNode::ptr> queue;

    for(auto & e : _adjacent[begin.id]){
        KSPNode::ptr p = KSPNode::pointer(e.road->length, _cross[e.end].gis_dist(end), &e);
        queue.push(p);
    }

    while(not queue.empty()){
        KSPNode::ptr top = queue.top();
        queue.pop();
        if (top->edge->end == end.id){
            Path pth;
            KSPNode::ptr p = top;
            while(p){
                bool ordered = p->edge->begin == p->edge->road->begin.id;
                if(ordered){
                    copy(p->edge->road->points | boost::adaptors::reversed, back_inserter(pth.points));
                }else{
                    copy(p->edge->road->points, back_inserter(pth.points));
                }
                p = p->parent;
            }
            boost::reverse(pth.points);
            pth.update();
            paths.push_back(std::move(pth));
            if (paths.size() == (size_t)k){
                break;
            }
        }

        for (auto& e : _adjacent[top->edge->end]){
            if(top->in_path.count(e.end) == 0){
                queue.push(KSPNode::pointer(top->g + e.road->length, _cross[e.end].gis_dist(end), &e, top));
            }
        }
        top->in_path.clear();
    }
    return paths;
}



vector<adjacent_edge const*>
Network::shortest_path_Astar(int begin, int end,
        unordered_set<int> const& deleted_cross,
        unordered_set<adjacent_edge const*> const& deleted_edge)const{

    vector<adjacent_edge const*> pth;
    if(begin == end){
        return pth;
    }
    unordered_set<int> close;
    priority_queue<AstarNode::ptr> open;

    close.insert(begin);
    for(auto& e : _adjacent[begin]){
        if(deleted_edge.count(&e) == 0 and deleted_cross.count(e.end) == 0){
            open.push(AstarNode::pointer(e.road->length, _cross[e.end].gis_dist(_cross[end]),&e));
        }
    }

    while(not open.empty()){

        AstarNode::ptr p = open.top();
        open.pop();
        if(close.count(p->edge->end) == 0){
            close.insert(p->edge->end);
        }else{
            continue;
        }

        if(p->edge->end == end){
            for(; p ; p = p->parent){
                pth.push_back(p->edge);
            }
            boost::reverse(pth);
            return pth;
        }

        for(auto& e : _adjacent[p->edge->end]){
            if(close.count(e.end) == 0 and deleted_edge.count(&e) == 0 and deleted_cross.count(e.end) == 0 ){
                open.push(AstarNode::pointer(e.road->length, _cross[e.end].gis_dist(_cross[end]), &e, p));
            }
        }
    }
    return pth;
}

#include  <list>

inline static double total_length(vector<adjacent_edge const*> const& path){
    return boost::accumulate(path, 0.0, [](double init, adjacent_edge const* edge){ return init + edge->road->length; });
}


static vector<Path> create_from_top_k(vector<pair<vector<adjacent_edge const*>, double> > const& top_k){
    vector<Path> paths;
    for(auto& p : top_k){
        Path pth;
        for(auto& adj : p.first){
            if(adj->begin == adj->road->begin.id){
                boost::push_back(pth.points, adj->road->points);
            }else{
                boost::push_back(pth.points, adj->road->points | boost::adaptors::reversed);
            }
        }

        pth.update();
        paths.push_back(std::move(pth));
    }
    return paths;
}

vector<Path> Network::k_shortest_path_Yen(int begin, int end, int k)const{
    unordered_set<int> deleted_cross;
    unordered_set<adjacent_edge const*> deleted_edge;
    typedef pair<vector<adjacent_edge const*>, double> path_pair;
    vector<path_pair> the_k_shortest;
    list<path_pair> to_be_find_the_kth;

    auto init = shortest_path_Astar(begin ,end, deleted_cross, deleted_edge);
    if (init.size() == 0){
        return vector<Path>();
    }
    double len = total_length(init);
    path_pair p;
    p.first = std::move(init);
    p.second = len;

    the_k_shortest.push_back(std::move(p));



    for(int iter = 1; iter < k; ++iter){
        deleted_edge.clear();

        auto& last_path = the_k_shortest.back();
        for(size_t i = 0; i < last_path.first.size(); ++i){
            deleted_edge.clear();
            deleted_cross.clear();

            vector<adjacent_edge const*> root(last_path.first.begin(), last_path.first.begin() + i);
            int node_id = last_path.first[i]->begin;

            for(size_t j = 0; j < i; ++j){
                //if has node_id -> last_path.first[j]->begin
                //when last_path.first[j]->begin is deleted
                //the degree_out should be - 1
                int begin = last_path.first[j]->begin;
                for(auto& e : _adjacent[node_id]){
                    if (e.end == begin){
                        deleted_edge.insert(&e);
                    }
                }
                deleted_cross.insert(begin);
            }

            size_t degree_out = _adjacent[last_path.first[i]->begin].size();
            for(auto& each : the_k_shortest){
                for(auto& adj : each.first){
                    if(adj->begin == node_id){
                        deleted_edge.insert(adj);
                        break;
                    }
                }
            }
            if(deleted_edge.size() >= degree_out){
                continue;
            }else{
                auto supr = shortest_path_Astar(last_path.first[i]->begin, end, deleted_cross, deleted_edge);
                if(supr.size() == 0){
                    continue;
                }
                boost::push_back(root, supr);
                len = total_length(root);
                p.first = std::move(root);
                p.second = len;
                to_be_find_the_kth.push_back(std::move(p));
            }
        }

        auto min_iter = boost::min_element(to_be_find_the_kth, binary_of<less>(&path_pair::second));
        the_k_shortest.push_back(std::move(*min_iter));
        to_be_find_the_kth.erase(min_iter);
    }

    return create_from_top_k(the_k_shortest);
}




kdtree_node* nearlest(kdtree_node* root, Point const& p){
    kdtree_node* pnearlest = root;
    kdtree_node* another = nullptr;
    if(p[root->split] < root->point[root->split] and root->left){
        pnearlest = nearlest(root->left, p);
        another = root->right;
    }else if(p[root->split] > root->point[root->split] and root->right){
        pnearlest = nearlest(root->right, p);
        another = root->left;
    }

    if(pnearlest == root)
        return pnearlest;

    double distance = pnearlest->point.gis_dist(p);
    Point the_proj(p);
    the_proj[root->split] = root->point[root->split];
    double distance2 = the_proj.gis_dist(p);
    if(distance > distance2 and another){
        kdtree_node* another_nearlest = nearlest(another, p);
        distance2 = another_nearlest->point.gis_dist(p);
        if(distance2 < distance){
            pnearlest = another_nearlest;
            distance = distance2;
        }
    }
    double cur_distance = root->point.gis_dist(p);
    if(cur_distance < distance){
        pnearlest = root;
    }

    return pnearlest;
}

CandidatePoint Network::project(Point const& p)const{
    kdtree_node* pnearlest = nearlest(_root, p);
    double radious = pnearlest->point.gis_dist(p);
    auto cps = query(p, radious);
    while(cps.empty()){
        radious *= 1.1 + 1;
        cps = query(p, radious);
    }
    using namespace boost::adaptors;
    auto filtered_cps = cps | transformed(key_of(&CandidatePoint::belong))
        | uniqued | transformed([&p](RoadSegment const* r){ return r->candidate_of(p); });
    CandidatePoint best;
    double min_dist = numeric_limits<double>::max();
    for(auto const& c : filtered_cps){
        double d = c.gis_dist(p);
        if(d < min_dist){
            min_dist = d;
            best = c;
        }
    }
    return best;
}

/*
void Network::split( int road_id, int cross_id )const{
   RoadSegment const& road = _road_segment[road_id];
   Cross c = _cross[cross_id];
   CandidatePoint cp = road.candidate_of( c );
   auto lower_it = boost::lower_bound(road.points, cp, binary_of<less>(&CandidatePoint::where));
   auto upper_it = boost::upper_bound(road.points, cp, binary_of<less>(&CandidatePoint::where));
   if (lower_it == road.points.begin() or upper_it == road.points.end()){
       cerr << "no need to split " << endl;
       return;
   }
   string name = road.name;
   string level = road.level;
   double speed = road.level == "unknown" ? 0 : road.speed;
   bool bidir = road.bidir;
   Cross const& begin = road.begin;
   Cross const& end = road.end;
   Cross const& mid = _cross[cross_id];
   vector<Point> left;
   vector<Point> right;
   * std::copy(road.points.begin(), lower_it, back_inserter(left)) = mid;
   right.push_back(mid);
   std::copy(upper_it, road.points.end(), back_inserter(right));

   auto acc_fun = [](double init , boost::tuple<Point const&,Point const&> const& tup){
            return init = tup.get<0>().gis_dist(tup.get<1>());
           };
   double left_length = boost::accumulate( left | boost::adjacented, 0.0,acc_fun);
   double right_length = boost::accumulate( right | boost::adjacented, 0.0, acc_fun );

   double lx[left.size()];
   double ly[left.size()];
   double rx[right.size()];
   double ry[right.size()];

   boost::transform(left, lx, key_of(&Point::x));
   boost::transform(left, ly, key_of(&Point::y));
   boost::transform(right, rx, key_of(&Point::x));
   boost::transform(right, ry, key_of(&Point::y));

   SHPHandle shp = SHPOpen( _saved_shp_name.c_str(), "rb+" );
   DBFHandle dbf = DBFOpen( _saved_dbf_name.c_str(), "rb+" );

   if ( DBFIsRecordDeleted( dbf, road_id ) ){
       DBFClose(dbf);
       SHPClose(shp);
       cout << "Waring: " << road_id << " already deleted" << endl;
       return;
   }

   SHPObject* left_arc = SHPCreateSimpleObject( SHPT_ARC, left.size(), lx, ly, nullptr );
   SHPObject* right_arc = SHPCreateSimpleObject(SHPT_ARC, right.size(), rx, ry, nullptr);

   int left_id = SHPWriteObject( shp, -1 , left_arc );
   int right_id = SHPWriteObject( shp, -1, right_arc );

   DBFWriteIntegerAttribute(dbf, left_id, 0, left_id);//ID
   DBFWriteStringAttribute( dbf, left_id, 1, name.c_str() );//name
   DBFWriteStringAttribute( dbf, left_id, 2, level.c_str() );//level
   DBFWriteIntegerAttribute( dbf, left_id, 3, speed);//speed
   DBFWriteDoubleAttribute( dbf, left_id, 4, left_length);//length
   DBFWriteIntegerAttribute( dbf,  left_id, 5, bidir);//bidir
   DBFWriteIntegerAttribute( dbf, left_id, 6, begin.id );//begin
   DBFWriteIntegerAttribute( dbf, left_id, 7, mid.id );//end



   DBFWriteIntegerAttribute(dbf, right_id, 0, right_id);//ID
   DBFWriteStringAttribute( dbf, right_id, 1, name.c_str() );//name
   DBFWriteStringAttribute( dbf, right_id, 2, level.c_str() );//level
   DBFWriteIntegerAttribute( dbf, right_id, 3, speed);//speed
   DBFWriteDoubleAttribute( dbf, right_id, 4, right_length);//length
   DBFWriteIntegerAttribute( dbf,  right_id, 5, bidir);//bidir
   DBFWriteIntegerAttribute( dbf, right_id, 6, mid.id );//begin
   DBFWriteIntegerAttribute( dbf, right_id, 7, end.id );//end

   bool deleted  = true;
   DBFMarkRecordDeleted(dbf, road_id, deleted);


   SHPDestroyObject( right_arc );
   SHPDestroyObject( left_arc );

   DBFClose( dbf );
   SHPClose(shp);
}*/


void Network::save_cross_to_map(string const& name)const{
    string basename = boost::ierase_last_copy(name, ".shp");
    string shp_name = basename + ".shp";
    string dbf_name = basename + ".dbf";
    SHPHandle shp = SHPCreate(shp_name.c_str(), SHPT_POINT);
    DBFHandle dbf = DBFCreate(dbf_name.c_str());
    DBFAddField(dbf, "id", FTInteger, 10, 0);
    DBFAddField(dbf, "DBID", FTString, 16, 0);
    for( Cross const& c : _cross ){
        double x = c.x;
        double y = c.y;
        SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, & x, & y, nullptr);
        int id = SHPWriteObject(shp, -1, obj);
        DBFWriteIntegerAttribute(dbf, id, 0, c.id);
        DBFWriteStringAttribute(dbf, id, 1, c.dbId.c_str());
        SHPDestroyObject( obj );
    }
    DBFClose(dbf);
    SHPClose(shp);
}
Network::~Network(){
    if ( _root ) kdtree_free( _root );
    _root = nullptr;
    _adjacent.clear();
    _cross.clear();
    _road_segment.clear();
}


bool Network::dumpTrajPointToFile( std::string const & traj, std::string const& file )const{
    ifstream ifs(traj);
    if ( ! ifs ) return false;
    SHPHandle shp = SHPCreate(file.c_str(), SHPT_POINT);
    if ( shp == nullptr ) return false;
    DBFHandle dbf = DBFCreate(file.c_str());
    if ( dbf == nullptr ) {
        SHPClose(shp);
        return false;
    }

    string cross;
    string time;
    long timestamp;
    DBFAddField(dbf, "cross", FTString, 11, 0);
    DBFAddField(dbf, "time", FTString, 20, 0);
    DBFAddField(dbf, "timestamp", FTInteger, 10, 0);
    while ( getline(ifs, cross, ',') && getline(ifs, time, ',') && ifs >> timestamp && ifs.ignore() ){
        cout << cross << endl;
        cout << time << endl;
        cout << timestamp << endl;
        double x = this->cross(cross).x; 
        double y = this->cross(cross).y;
        SHPObject* obj = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y , nullptr);
        const int insert = -1;
        int newID = SHPWriteObject(shp, insert, obj);
        DBFWriteStringAttribute(dbf, newID, 0, cross.c_str());
        DBFWriteStringAttribute(dbf, newID, 1, time.c_str());
        DBFWriteIntegerAttribute(dbf, newID, 2, timestamp);
        SHPDestroyObject(obj);
    }
    SHPClose(shp);
    DBFClose(dbf);
    return true;
}

bool Network::dumpTrajPathToFile( std::string const& traj, std::string const& file )const{
    ifstream ifs(traj);
    if ( ! ifs ) return false;
    SHPHandle shp = SHPCreate(file.c_str(), SHPT_ARC);
    if ( shp == nullptr ) return false;
    DBFHandle dbf = DBFCreate(file.c_str());
    if ( dbf == nullptr ) {
        SHPClose(shp);
        return false;
    }

    DBFAddField(dbf, "length", FTDouble, 10, 6);
    DBFAddField(dbf, "timecost", FTInteger, 10, 0);
    string cross;
    long timestamp;
    string preCross;
    long preTimestamp;
    getline(ifs, preCross, ',') && ifs.ignore(numeric_limits<streamsize>::max(), ',') && ifs >> preTimestamp && ifs.ignore();
    while ( getline(ifs, cross, ',') && ifs.ignore(numeric_limits<streamsize>::max(), ',') && ifs >> timestamp && ifs.ignore() ){
        adjacent_edge const* edge = this->edge(preCross, cross);
        if ( edge ){
            size_t sz = edge->road->points.size();
            double x[sz];
            double y[sz];
            boost::transform(edge->road->points, x, key_of(&Point::x));
            boost::transform(edge->road->points, y, key_of(&Point::y));
            SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, sz, x, y, nullptr);
            int const insert = -1;
            int newID = SHPWriteObject(shp, insert, obj);
            DBFWriteDoubleAttribute(dbf, newID, 0, edge->road->length);
            DBFWriteIntegerAttribute(dbf, newID, 1, timestamp - preTimestamp);
        }
        preCross = std::move(cross);
        preTimestamp = timestamp;
    }
    SHPClose(shp);
    DBFClose(dbf);
    return true;

}

