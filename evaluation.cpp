#include  <algorithm>
#include  <vector>
#include  <numeric>
#include  <boost/range/algorithm.hpp>
#include  <boost/range/algorithm_ext.hpp>
#include  <boost/accumulators/accumulators.hpp>
#include  <boost/accumulators/statistics.hpp>
#include    "range_extend.hpp"
#include    "evaluation.h"
#include    "key_visitor.hpp"

using namespace std;
double static g_sample_distance = 30.0;

size_t max_variance_dim(PathInfo::SampleVector::iterator begin, PathInfo::SampleVector::iterator end){
    using namespace boost::accumulators;
    accumulator_set<double, stats<tag::lazy_variance> > xacc;
    accumulator_set<double, stats<tag::lazy_variance> > yacc;
    for_each(begin, end, [&xacc, &yacc](PathInfo::SampleVector::const_reference p){
            xacc(p.first.x);yacc(p.first.y);
            });
    return variance(xacc) > variance(yacc)?0:1;
}

path_kdtree_node* build_kdtree(PathInfo::SampleVector::iterator begin, PathInfo::SampleVector::iterator end){
    size_t split = max_variance_dim(begin, end);
    auto nth = begin + distance(begin, end) / 2;
    typedef PathInfo::SampleVector::value_type ValueType;
    nth_element(begin, nth, end, [split](ValueType const& l, ValueType const& r){
            return l.first[split] < r.first[split];
            });

    path_kdtree_node * node = new path_kdtree_node;
    node->point = & nth->first;
    node->segment = nth->second;
    node->split = split;
    if(begin < nth){
        node->left = build_kdtree(begin, nth);
    }

    if( nth +1 < end ){
        node->right = build_kdtree(nth + 1, end);
    }

    return node;
}


static void free_kdtree(path_kdtree_node* node){
    if(node->left)  free_kdtree(node->left);
    if(node->right) free_kdtree(node->right);
    delete node;
}


PathInfo::PathInfo(Path const& path):path(&path){
    size_t id = 0;
    boost::foreach_adjacent(this->path->points, [this, &id](PathPoint const& first, PathPoint const& second){
        if (not first.pos_equal(second)){
            if(first.belong != second.belong){
                //cout << "异常检测@" << id << ", " << id + 1 << endl;
                //cout << first << " pos not equal "<< second << " but not in same road" << endl;
                //cout << "first road : " << first.belong->id << " DBID " << first.belong->dbId << " second road : " << second.belong->id << " DBID " << second.belong->dbId<< endl;
                this->has_exception = true;
            }

            Segment seg{ .first = &first, .second = &second, .length = second.dist_of_path - first.dist_of_path };
            this->segments.push_back(seg);
        }
        ++id;
    });

    for (auto& seg : segments){
        size_t count = ceil(seg.length / g_sample_distance);
        double sample_dist = seg.length / count;
        for (size_t i = 0; i <= count; ++i){
            double t = i * sample_dist / seg.length;
            Point p = *seg.first  + t * (*seg.second - * seg.first);
            samples.emplace_back(p, & seg);
        }
    }

    if (samples.empty()){
        _root = nullptr;
        has_exception = true;
    }
    else _root = build_kdtree(samples.begin(), samples.end());
}


static path_kdtree_node const * nearlest(path_kdtree_node const * root, Point const& p){
    double distance_min = root->point->gis_dist(p);
    size_t split = root->split;
    Point const& point = * root->point;
    path_kdtree_node const* the_nearlest = root;
    if(p[split] == point[split]){
        if (root->left) {
            auto left_nearlest = nearlest( root->left, p );
            double dist_to_left = left_nearlest->point->gis_dist(p);
            if ( dist_to_left < distance_min ){
                the_nearlest = left_nearlest;
                distance_min = dist_to_left;
            }
        }
        if ( root->right ){
            auto right_nearlest = nearlest( root->right, p );
            double dist_to_right = right_nearlest->point->gis_dist(p) ;
            if ( dist_to_right < distance_min ){
                the_nearlest = right_nearlest;
                distance_min = dist_to_right;
            }
        }

        return the_nearlest;
    }

    path_kdtree_node const* other_side = nullptr;
    Point prj(p);
    prj[split] = point[split];
    double distance_to_prj = prj.gis_dist(p);
    if( p[split] < point[split] and root->left ){ 
        auto left_nearlest = nearlest( root->left, p );
        double dist_to_left = left_nearlest->point->gis_dist( p );
        if ( dist_to_left < distance_min ){
            the_nearlest = left_nearlest;
            distance_min = dist_to_left;
        }
        
        if ( distance_to_prj <= dist_to_left){
            other_side = root->right;
        }
    }else if (p[split] > point[split] and root->right ){
        auto right_nearlest = nearlest( root->right, p );
        double dist_to_right = right_nearlest->point->gis_dist( p );
        if ( dist_to_right < distance_min ){
            the_nearlest = right_nearlest;
            distance_min = dist_to_right;
        }
        
        if ( distance_to_prj <= dist_to_right){
            other_side = root->left;
        }
    }

    if ( other_side ){
        auto other_side_node = nearlest( other_side,  p);
        double dist_to_other_side = other_side_node->point->gis_dist(p);
        if ( dist_to_other_side < distance_min ){
            the_nearlest = other_side;
            distance_min = dist_to_other_side;
        }
    }

    return the_nearlest;
}

PathInfo::SampleVector::value_type PathInfo::nearlest_sample(Point const& point)const{
    auto the_nearlest = nearlest( _root, point );
    return make_pair(* the_nearlest->point, the_nearlest->segment );
}

void kd_tree_rough_query(path_kdtree_node const* root, Point const& point, double radious, PathInfo::SampleVector & out){
    if ( root->point->gis_dist(point) <= radious ){
        out.emplace_back (  * root->point , root->segment );
        if ( root->left ){
            kd_tree_rough_query( root->left, point, radious, out );
        }

        if ( root->right ){
            kd_tree_rough_query( root->right, point, radious, out );
        }
    }else {
        Point prj(point);
        size_t split = root->split;
        prj[split] = (* root->point)[split];
        double dist_2_prj = prj.gis_dist( point );
        if ( dist_2_prj <= radious ){
            if ( root->left ) kd_tree_rough_query ( root->left, point, radious, out ); 
            if ( root->right ) kd_tree_rough_query ( root->right, point, radious, out );
        }else if ( point[split] < (* root->point)[split] and root->left ){
            kd_tree_rough_query ( root->left, point, radious, out ); 
        } else if ( point[split] > (* root->point)[split] and root->right ){
            kd_tree_rough_query ( root->right, point, radious, out );
        }
    }
}

PathInfo::SampleVector PathInfo::query ( Point const& point, double radious )const{
    double rough_radious = hypot( radious, g_sample_distance / 2.0 );
    PathInfo::SampleVector rough_result;
    kd_tree_rough_query( _root, point, radious,  rough_result );
    PathInfo::SampleVector result;
    copy_if( rough_result.begin(), rough_result.end(), back_inserter(result), 
            [radious, &point](PathInfo::SampleVector::const_reference seg){
                Point mid;
                return point.dist_to_segment( * seg.second->first , * seg.second->second, mid) <= radious;
            } );
    return result;
}

vector<PathPoint> PathInfo::on_path_point(Point const& point , double distance_mask)const{
    auto nearlest = nearlest_sample( point );
    double min_radious = nearlest.first.gis_dist(point);
    auto rough_result = query( point , min_radious );

    boost::sort( rough_result, [](SampleVector::const_reference l, SampleVector::const_reference r){
            return l.second->first->dist_of_path < r.second->first->dist_of_path;
            } );
    
    vector<PathPoint> result;
    for (auto & sample : rough_result ){

        Segment* segment = sample.second;
        if ( segment->first->pos_equal( point ) ){
            if( segment->first->dist_of_path >= distance_mask ) 
                result.push_back(* segment->first);
        } else if ( &sample == & rough_result.back() and segment->second->pos_equal(point) ){
            if( segment->second->dist_of_path >= distance_mask ) 
                result.push_back(* segment->second);
        } else {
            Point mid;
            double dist = point.dist_to_segment(* segment->first, * segment->second, mid);
            if ( dist == 0.0 ){
                double dist_to_first = segment->first->gis_dist(point);
                double t = dist_to_first / segment->length;
                Point p = * segment->first + t * (*segment->second - *segment->first);
                PathPoint pp;
                pp.x = p.x;
                pp.y = p.y;
                pp.belong = segment->first->belong;
                pp.where = segment->first->where * (1 - t) + t * segment->second->where;
                pp.dist_of_path = segment->first->dist_of_path + dist_to_first;
                if ( pp.dist_of_path >= distance_mask ){
                    result.push_back(pp);
                }
            }
        }
        /*
        if ( point.on_segment(* segment->first, * segment->second) == Point::WHTIN_IN_SEGMENT ){
            double dist_to_first = segment->first->gis_dist(point);
            double t = dist_to_first / segment->length;
            Point p = * segment->first + t * (*segment->second - *segment->first);
            PathPoint pp;
            pp.x = p.x;
            pp.y = p.y;
            pp.belong = segment->first->belong;
            pp.where = segment->first->where * (1 - t) + t * segment->second->where;
            pp.dist_of_path = segment->first->dist_of_path + dist_to_first;
            if ( pp.dist_of_path >= distance_mask )
                result.push_back(std::move(pp));
        }*/
    }

    return result;
}

void Evaluation::walk(PathInfo const& labeld, PathInfo const& test,
            function<void(Segment const&)> not_match_segment_callback,
            function<void(Segment const&)> match_segment_callback
        )const{
    double mask = 0.0;
    for ( size_t i = 0; i < test.segments.size(); ++i ){
        Segment const& segment = test.segments[i];
        auto first_set = labeld.on_path_point(* segment.first , mask);
        auto second_set = labeld.on_path_point(* segment.second, mask);
        if ( first_set.empty() or second_set.empty() ){
            //Segment 不在标签path上
            not_match_segment_callback( segment );
        }else {
            auto & first = * boost::min_element(first_set, binary_of<less>(&PathPoint::dist_of_path));
            auto & second = * boost::min_element(second_set, binary_of<less>(&PathPoint::dist_of_path));
            if ( first.dist_of_path >= second.dist_of_path ){
                //Segment 依旧不在标签path上
                not_match_segment_callback( segment );
            }else{
                //Segment 在标签path上
                mask = second.dist_of_path;
                match_segment_callback(segment);
            }
        }
    }
}

EvaluationResult Evaluation::correct_rate_by_distance(PathInfo const& labeld, PathInfo const& test)const{
    EvaluationResult rst;
    double test_not_match_length = 0.0;
    double labeled_missing_length = 0.0;
    walk(labeld, test, [&test_not_match_length](Segment const& a_not_match_segment){
            test_not_match_length += a_not_match_segment.length;
            }, [](Segment const& a_matched_segment){
            });
    walk(test, labeld, [&labeled_missing_length](Segment const& a_not_match_segment){
            labeled_missing_length += a_not_match_segment.length;
            }, [](Segment const& a_matched_segment){
            });

    rst.test_error = test_not_match_length / test.path->length;
    rst.labeled_error = labeled_missing_length / labeld.path->length;
    rst.all_error = (test_not_match_length + labeled_missing_length) / ( labeld.path->length + test.path->length);
    return rst;
}

size_t kdtree_count(path_kdtree_node const* root ){
    return (root->left ? kdtree_count( root->left ) : 0) + (root->right ? kdtree_count( root->right ) : 0) + 1;
}
size_t PathInfo::count()const{
   return kdtree_count( _root ); 
}
