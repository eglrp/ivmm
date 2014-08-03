#include  <cassert>
#include  <iostream>
#include  <boost/range/adaptors.hpp>
#include  <boost/range/algorithm.hpp>
#include  <boost/range/algorithm_ext.hpp>
#include  <numeric>
#include  <cmath>
#include    "road.h"
#include    "range_extend.hpp"
#include    "key_visitor.hpp"
#include    "json.h"


using namespace std;

double Point::sqr_dist(double x, double y)const{
    return ( this->x - x ) * ( this->x - x ) + ( this->y - y ) * ( this->y - y );
}

double Point::dist_to_segment(double x1, double y1, double x2, double y2, Point & minDistPoint)const{
    double nx, ny; //normal vector

    nx = y2 - y1;
    ny = -( x2 - x1 );

    double t;
    t = ( x * ny - y * nx - x1 * ny + y1 * nx ) / (( x2 - x1 ) * ny - ( y2 - y1 ) * nx );

    if ( t < 0.0 )
    {
        minDistPoint.x =  x1 ;
        minDistPoint.y =  y1 ;
    }
    else if ( t > 1.0 )
    {
        minDistPoint.x =  x2 ;
        minDistPoint.y =  y2 ;
    }
    else
    {
        minDistPoint.x =  x1 + t *( x2 - x1 ) ;
        minDistPoint.y =  y1 + t *( y2 - y1 ) ;
    }

    double dist = gis_dist( minDistPoint );
    //prevent rounding errors if the point is directly on the segment
    if ( double_nearly_equal(dist, 0.0))
    {
        minDistPoint.x =  x ;
        minDistPoint.y =  y ;
        return 0.0;
    }
    return dist;

}


double Point::azimuth( Point const& other )const{
    double dx = other.x - x;
    double dy = other.y - y;
    return ( atan2( dx, dy ) * 180.0 / M_PI );
}

Point::SegmentPostion Point::on_segment(Point const& a, Point const& b)const{
    //algorithm from 'graphics GEMS', A. Paeth: 'A Fast 2D Point-on-line test'
    if (
            abs(( b.y - a.y ) *( x - a.x ) - ( y - a.y ) *( b.x - a.x ) )
            >= max( abs( b.x - a.x ), abs( b.y - a.y ) )
       )
    {
        return OUT_OF_SEGMENT;
    }
    if (( b.x < a.x && a.x < x ) || ( b.y < a.y && a.y < y ) )
    {
        return ON_RAY_OF_A;
    }
    if (( x < a.x && a.x < b.x ) || ( y < a.y && a.y < b.y ) )
    {
        return ON_RAY_OF_A;
    }
    if (( a.x < b.x && b.x < x ) || ( a.y < b.y && b.y < y ) )
    {
        return ON_RAY_OF_B;
    }
    if (( x < b.x && b.x < a.x ) || ( y < b.y && b.y < a.y ) )
    {
        return ON_RAY_OF_B;
    }
    return WHTIN_IN_SEGMENT;
}


Json::Value Point::geojson_coordinates()const{
    return Json::EasyPutValue()
        .append(x).append(y);
}
Json::Value  Point::geojson_geometry()const{
    return Json::EasyPutValue()
        .add("type", "Point")
        .add("coordinates", geojson_coordinates() );
}
Json::Value Point::geojson_properties()const{
    return Json::EasyPutValue()
        .add("x", x)
        .add("y",y);
}

Json::Value Point::geojson_feature()const{
    return Json::EasyPutValue()
        .add("type", "Feature")
        .add("geometry", geojson_geometry())
        .add("properties", geojson_properties() );
}

//========================================================================
Json::Value GpsPoint::geojson_properties()const{
    char time_str[32];
    strftime(time_str, sizeof(time_str), "%Y %m %d %H:%M:%S", localtime( & timestamp ));
    return 
            Json::EasyPutValue()
            .add("timestamp", timestamp)
            .add("time", time_str)
            .add("x", x)
            .add("y", y);
}
//======================================================================
Json::Value Cross::geojson_properties()const{
    return  Json::EasyPutValue()
                .add("id", id)
                .add("dbID", dbId)
                .add("x", x)
                .add("y", y);
}
//=======================================================================
Json::Value CandidatePoint::geojson_properties()const{
    return Json::EasyPutValue()
                .add("roadId", belong->id)
                .add("roadDBID", belong->dbId)
                .add("vote", vote)
                .add("fvalue", fvalue)
                .add("x", x)
                .add("y", y);
}
//=====================================================================
#include  <boost/range/numeric.hpp>
void RoadSegment::init_with_points(vector<Point> const& points){
    length = boost::accumulate(points | boost::adjacented, 0.0, [](double init, boost::tuple<Point const&, Point const&> tup){
            return init + tup.get<0>().gis_dist(tup.get<1>());
            });
    double distanceSum = 0.0;
    boost::copy_adjacent<boost::with_prepare>(points, std::back_inserter(this->points), 
            [this](Point const&){
                CandidatePoint pnt;
                pnt.x = this->begin.x;
                pnt.y = this->begin.y;
                pnt.where = 0.0;
                pnt.belong = this;
                return pnt;
            },
            [&distanceSum, this](Point const& a, Point const& b){
                CandidatePoint pnt;
                pnt.x = b.x;
                pnt.y = b.y;
                pnt.belong = this;
                double distance = a.gis_dist(b);
                distanceSum += distance;
                pnt.where = distanceSum / this->length;
                return pnt;
        });
    
    CandidatePoint& back = this->points.back();
    back.x = this->end.x;
    back.y = this->end.y;
    back.where = 1.0;
}


CandidatePoint RoadSegment::candidate_at_dist(double where)const{
    if(where <= 0){
        return points.front();
    }
    auto iter = boost::lower_bound(points, where, [](CandidatePoint const& c, double value){
            return c.distance_from_begin() < value;
            });
    
    if(iter == points.end())
        return points.back();

    if(iter->distance_from_begin() == where){
        return *iter;
    }

    CandidatePoint const& point = *(iter - 1);
    CandidatePoint const& behind = *iter;
    double distance_sum = point.distance_from_begin();
    double distance = point.gis_dist(behind);
    double t = (where - distance_sum) / distance;
    Point p = point + (behind - point) * t;
    CandidatePoint cp;
    cp.x = p.x;
    cp.y = p.y;
    cp.belong = (RoadSegment*)this;
    cp.where = where / length;
    return cp;
}
vector<CandidatePoint> RoadSegment::path_follow(double begin, double end)const{
    if (double_nearly_equal( begin ,end ))
        return vector<CandidatePoint>();

    using namespace boost::adaptors;
    vector<CandidatePoint> ret;
    bool ordered = begin <= end;
    if(not ordered) swap(begin, end);

    ret.push_back(candidate_at_normal(begin));
    auto lower = boost::lower_bound(points, begin, binary_of<less>(&CandidatePoint::where));
    auto upper = boost::upper_bound(points, end, binary_of<less>(&CandidatePoint::where));
    boost::push_back(ret, boost::make_iterator_range(lower, upper));
    ret.push_back(candidate_at_normal(end));
    ret.resize(boost::unique<boost::return_begin_found>(ret).size());
    if(not ordered){
        boost::reverse(ret);
    }
    return ret;
}

CandidatePoint RoadSegment::candidate_of(Point const& point)const{
    boost::tuple<CandidatePoint, CandidatePoint> min_dist_segment;
    Point min_mid_point;
    double min_distance = numeric_limits<double>::infinity();
    for(auto adj_point : points | boost::adjacented ){
        Point mid;
        double dist = point.dist_to_segment(adj_point.get<0>(), adj_point.get<1>(), mid);
        if(dist < min_distance){
            min_mid_point = mid;
            min_distance = dist;
            min_dist_segment = adj_point;
        }
    }

    double dist_from_begin = min_mid_point.gis_dist(min_dist_segment.get<0>()) + min_dist_segment.get<0>().distance_from_begin();
    return candidate_at_dist(dist_from_begin);
}

Json::Value RoadSegment::geojson_coordinates()const{
    Json::EasyPutValue coordinates;
    for ( auto & c : points ){
        coordinates.append(Json::EasyPutValue().append(c.x).append(c.y));
    }
    return coordinates;
}
Json::Value RoadSegment::geojson_properties()const{
    return Json::EasyPutValue()
                .add("id", id)
                .add("dbId", dbId)
                .add("speed", speed)
                .add("bidir", bidir)
                .add("length", length)
                .add("begin", begin.id)
                .add("beginDBID", begin.dbId)
                .add("end", end.id)
                .add("endDBID", end.dbId);

}

Json::Value RoadSegment::geojson_geometry()const{
    return Json::EasyPutValue()
        .add("type", "LineString")
        .add("geometry",geojson_geometry() );
}

Json::Value RoadSegment::geojson_feature()const{
    return Json::EasyPutValue()
        .add("type", "Feature")
        .add("geometry", geojson_geometry())
        .add("properties", geojson_properties());
}
