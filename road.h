#ifndef  ROAD_H
#define  ROAD_H
#include  <boost/operators.hpp>
#include  <string>
#include  <functional>
#include  <ostream>
#include  <vector>
#include    "util.h"

namespace Json{
    class Value;
}

class Point:
    boost::equality_comparable1<Point, boost::additive1<Point, boost::multipliable<Point, double> > >{
public:
    double x, y;
    Point()=default;
    inline Point(double x, double y):x(x),y(y){}
    inline bool operator==(Point const & other)const{
        return double_nearly_equal(x, other.x) and double_nearly_equal(y, other.y);
    }

    inline bool pos_equal(Point const& other)const{
        return *this == other;
    }
    inline double sqr_dist(Point const& other)const{
        return sqr_dist(other.x, other.y);
    }

    double sqr_dist(double x, double y)const;

    inline double dist_to_segment(Point const& a, Point const &b, Point & minDistPoint)const{
        return dist_to_segment(a.x, a.y, b.x, b.y, minDistPoint);
    }

    double dist_to_segment(double x1, double y1, double x2, double y2, Point & minDistPoint)const;

    //! Test if this point is on the segment defined by points a, b
    //! @return 0 if this point is not on the open ray through a and b,
    //! 1 if point is on open ray a, 2 if point is within line segment,
    //! 3 if point is on open ray b.
    enum SegmentPostion{
        OUT_OF_SEGMENT,
        ON_RAY_OF_A,
        WHTIN_IN_SEGMENT,
        ON_RAY_OF_B
    };
    SegmentPostion on_segment(Point const& a, Point const& b)const;

    double azimuth( Point const& other )const;

    Point& operator+=(Point const& other){
        x += other.x;
        y += other.y;
        return *this;
    }

    Point& operator-=(Point const& other){
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Point& operator*=(double scale){
        x *= scale;
        y *= scale;
        return *this;
    }

    Point& operator/=(double scale){
        x /= scale;
        y /= scale;
        return *this;
    }

    inline double gis_dist(Point const& other)const{
        return gis_distance(x, y, other.x, other.y);
    }

    double& operator[](int idx){
        if(idx == 0) return x;
        return y;
    }
    double operator[](int idx)const{
        if(idx == 0) return x;
        return y;
    }
    
    friend std::ostream& operator<<(std::ostream& o, Point const& p){
        return o << p.x << ", " << p.y;
    }


    Json::Value geojson_coordinates()const;
    Json::Value geojson_geometry()const;
    Json::Value geojson_feature()const;
    virtual Json::Value geojson_properties()const;
};


class GpsPoint:public Point, boost::equality_comparable1<GpsPoint, boost::equality_comparable<Point, GpsPoint> >{
public:
    long timestamp;
    GpsPoint()=default;
    GpsPoint(double x, double y, long timestamp):Point(x,y),timestamp(timestamp){}


    inline bool operator ==(GpsPoint const& other)const{
        return static_cast<Point const&>(*this) == static_cast<Point const&>(other) and timestamp == other.timestamp;
    }

    Json::Value geojson_properties()const override;
};

class Cross:public Point, boost::equality_comparable<Cross, boost::equality_comparable<Point, Cross> >{
public:
    int  id;
    std::string dbId;
    Cross()=default;
    
    template<typename String>
    Cross(int id, String&& dbid, double x, double y):Point(x,y),id(id), dbId(std::forward<String>(dbid)){}

    bool operator==(Cross const & other)const{
        return id == other.id;
    }

    virtual Json::Value geojson_properties()const;
};

class RoadSegment;


class CandidatePoint:public Point, boost::equality_comparable1<CandidatePoint, boost::equality_comparable<Point, CandidatePoint>>{
public:
    CandidatePoint()=default;

    bool is_vaild()const{
        return belong != nullptr and where >= 0.0 and where <= 1.0;
    }

    RoadSegment* belong;
    double where;//< normal param @[0,1]

    double distance_from_begin()const;
    double distance_to_end()const;
    int vote = 0;
    double fvalue = 0.0;

    bool operator==(CandidatePoint const& other)const{
        return belong == other.belong and this->pos_equal(other);
    }

    friend std::ostream& operator <<(std::ostream& o, CandidatePoint const& c){
        //fmt::print("{}, {}", static_cast<Point>(c), c.where);
        return o << static_cast<Point const&>(c) << " " << c.where ;
    }


    virtual Json::Value geojson_properties()const override;
};

class RoadSegment:boost::equality_comparable<RoadSegment>{
public:
    int id;
    std::string dbId;
    double speed;
    double length;
    bool bidir;
    Cross begin;
    Cross end;
    std::vector<CandidatePoint> points;

    RoadSegment()=default;
    template<typename DBID>
    RoadSegment(int id,  DBID&& dbId, double speed, bool bidir,
            Cross const& begin, Cross const& end, 
            std::vector<Point> const& points):id(id),dbId(std::forward<DBID>(dbId)),speed(speed),
    bidir(bidir), begin(begin),end(end){
        init_with_points(points);
    }

    RoadSegment(RoadSegment&& rd):
        id(rd.id),
        dbId(std::move(rd.dbId)),
        speed(rd.speed),
        length(rd.length),
        bidir(rd.bidir),
        begin(std::move(rd.begin)),
        end(std::move(rd.end)),
        points(std::move(rd.points)){
        for(auto& p : points){
            p.belong = this;
        }
    }
    RoadSegment(RoadSegment const& rd):
        id(rd.id),
        dbId(rd.dbId),
        speed(rd.speed),
        length(rd.length),
        bidir(rd.bidir),
        begin(rd.begin),
        end(rd.end),
        points(rd.points){
            for(auto& p : points){
            p.belong = this;
        }
    }

    RoadSegment& operator=(RoadSegment&& rd){
        id = rd.id;
        dbId = std::move(rd.dbId);
        speed = rd.speed;
        length = rd.length;
        bidir = rd.bidir;
        begin = std::move(rd.begin);
        end = std::move(rd.end);
        points = std::move(rd.points);
        for(auto& p : points){
            p.belong = this;
        }
        return *this;
    }
    RoadSegment& operator=(RoadSegment const& rd){
        id = rd.id;
        dbId = rd.dbId;
        speed = rd.speed;
        length = rd.length;
        bidir = rd.bidir;
        begin = rd.begin;
        end = rd.end;
        points = rd.points;
        for(auto& p : points){
            p.belong = this;
        }
        return *this;
    }

    std::vector<CandidatePoint> path_follow(double begin, double end)const;

    CandidatePoint candidate_at_dist(double where)const;
    inline CandidatePoint candidate_at_normal(double where)const{
        return candidate_at_dist(where * length);
    }

    CandidatePoint candidate_of(Point const& point)const;

    bool operator==(RoadSegment const& other)const{
        return id == other.id;
    }

    Json::Value geojson_coordinates()const;
    Json::Value geojson_geometry()const;
    Json::Value geojson_properties()const;
    Json::Value geojson_feature()const;
private:
    void init_with_points(std::vector<Point>const& points);
};

inline double CandidatePoint::distance_from_begin()const{
    return belong->length * where;
}

inline double CandidatePoint::distance_to_end()const{
    return belong->length * (1-where);
}
#endif  /*ROAD_H*/
