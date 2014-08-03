#ifndef  DEBUGER_H
#define  DEBUGER_H

#include  <boost/tuple/tuple.hpp>
#include  <boost/algorithm/string.hpp>
#include  <string>
#include  <fstream>
#include    "network.h"
#include    "road.h"
#include    "json.h"
#include    "range_extend.hpp"


class SimpleDebugDump{
public:
    SimpleDebugDump(){

    }


    SimpleDebugDump& add_point(Point const& p, std::string const& _class = ""){
        auto pp=  p.geojson_properties();
        _features.append(Json::EasyPutValue()
                .add("type", "Feature")
                .add("geometry", p.geojson_geometry())
                .add("properties", Json::EasyPutValue()
                    .add("class", _class)
                    .add("x", p.x)
                    .add("y",p.y)
                    .add("what", p2str(p.geojson_properties()))
                    //p.geojson_properties()["class"] = _class
                    )
                );
        return *this;
    }

    template<typename Seq>
        SimpleDebugDump& add_points(Seq const& points, std::string const& _class = ""){
            for(auto& p : points)
                add_point(p , _class);
            return *this;
        }

    template<typename Seq>
        SimpleDebugDump& add_mutilpoints(Seq const& points, std::string const& _class = ""){
            Json::EasyPutValue  mp;
            mp.add("type", "Feature");
            Json::EasyPutValue ge;
            int i = 0;
            for(Point const& p : points){
                ge.append(p.geojson_geometry());
                ++i;
            }
            mp.add("geometry", Json::EasyPutValue()
                    .add("type", "MultiPoint")
                    .add("coordinates", ge))
                .add("properties",
                        Json::EasyPutValue()
                        .add("class", _class)
                        .add("count", i)
                        .add("what", "")
                    );
            _features.append(mp);
            return *this;
        }

    SimpleDebugDump& add_line(Point const& a, Point const& b, std::string const& _class = ""){
        _features.append(Json::EasyPutValue()
                .add("type", "Feature")
                .add("geometry", Json::EasyPutValue().append(a.geojson_geometry()).append(b.geojson_feature()))
                .add("properties", 
                    Json::EasyPutValue()
                    .add("class", _class)
                    .add("length", a.gis_dist(b))
                    .add("what", "")
                )
             );
        return *this;
    }

        template<typename Seq>
    SimpleDebugDump& add_lines(Seq const& points, std::string const& _class = ""){
        for ( boost::tuple<Point const& , Point const&> tup : points | boost::adjacented ){
            add_line(tup.get<0>(), tup.get<1>(), _class);
        }
        return *this;
    }

    template<typename Seq>
        SimpleDebugDump& add_mline(Seq const& points, std::string const& _class = ""){
            Json::EasyPutValue ml;
            Json::EasyPutValue co;
            double sum = 0;
            int i = 0;
            for( boost::tuple<Point const&, Point const&> tup : points ){
                sum += tup.get<0>().gis_dist(tup.get<1>());
                Json::EasyPutValue line ;
                line.append(tup.get<0>().geojson_coordinates()).append(tup.get<1>().geojson_coordinates());
                co.append(line);
                ++i;
            }
            ml.add("type", "Feature")
                .add("geometry",Json::EasyPutValue()
                        .add("type", "MultiLineString")
                        .add("coordinates", co)
                ).add("properties", Json::EasyPutValue()
                        .add("class", _class)
                        .add("length", sum)
                        .add("count", i)
                        .add("what", "")
                    );
            return *this;
        }

    SimpleDebugDump& add_path_as_line(Path const& path, std::string const& _class = ""){
        _features.append(
        Json::EasyPutValue().add("type", "Feature")
            .add("geometry", path.geojson_geometry())
            .add("properties", 
                    Json::EasyPutValue()
                        .add("class", _class)
                        .add("length", path.length)
                        .add("what", p2str(path.geojson_properties()))
                //path.geojson_properties()["class"] = _class
                ));
        return *this;
    }

    void write(std::string const& name)const{
        std::ofstream o(name);
        if ( ! o ) return;
        o << Json::EasyPutValue()
            .add("type", "FeatureCollection")
            .add("features", _features).toStyledString() << std::endl;
        o.close();
    }
private:
    std::string p2str(Json::Value const& pp){
        std::string str;
        for(auto it = pp.begin(); it != pp.end(); ++it){
            str += it.key().asString() + ":" + Json::FastWriter().write(*it) + " | ";
        }
        boost::replace_all(str, "\n", "");
        return str;
    }
    Json::EasyPutValue _features;
};



#endif  /*DEBUGER_H*/
