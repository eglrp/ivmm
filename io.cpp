#include <iostream>
#include  <ctime>
#include  <boost/range.hpp>
#include  <boost/range/algorithm.hpp>
#include  <shapefil.h>
#include    "range_extend.hpp"
#include    "io.h"
#include    "key_visitor.hpp"
#include    "pathendpointwalker.hpp"

using namespace std;


GpsPoint toGpsPoint(PreparedGpsRecord const& d){
    GpsPoint p;
    p.x = strtod(d.xStr.c_str(), nullptr);
    p.y = strtod(d.yStr.c_str(), nullptr);
    struct tm tm;
    strptime(d.datatime.c_str(), "%Y-%m-%d %H:%M:%S", &tm);
    p.timestamp = mktime(&tm);
    return p;
}

istream& operator >> (istream & in, PreparedGpsRecord& d ){
    getline(in, d.carID, ',') && 
    getline(in, d.datatime, ',') && 
    getline(in, d.lngStr, ',') && 
    getline(in, d.latStr, ',') &&
    getline(in, d.xStr, ',') &&
    getline(in, d.yStr) && (d.gpsPoint = toGpsPoint(d), true );
    return in;
}
ostream& operator << (ostream& out, PreparedGpsRecord const& d ){
    return out << d.carID << ',' << d.datatime << ',' << d.lngStr << ',' << d.latStr << ',' << d.xStr << ',' << d.yStr;
}

vector<GpsPoint> loadGpsPoints(string const& filename){
    vector<GpsPoint> points;
    ifstream ins(filename);
    if ( ins  ){
        PreparedGpsRecord d;
        while ( ins >> d ){
            points.push_back(d.gpsPoint);
        }
    }
    return points;
}

vector<PreparedGpsRecord> loadPreparedGpsRecord(string const& filename){
    vector<PreparedGpsRecord> ds;
    ifstream ins(filename);
    if ( ins ){
        PreparedGpsRecord d;
        while ( ins >> d ){
            ds.push_back(std::move(d));
        }
    }
    return ds;
}

vector<GpsPoint> loadGpsPoints(vector<PreparedGpsRecord> const& ds){
    vector<GpsPoint> points;
    boost::transform(ds, back_inserter(points), key_of(&PreparedGpsRecord::gpsPoint));
    return points;
}


void saveToShpPoints(std::string const& filename, std::vector<GpsPoint> const& points){
    SHPHandle shp = SHPCreate(filename.c_str(), SHPT_POINT);
    DBFHandle dbf = DBFCreate(filename.c_str());
    DBFAddField(dbf, "x", FTDouble, 20, 6);
    DBFAddField(dbf, "y", FTDouble, 20, 6);
    DBFAddField(dbf, "timestamp", FTInteger, 20, 0);
    DBFAddField(dbf, "time", FTString, 20, 0);
    char buf[20];
    for( auto& p : points ){
        double x = p.x;
        double y = p.y;
        long t = p.timestamp;
        struct tm atm = *localtime(&t);
        strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", &atm);
        SHPObject* shpPoint = SHPCreateSimpleObject(SHPT_POINT, 1, &x, &y, nullptr);
        int const insert = -1;
        int newID = SHPWriteObject(shp, insert, shpPoint);
        DBFWriteDoubleAttribute(dbf, newID, 0, x);
        DBFWriteDoubleAttribute(dbf, newID, 1, y);
        DBFWriteIntegerAttribute(dbf, newID, 2, t);
        DBFWriteStringAttribute(dbf, newID, 3, buf);
        SHPDestroyObject(shpPoint);
    }
    SHPClose(shp);
    DBFClose(dbf);
}


void preparedGpsLogToShpPoints(std::string const& gpsLog, std::string const& filename){
    auto gps = loadGpsPoints(gpsLog);
    saveToShpPoints(filename, gps);
}


void savePathToRawText(Path const& path, string const& filename){
    PathEndpointWalker<Path const> walker(path);
    ofstream outs(filename);
    if ( ! outs  ) return;
    auto makeTime = [](long t){
        struct tm atm = *localtime(&t);
        char buf[20];
        strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", &atm);
        return string(buf);
    };
    while ( !walker.isEnd() ){
        if ( walker.pathpoint().cross() ){
            outs << * walker.pathpoint().cross() << "," << makeTime(walker.pathpoint().timestamp) << "," << walker.pathpoint().timestamp << "\n";
        }
        ++walker;
    }
}

istream& operator >> (istream& in, RawTrajRecord& d){
    getline(in, d.crossDBID, ',') &&
    getline(in, d.datetime, ',') &&
    in >> d.timestamp && in.ignore(numeric_limits<streamsize>::max(), '\n');
    return in;
}

ostream& operator << (ostream& outs, RawTrajRecord const& d){
    return outs << d.crossDBID << ',' << d.datetime << ',' << d.timestamp;
}

RawTraj loadTraj(std::string const& filename){
    RawTraj traj;
    RawTrajRecord d;
    ifstream in(filename);
    while ( in  >> d ){
        traj.push_back(std::move(d));
    }
    return traj;
}

Path trajToPath(Network const& network, RawTraj const& traj){
    Path path;
    boost::foreach_adjacent(traj, [&path, &network](RawTrajRecord const& first, RawTrajRecord const& second){
        Path shortest = network.shortest_path_Astar(network.cross(first.crossDBID), network.cross(second.crossDBID));
        if ( ! shortest.points.empty() ){
            shortest.points.front().timestamp = first.timestamp;
            shortest.points.back().timestamp = second.timestamp;
            path.append(shortest);
        }
    });
    return path;
}

void savePathToShp(std::string const& filename, Path const& path){
    PathEndpointWalker<Path const> walker1(path), walker2(path);
    walker2++;
    SHPHandle shp = SHPCreate(filename.c_str(), SHPT_ARC);
    DBFHandle dbf = DBFCreate(filename.c_str());
    DBFAddField(dbf, "cost", FTInteger, 10, 0);
    while ( ! walker2.isEnd() ){
        int n = distance(walker1.second, walker2.second);
        double x[n];
        double y[n];
        transform(walker1.second, walker2.second, x, key_of(&Point::x));
        transform(walker1.second, walker2.second, y, key_of(&Point::y));
        SHPObject* line = SHPCreateSimpleObject(SHPT_ARC, n, x, y, nullptr);
        long cost = walker2.first->timestamp - walker1.second->timestamp;
        int const insert = -1;
        int newID = SHPWriteObject(shp, insert, line);
        SHPDestroyObject(line);
        DBFWriteIntegerAttribute(dbf, newID, 0, cost);
        ++walker1;
        ++walker2;
    }
    SHPClose(shp);
    DBFClose(dbf);
}

void saveRawTrajToShp(std::string const& filename, Network const& network, RawTraj const& traj){
    Path path = trajToPath(network, traj);
    savePathToShp(filename, path);
}

void saveTraj(std::string const& filename, RawTraj const& traj){
    ofstream outs(filename);
    if ( outs ){
        for (RawTrajRecord const& d : traj){
            outs << d << "\n";
        }
    }
}
