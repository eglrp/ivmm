#include <string>
#include <list>
#include <iostream>
#include <ctime>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/iterator.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/bind.hpp>

#include "road.h"
#include "simple_progress.hpp"
#include "simple_guard.hpp"
#include "range_extend.hpp"
#include "format.h"

using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;


struct LatLngGpsPoint:GpsPoint
{
    LatLngGpsPoint(double lng,double lat, double x, double y, long timestamp):
        GpsPoint(x, y, timestamp),lat(lat),lng(lng){}
    double lat;
    double lng;
};

template<template<typename T, typename Alloc>class Container = list>
Container<LatLngGpsPoint, std::allocator<LatLngGpsPoint> > readGpsFromFile(ifstream & ins)
{
    string line;
    static vector<string> split;
    typedef Container<LatLngGpsPoint, std::allocator<LatLngGpsPoint> > GpsContainer;
    GpsContainer gpsContainer;
    while ( getline(ins, line))
    {
        boost::split(split, line, boost::is_any_of(","));
        if ( split.size() == 6){
            double x = atof(split[4].c_str());
            double y = atof(split[5].c_str());
            double lng = atof(split[2].c_str());
            double lat = atof(split[3].c_str());
            tm t;
            strptime(split[1].c_str(), "%Y-%m-%d %H:%M:%S", &t);
            long timestamp = mktime(&t);
            gpsContainer.push_back(LatLngGpsPoint(lng, lat, x, y, timestamp) );
        }
    }

    return gpsContainer;
}

pair<double, double> speedAndDist(LatLngGpsPoint const& p1, LatLngGpsPoint const& p2)
{
    double dist = p2.gis_dist(p1);
    double speed = dist / (p2.timestamp - p1.timestamp);
    return { speed , dist};
}

void removeSameAdjTimestampPoints(list<LatLngGpsPoint> & inputGps)
{
    auto it = inputGps.begin();
    auto it2 = next(it,1);
    for(; it2 != inputGps.end(); )
    {
        if (it->timestamp == it2->timestamp)
        {
            inputGps.erase(it);
        }
        it = it2;
        ++it2;
    }
}

list<list<LatLngGpsPoint> > preSplit(list<LatLngGpsPoint> & inputGps, long timeSplit, size_t minWriteGpsSize)
{
    list<list<LatLngGpsPoint> > splits;
    auto it = inputGps.begin();
    auto it2 = next(it,1);
    for(; it2 != inputGps.end(); ++it2)
    {
        if ( it2->timestamp - it->timestamp > timeSplit)
        {
            list<LatLngGpsPoint> l;
            l.splice(l.begin(), inputGps, inputGps.begin(),it2) ;
            if ( l.size() >= minWriteGpsSize )
                splits.push_back(std::move(l));
        }
        it = it2;
    }
    if ( inputGps.size() >= minWriteGpsSize )
        splits.push_back(std::move(inputGps));
    return splits;
}

void removeOutlinePoints(list<LatLngGpsPoint> & gpsList, double outlineSpeedLimit)
{
    auto it = gpsList.begin();
    auto it2 = next(it, 1);
    for( ; it2 != gpsList.end(); )
    {
        double dist = it2->gis_dist(*it);
        long t = it2->timestamp - it->timestamp;
        double speed = dist / t;
        if ( speed > outlineSpeedLimit)
        {
            it2 = gpsList.erase(it2);
            it = prev(it2, 1);
        }else{
            it = it2;
            it2++;
        }
    }
}

void removeParkPoints(list<LatLngGpsPoint> & inputGps, int window, double parkFindMinAvgSpeed, double parkFindDistRate)
{
    list<list<LatLngGpsPoint>::iterator > deletePos;
    auto first = inputGps.begin();
    auto last = next(first , window);
    double sumOfSpeed = 0.0;
    double sumOfDist = 0.0;
    int windowOffset = 0;
    int lastDeleteIterIndex = -1;
    boost::foreach_adjacent(first, last, [&sumOfSpeed, &sumOfDist](LatLngGpsPoint const& p1st, LatLngGpsPoint const& p2nd)
            {
                double speed, dist;
                tie(speed, dist) = speedAndDist( p1st, p2nd);
                sumOfSpeed += speed;
                sumOfDist += dist;
            });
    double avgSpeed = sumOfSpeed / ( window - 1);
    double distDirect = prev(last)->gis_dist(*first);
    if ( avgSpeed < parkFindMinAvgSpeed && distDirect / sumOfDist < parkFindDistRate )
    {
        for(auto it = first; it != last; ++it)
        {
            deletePos.push_back(it);
        }
        lastDeleteIterIndex = window - 1;
    }

    if ( last != inputGps.end())
    {
        ++first;
        ++last;
        ++windowOffset;
        while ( last != inputGps.end() )
        {
            double speed, dist;
            tie(speed, dist) = speedAndDist(*prev(first,1), *first);
            sumOfDist -= dist;
            sumOfSpeed -= speed;
            tie(speed, dist) = speedAndDist(*prev(last,2), *prev(last,1));
            sumOfDist += dist;
            sumOfSpeed += speed;
            double avgSpeed = sumOfSpeed / ( window - 1);
            double distDirect = prev(last,1)->gis_dist(*first);
            if ( avgSpeed < parkFindMinAvgSpeed && distDirect / sumOfDist < parkFindDistRate )
            {
                int offset = lastDeleteIterIndex - windowOffset + 1;
                if ( offset < 0 )  offset = 0;
                for(auto it = next(first, offset); it != last; ++it)
                {
                    deletePos.push_back(it);
                }
                lastDeleteIterIndex = windowOffset + window - 1;
            }
            ++first;
            ++last;
            ++windowOffset;
        }
    }
    for(auto& eachIter : deletePos )
    {
        inputGps.erase(eachIter);
    }
}
void splitGpsList(list<LatLngGpsPoint> const& gpsList, fs::path const& outputDir, string const& prefix, long splitTime, size_t logSize)
{
    auto last = gpsList.begin();
    auto it = gpsList.begin();
    auto it2 = next(it, 1);
    typedef list<LatLngGpsPoint>::const_iterator iter;
    list<pair<iter, iter> > split;
    for(; it2 != gpsList.end(); ++it, ++it2)
    {
        if ( it2->timestamp - it->timestamp > splitTime)
        {
            split.push_back({last, it2});
            last = it2;
        }
    }
    split.push_back({last, gpsList.end()});
    int i = 0;
    char timeStr[20];
    for(auto& p : split)
    {
        if (distance(p.first, p.second) >= (long)logSize )
        {
            if ( ! fs::exists(outputDir / prefix))
            {
                fs::create_directories(outputDir / prefix);
            }
            fs::ofstream outputStream(outputDir/prefix/ (prefix + "-" + to_string(i) + ".txt") );
            ++i;
            for( ; p.first != p.second; ++p.first)
            {
                LatLngGpsPoint const& pnt = *p.first;
                tm t = *localtime(&pnt.timestamp);
                strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &t);
                fmt::print(outputStream, "{},{},{:.6f},{:.6f},{:.2f},{:.2f}\n",prefix, timeStr, pnt.lng,pnt.lat,pnt.x, pnt.y);
            }
            outputStream.close();
        }
    }
}

int main(int argc, char *argv[])
{
    fs::path outputDir;
    size_t window;
    double parkFindMinAvgSpeed;
    double parkFindDistRate;
    size_t minWriteGpsSize;
    long splitTime;
    double outlineSpeedLimit;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("output,o", po::value(&outputDir), "special output dir")
        ("window,w", po::value(&window)->default_value(30), "window to find park")
        ("parkFindMinAvgSpeed,s", po::value(&parkFindMinAvgSpeed)->default_value(1), "min avg speed to find park")
        ("parkFindDistRate,r", po::value(&parkFindDistRate)->default_value(0.2), "min direct dist / all dist rate to find park")
        ("minWriteGpsSize,z", po::value(&minWriteGpsSize)->default_value(30), "min size to write ")
        ("split,p", po::value(&splitTime)->default_value(1800), "if time interval > split time, log will split'")
        ("outlineSpeedLimit", po::value(&outlineSpeedLimit)->default_value(30), "outline speed limit");

    po::variables_map vm;
    try
    {
        po::store( po::parse_command_line(argc, argv, desc), vm);
    }catch( po::error const& e)
    {
        cerr << e.what() << endl;
        return 1;
    }
    vm.notify();

    if ( vm.count("help")){
        cout << desc << endl;
        return 0;
    }

    if ( vm.count("output") == 0){
        cerr << "need output " << endl;
        cerr << desc << endl;
        return 1;
    }

    if ( ! fs::exists(outputDir ))
    { boost::system::error_code err; fs::create_directories(outputDir, err);
        if ( err )
        {
            cerr << err.message() << endl;
            return 1;
        }
    }

    string line;
    vector<string> split;
    list<LatLngGpsPoint> inputGps;

    list<string> allInputs;
    while ( getline(cin, line) ){
        allInputs.push_back(std::move(line));
    }


    SimpleProgress progress(allInputs.size(),0.2);
    auto notInBound = [](GpsPoint const& p){
        return ! (p.x > 12872019 && p.x < 13071847 && p.y > 4813694 && p.y < 4942839 );
    };



    while ( ! allInputs.empty() )
    {
        SimpleGuard guard([&progress](){++progress;});

        line = std::move(allInputs.front());
        allInputs.pop_front();
        fs::path inputGpsPath(line);
        fs::ifstream inputGpsStream(inputGpsPath);
        if ( ! inputGpsStream)
        {
            continue;
        }
        inputGps = readGpsFromFile<list>(inputGpsStream);
        inputGps.remove_if(notInBound);
        removeSameAdjTimestampPoints(inputGps);
        if ( inputGps.size() < minWriteGpsSize)
        {
            continue;
        }
        list<list<LatLngGpsPoint> > preSplitLists = preSplit(inputGps, splitTime, minWriteGpsSize);
        list<list<LatLngGpsPoint> > finalLists;
        for(auto it = preSplitLists.begin(); it != preSplitLists.end(); ++it)
        {
            removeOutlinePoints(*it, outlineSpeedLimit);
            removeParkPoints(*it, min(window, it->size()), parkFindMinAvgSpeed, parkFindDistRate);
            finalLists.splice(finalLists.end(), preSplit(*it, splitTime, minWriteGpsSize));
        }

        string prefix = inputGpsPath.filename().stem().string();
        fs::path dir = outputDir/ prefix;
        if ( !fs::exists(dir) )
        {
            fs::create_directories(dir);
        }
        int i = 0;
        char timeStr[20];
        tm t;
        for (auto const& gps : finalLists )
        {
            fs::ofstream outputStream(dir/(prefix+"-" + to_string(i) + ".txt"));
            for (auto const& point : gps)
            {
                t = * localtime(&point.timestamp);
                strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &t);
                fmt::print(outputStream, "{},{},{:.6f},{:.6f},{:.2f},{:.2f}\n", prefix,
                        timeStr, point.lng, point.lat, point.x, point.y);
            }
            ++i;
        }
    }
    progress.done();
    return 0;
}
