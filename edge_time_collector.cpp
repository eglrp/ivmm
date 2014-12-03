#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include  <ctime>
#include <algorithm>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/date_time.hpp>

using namespace std;
namespace fs = boost::filesystem;
namespace po = boost::program_options;

struct Row{
    string rdbID;
    long enterTimestamp;
    long leaveTimestamp;
};

namespace std{
template<>
struct hash< pair<string, string> >{
    size_t operator()(pair<string, string> const& p)const{
        return hash<string>()(p.first) ^ (hash<string>()(p.second) << 1);
    }
};

template<>
struct equal_to<pair<string, string> >{
    bool operator()( pair<string, string> const& a, pair<string, string> const& b)const{
        return a.first == b.first and a.second == b.second;
    }
};
}
struct TimeRecord{
    vector<pair<long, long> > weekend;
    vector<pair<long, long> > weekday;
    int support = 0;
};

typedef unordered_map< pair<string, string>, TimeRecord > EdgeCounter;

void saveToDisk( EdgeCounter & counter, fs::path const& outputDir){

    fs::ofstream outs;
    for ( auto& each : counter ){
        fs::path outputFileWeekend = outputDir /"weekend"/ ( each.first.first + "," + each.first.second + ".txt");
        fs::path outputFileWeekday = outputDir /"weekday"/ ( each.first.first + "," + each.first.second + ".txt");

        if (not each.second.weekday.empty()){
            outs.open(outputFileWeekday, ios::app);
            for (auto& i : each.second.weekday){
                tm t1 = *localtime(&i.first);
                tm t2 = *localtime(&i.second);
                long t = (i.first + i.second) / 2;
                tm t3 = *localtime(&t);
                outs << i.first << "," << i.second << "," << i.second - i.first << ","
                    << ((t1.tm_hour * 3600.0 + t1.tm_min * 60.0 + t1.tm_sec ) / 3600.0) << ","
                    << ((t2.tm_hour * 3600.0 + t2.tm_min * 60.0 + t2.tm_sec) / 3600.0 ) << "\n";
            }
            outs.close();
            each.second.weekday.clear();
        }

        if ( not each.second.weekend.empty() ){
            outs.open(outputFileWeekend, ios::app);
            for (auto& i : each.second.weekend){
                tm t1 = *localtime(&i.first);
                tm t2 = *localtime(&i.second);
                long t = (i.first + i.second) / 2;
                tm t3 = *localtime(&t);
                outs << i.first << "," << i.second << "," << i.second - i.first << ","
                    << ((t1.tm_hour * 3600.0 + t1.tm_min * 60.0 + t1.tm_sec ) / 3600.0) << ","
                    << ((t2.tm_hour * 3600.0 + t2.tm_min * 60.0 + t2.tm_sec) / 3600.0 ) << "\n";
            }
            outs.close();
            each.second.weekend.clear();
        }
    }
}

int main(int argc, char *argv[])
{
    fs::path outputDir;
    fs::path topkFile;
    po::options_description desc(argv[0]);
    long tMax;
    desc.add_options()
        ("help,h", "show help")
        ("output,o", po::value(&outputDir), "special output dir")
        ("topk,k",po::value(&topkFile), "special topk file")
        ("tmax", po::value(&tMax)->default_value(30*60) , "special tmax");

    po::variables_map vm;
    po::store( po::parse_command_line(argc, argv, desc), vm);
    vm.notify();
    if ( vm.count("help")){
        cout << desc << endl;
        return 0;
    }

    if ( vm.count("output") == 0 || vm.count("topk") == 0){
        cerr << desc << endl;
        return 1;
    }

    if ( not fs::exists( outputDir /"weekend")){
        fs::create_directories(outputDir/"weekend");
    }

    if ( not fs::exists( outputDir / "weekday")){
        fs::create_directories(outputDir/"weekday");
    }

    fs::ifstream topkFileStream(topkFile);
    if ( not topkFileStream){
        cerr << "can not open " << topkFile << endl;
        return 1;
    }
    unordered_map<string, int> topkMap;

    string line;
    vector <string> split;
    while( getline(topkFileStream, line)){
        int pos = line.find(',');
        topkMap[line.substr(0, pos)] =  atoi(line.substr(pos+1, line.size() - pos).c_str());
    }
    cout << "load topk file" << endl;

    topkFileStream.close();
    vector<Row> rows;
    EdgeCounter edgeCounter;

    int ioCounter = 0;
    int pathCounter = 0;
    const int  ioMaxCount = 1000 * 100;//~= 400M
    bool inWeekend = false;
    while ( getline(cin, line)){
        inWeekend = false;
        rows.clear();
        pathCounter += 1;
        cout << line << flush;

        fs::path inputPath(line);
        //ifstream inputStream(line);
        string dataStr = inputPath.parent_path().leaf().string();
        tm t;
        strptime(dataStr.c_str(), "%Y-%m-%d", &t);
        if ( t.tm_wday == 6 or t.tm_wday == 0){
            inWeekend = true;
        }

        fs::ifstream inputStream(line);
        while ( getline(inputStream, line)){
            boost::split(split, line, boost::is_any_of(","));
            Row r;
            r.rdbID = std::move(split[0]);
            r.enterTimestamp = atol( split[3].c_str());
            r.leaveTimestamp = atol( split[4].c_str());
            rows.push_back( r );
        }

        vector<Row>::iterator prevHotRoadIter = rows.end();
        for ( auto it = rows.begin(); it != rows.end(); ++it){
            if (topkMap.count( it->rdbID )){
                if (prevHotRoadIter != rows.end()){
                    long cost = it->leaveTimestamp - prevHotRoadIter->enterTimestamp;
                    if ( cost <= tMax){
                        auto& record = edgeCounter[{prevHotRoadIter->rdbID, it->rdbID}];
                        if ( inWeekend ){
                            record.weekend.push_back({prevHotRoadIter->enterTimestamp, it->leaveTimestamp});
                        }else{
                            record.weekday.push_back({prevHotRoadIter->enterTimestamp, it->leaveTimestamp});
                        }
                        record.support += 1;
                        ++ioCounter;
                        if ( ioCounter >= ioMaxCount ){
                            saveToDisk(edgeCounter, outputDir);
                            ioCounter = 0;
                        }
                    }
                }
                prevHotRoadIter = it;
            }
        }
        cout << " done" << endl;
    }
    saveToDisk(edgeCounter, outputDir);

    fs::ofstream out(outputDir / "candidateEdge.txt");
    typedef vector<pair<pair<string,string>, TimeRecord > > Edges;
    vector<pair<pair<string,string>, TimeRecord > > edges(edgeCounter.begin(), edgeCounter.end());
    sort( edges.begin(), edges.end(), [](Edges::value_type const& a, Edges::value_type const& b){
            return a.second.support > b.second.support;
            });
    out << pathCounter << endl;
    for (auto & i : edges ){
        out << i.first.first << "," << i.first.second << "," << i.second.support << "," << (double)i.second.support / pathCounter << "\n";
    }
    return 0;
}
