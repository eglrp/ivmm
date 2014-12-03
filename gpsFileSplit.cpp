#include <iostream>
#include <vector>
#include <list>
#include <atomic>
#include <deque>
#include <unordered_map>
#include <boost/range.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include "simple_progress.hpp"
#include "simple_guard.hpp"

using namespace std;
namespace fs = boost::filesystem;
namespace po = boost::program_options;

typedef unordered_map<string,  vector< vector<string> > > GpsIDRecordMap;
/*
class GpsRecordQueue{
public:
    typedef vector<GpsIDRecordMap::value_type::second_type> PackageType;
    typedef list< PackageType > QueueAdapterType;
    GpsRecordQueue(QueueAdapterType & queue ):_queue(queue){}
    bool pop(PackageType & ret){
        while ( _lock.test_and_set())
            ;
        if ( _queue.empty() ){
            _lock.clear();
            return false;
        }
        ret = std::move(_queue.front());
        _queue.pop_front();
        _lock.clear();
        return true;
    }

    template<typename T>
    void push(T && p){
        while ( _lock.test_and_set() )
            ;
        _queue.push_back( std::forward<T>(p));
        _lock.clear();
    }
private:
    QueueAdapterType _queue;
    mutable atomic_flag _lock = ATOMIC_FLAG_INIT;
};*/
/*
class Sorter{
public:
    Sorter(GpsRecordQueue& source, GpsRecordQueue & dist):
        _source(source), _dist(dist){}

    void operator()(){
        cout << "sorter thread " <<  boost::this_thread::get_id() << " begin"<<endl;
        run();
    }
protected:
    void run(){
        GpsRecordQueue::PackageType onePackage;
        while ( _source.pop( onePackage ) ){
            for(auto& r : onePackage){
                boost::sort(r , [](vector<string> const& a, vector<string> const& b){
                        return a.at(3) < b.at(3);
                        });
            }
            _dist.push(std::move(onePackage));
        }
    }
private:
    GpsRecordQueue& _source;
    GpsRecordQueue& _dist;
};*/
int main(int argc, char *argv[])
{
    fs::path outputDir;
    int mode;
    int eq;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("output,o", po::value(&outputDir), "output directory")
        ("mode,m", po::value(&mode)->default_value(10), "only dump id % m == e")
        ("equal,e", po::value(&eq)->default_value(0), "only dump id % m == e");
    po::positional_options_description pdesc;
    pdesc.add("output", 1);

    po::variables_map vm;
    try{
        //po::store(po::parse_command_line(argc, argv, desc), vm);
        po::command_line_parser parser(argc, argv);
        po::store(parser.options( desc).positional( pdesc).run(), vm);
    }catch (exception const& e){
        cerr << e.what() << endl;
        return 1;
    }

    vm.notify();

    if ( vm.count("help") ){
        cout << desc << endl;
        return 0;
    }

    if ( ! vm.count("output") ){
        cerr << "require output" << endl;
        return 1;
    }

     if (not fs::exists(outputDir)){
         fs::create_directories( outputDir);
     }

    //char* line;
    vector<string> split;
    string lineString;
    GpsIDRecordMap idRecord;
    idRecord.reserve(1000);
    string id;
    string line;
    //while ( (line = readline("")) ){
    list<string> lines;
    while(getline(cin, line))
        lines.push_back(std::move(line));

    SimpleProgress p(lines.size());
    cout << "read gps for id mode " << mode << " equal " << eq << endl;
    while( not lines.empty()){
        SimpleGuard guard([&p](){++p;});
        line = std::move(lines.front());
        lines.pop_front();
        fs::path massGpsPath( line);

        fs::ifstream massGpsLogStream(massGpsPath);
        if ( massGpsLogStream ){
            while ( getline(massGpsLogStream, lineString )){
                boost::trim(lineString);
                boost::split( split, lineString, boost::is_any_of(","));
                if ( split.size() == 9 and split.at(2) != "2" and split.at(8) != "0"){
                    id = boost::trim_left_copy_if(split.at(0), boost::is_any_of("0 "));
                    long lid = atol(id.c_str());
                        if ( lid % mode == eq ){
                        if ( idRecord.find(id) == idRecord.end() ){
                            idRecord[id].reserve(50);
                        }
                        idRecord[id].push_back( std::move(split) );
                    }
                }
            }
        }
       // free(line);
    }
    p.done();

    //GpsRecordQueue::QueueAdapterType allRecords;
    //GpsRecordQueue::QueueAdapterType ioRecords;


    //auto project2 = [](GpsIDRecordMap::value_type & value)->GpsIDRecordMap::value_type::second_type&{ return (value.second); };
/*    copy(   make_move_iterator(boost::make_transform_iterator(idRecord.begin(), project2)),
            make_move_iterator(boost::make_transform_iterator(idRecord.end(), project2)),*
            back_inserter(allRecords));*/
    //size_t const AllRecors = idRecord.size();
    //int i = 0;
    //GpsRecordQueue::PackageType pkg;
    /*for (auto& p : idRecord ){
        ++i;
        pkg.push_back(std::move(p.second));
        if ( i == packageSize ){
            i = 0;
            allRecords.push_back(std::move(pkg));
            pkg.clear();
        }
    }
    if ( pkg.size() )
        allRecords.push_back(std::move(pkg));*/
    //GpsRecordQueue sorterQueue(allRecords);
    //GpsRecordQueue ioQueue(ioRecords);
    cout << "sort by date " << endl;
    SimpleProgress p2(idRecord.size());
    for (auto & p : idRecord ){
        SimpleGuard g([&p2]{++p2;});
        boost::sort(p.second, [](vector<string> const& a, vector<string> const& b){
                return a.at(3) < b.at(3);
            });
    }
    p2.done();
    /*
    for ( size_t i = 0; i < count; ++i){
        boost::thread t(Sorter(sorterQueue, ioQueue));
    }*/


    //size_t ioCount = 0;
    //GpsIDRecordMap::value_type::second_type oneRecord;
    /*
    GpsRecordQueue::PackageType onePackage;
    while ( ioCount < AllRecors ){
        if ( ioQueue.pop( onePackage )){
            for (auto& r: onePackage ){
                ++ioCount;
                string id = r.at(0).at(0);
                fs::ofstream outs(outputDir/(id +".txt"));
                for ( auto & e : r ){
                    outs << boost::join(e, ",") << "\n";
                }
                outs.close();
            }
        }
    }*/
    for(auto &p : idRecord ){
        fs::ofstream outs(outputDir/(p.first+".txt"));
        for(auto& r: p.second){
            outs << boost::join(r, ",") << "\n";
        }
        outs.close();
    }
    return 0;
}
