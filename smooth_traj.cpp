#include  <boost/program_options.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include  <boost/algorithm/string.hpp>
#include  <boost/iterator_adaptors.hpp>
#include  <iostream>
#include  <string>
#include  <vector>

#include    "road.h"
#include    "network.h"
#include    "format.h"
#include    "range_extend.hpp"

using namespace std;
namespace fs = boost::filesystem;
namespace po = boost::program_options;


struct RawRecord{
    string cdbID;
    string rdbID;
    long timestamp;
};

struct RoadRecord{
    string rdbID;
    string enterCID;
    string leaveCID;
    long enterTimestamp;
    long leaveTimestamp;
    long costTime;
};

template<typename Container>
struct RoadEndIterator : public boost::iterator_adaptor< RoadEndIterator <Container> , typename Container::iterator> {
    RoadEndIterator ( typename Container::iterator it, Container & container ):RoadEndIterator::iterator_adaptor_(it), _container(container){}
    RoadEndIterator ( Container & container):_container(container){
        auto& base = this->base_reference();
        base = find_if( container.begin(), container.end(), [](RawRecord const& r){
                    return not r.cdbID.empty();
                });
    }

    bool atEnd()const{
        return this->base() == _container.end();
    }

private:
    Container & _container;

    friend boost::iterator_core_access;

    void increment(){
        auto& base = this->base_reference();
        while ( not atEnd()){
            ++base;
            if ( atEnd() or not base->cdbID.empty() )
                break;
        }
    }

    void decrement(){
        auto& base = this->base_reference();
        while ( not atEnd()){
            --base;
            if ( atEnd() or not base->cdbID.empty()){
                break;
            }
        }
    }
};

int main(int argc, char *argv[])
{
    fs::path outputdir;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("output,o", po::value(&outputdir), "special the output dir");

    po::variables_map vm;
    po::store( po::parse_command_line(argc, argv, desc),vm);
    vm.notify();

    if (vm.count("help") ){
        cout << desc << endl;
        return 0;
    }

    if ( vm.count("output") == 0){
        cout << desc << endl;
        return 1;
    }

    string line;
    vector <RawRecord> rawRecrods;
    vector <RoadRecord> roadRecords;
    vector <string> splitDate;
    while ( getline(cin, line)){
        if (line.empty() ) continue;
        fs::path inputPath(line);
        cout << inputPath << " " << flush;
        fs::path catalogDir = inputPath.parent_path().leaf();
        fs::path fullDir = outputdir/catalogDir;
        if ( not fs::exists(fullDir)){
            fs::create_directories(fullDir);
        }
        fs::ifstream inputStream( inputPath);
        rawRecrods.clear();
        while ( getline(inputStream, line)){
            boost::split(splitDate, line, boost::is_any_of(","));
            RawRecord r;
            r.cdbID = boost::trim_copy(splitDate[0]);
            r.rdbID = std::move(splitDate[1]);
            r.timestamp = atof( splitDate[2].c_str());
            rawRecrods.push_back(std::move(r));
        }

        RoadEndIterator<vector<RawRecord> > begin(rawRecrods);
        RoadEndIterator<vector<RawRecord> > end(rawRecrods.end(), rawRecrods);

        roadRecords.clear();
        boost::foreach_adjacent(begin, end, [&roadRecords](RawRecord const& r1st, RawRecord const& r2nd){
                if ( r1st.rdbID == r2nd.rdbID and r1st.cdbID != r2nd.cdbID){
                    RoadRecord r;
                    r.rdbID = r1st.rdbID;
                    r.enterCID = r1st.cdbID;
                    r.leaveCID = r2nd.cdbID;
                    r.enterTimestamp = r1st.timestamp;
                    r.leaveTimestamp = r2nd.timestamp;
                    r.costTime = r2nd.timestamp - r1st.timestamp;
                    roadRecords.push_back(std::move(r));
                }
        });

        fs::ofstream outStream(fullDir/ inputPath.filename());
        for ( auto & r : roadRecords){
            outStream << r.rdbID << "," << r.enterCID << "," << r.leaveCID << "," << r.enterTimestamp << "," << r.leaveTimestamp<< "," << r.costTime << endl;
        }
        cout << " done." << endl;
    }
}
