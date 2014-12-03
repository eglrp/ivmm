#include  <iostream>
#include  <string>
#include  <list>
#include  <vector>
#include  <unordered_map>
#include  <fstream>
#include  <boost/range.hpp>
#include  <boost/range/algorithm.hpp>
#include  <boost/program_options.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/filesystem/fstream.hpp>
#include    "simple_progress.hpp"
#include    "simple_guard.hpp"
#include    "range_extend.hpp"
#include    "key_visitor.hpp"
using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct GreaterBySecond
{
    template<typename T>
    bool operator() (T const & t1, T const & t2)const
    {
        return t1.second > t2.second;
    }
};


typedef pair<string, string> CrossForward;
namespace std
{
    template<>
    struct hash<CrossForward>:unary_function<CrossForward const&, size_t>
    {
        size_t operator()(CrossForward const& fwd)const
        {
            hash<string> hasher;
            size_t hash1 = hasher(fwd.first);
            size_t hash2 = hasher(fwd.second);
            return hash1 ^ (hash2 << 1);
        }
    };
    template<>
    struct equal_to<CrossForward>:binary_function<CrossForward const&, CrossForward const&, bool>
    {
        bool operator()(CrossForward const& f1, CrossForward const& f2)const
        {
            return f1.first == f2.first and f1.second == f2.second;
        }
    };
}
int main(int argc, char *argv[])
{
    fs::path outputDir;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("output,o", po::value(&outputDir), "special the output directory");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    vm.notify();

    if ( vm.count("help") )
    {
        cout << desc << endl;
        return 0;
    }

    if ( vm.count("output") == 0)
    {
        cerr << "need output directory" << endl;
        return 1;
    }

    if ( not fs::exists(outputDir) )
    {
        try
        {
            fs::create_directories(outputDir);
        }catch(fs::filesystem_error const& err)
        {
            cerr << err.what() << endl;
            return 1;
        }
    }
    
    string line;
    list<string> lines;
    while(getline(cin, line))
    {
        lines.push_back(std::move(line));
    }
    
    unordered_map<string, int> crossFrequence;
    unordered_map<CrossForward, int> forwardFrequence;
    SimpleProgress progress(lines.size());
    cout << "statistics : " << endl;
    while(not lines.empty())
    {
        SimpleGuard guard([&progress](){ ++progress; });
        line = std::move(lines.front());
        lines.pop_front();
        ifstream inputStream(line);
        vector<string> crossDBIDs;
        if ( not inputStream  )
            continue;
        while(getline(inputStream, line))
        {
            crossDBIDs.push_back(line.substr(0,line.find_first_of(',')));
        }

        boost::foreach_adjacent<boost::with_prepare>(crossDBIDs,[&](string const& first){
                ++crossFrequence[first];
            },[&](string const& from, string const& to){
                ++crossFrequence[to];
                if ( from != to )
                    ++forwardFrequence[ { from, to } ];
            });
    }
    progress.done();
    
    vector< pair <string , int > > crossFrequenceSorterVec(boost::begin(crossFrequence), boost::end(crossFrequence));
    boost::sort(crossFrequenceSorterVec, GreaterBySecond());
    cout << "write to file " << endl;
    SimpleProgress writeProgress(forwardFrequence.size() + crossFrequence.size());
    fs::ofstream crossFrequenceStream(outputDir/"crossFrequence.txt");

    if ( not crossFrequenceStream )
    {
        cerr << "can not write to file " << outputDir/"crossFrequence.txt" << endl;
        return 0;
    }
    for(auto & p : crossFrequenceSorterVec)
    {
        crossFrequenceStream << p.first << "," << p.second << "\n";
        ++writeProgress;
    }

    vector< pair < CrossForward , int> > forwardFrequenceSorterVec(forwardFrequence.begin(), forwardFrequence.end());
    boost::sort(forwardFrequenceSorterVec, GreaterBySecond());
    fs::ofstream forwardFrequenceStream(outputDir/"forwardFrequence.txt");
    if ( not forwardFrequenceStream )
    {
        cerr << "can not write to file " << outputDir/"forwardFrequence.txt" << endl;
        return 1;
    }
    for(auto & p : forwardFrequenceSorterVec)
    {
        forwardFrequenceStream << p.first.first << "," << p.first.second << "," << p.second << endl;
        ++writeProgress;
    }
    writeProgress.done();
    return 0;
}
