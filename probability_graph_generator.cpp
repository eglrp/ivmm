#include  <iostream>
#include  <unordered_map>
#include  <boost/program_options.hpp>
#include  <boost/filesystem.hpp>
#include    "network.h"
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;

typedef unordered_map<string, int> CrossFrequenceTable;
int main(int argc, char *argv[])
{
    fs::path roadPath;
    fs::path outputPath;
    fs::path crossFreqPath;
    fs::path forwardFreqPath;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h","show help")
        ("road,r", po::value(&roadPath)->required(), "special the road shp file")
        ("output,o", po::value(&outputPath)->required(), "special the output directory")
        ("crossfrequence,c", po::value(&crossFreqPath)->required(), "special the cross freqence file")
        ("forwardFrequence,f", po::value(&forwardFreqPath)->required(), "spceial the forward edge frequence file");
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        vm.notify();
    }catch( po::error const& e )
    {
        if ( vm.count("help") )
        {
            cout << desc << endl;
            return 0;
        }
        cerr << e.what() << endl;
        return 1;
    }

    Network network;
    if ( not  network.load(roadPath.string()) )
    {
       cerr << "can not load network." << endl; 
    }
    cout << "network load success." << endl;

    return 0;
}
