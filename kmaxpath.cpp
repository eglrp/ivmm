#include  <iostream>
#include  <string>
#include  <boost/program_options.hpp>
#include  <boost/range.hpp>
#include  <boost/range/adaptors.hpp>
#include  <boost/filesystem.hpp>
#include  <shapefil.h>
#include    "transitionGraph.h"
using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;
void drawSequence(SequencePair const& seq, string const& output, int n){
    SHPHandle shp = nullptr;
    DBFHandle dbf = nullptr;
    string name = output + "-" + to_string(n) ;
    shp = SHPCreate(name.c_str(), SHPT_ARC);
    dbf = DBFCreate(name.c_str());
    DBFAddField(dbf, "p", FTDouble, 21, 20);
    auto& sequcences = seq.first;
    double p = seq.second;
    vector<double> x;
    vector<double> y;
    for ( auto & each : sequcences){
        if (each->source->road->begin.id == each->source->begin){
            for(auto& point : each->source->road->points){
                x.push_back(point.x);
                y.push_back(point.y);
            }
        }else{
            for(auto& point : each->source->road->points| boost::adaptors::reversed){
                x.push_back(point.x);
                y.push_back(point.y);
            }
        }
    }
    SHPObject* obj = SHPCreateSimpleObject(SHPT_ARC, x.size(), x.data(), y.data(), nullptr);
    int const insert = -1;
    int newID = SHPWriteObject(shp, insert, obj);
    DBFWriteDoubleAttribute(dbf, newID, 0, p);
    SHPDestroyObject(obj);
    SHPClose(shp);
    DBFClose(dbf);
}
int main(int argc, char *argv[])
{
    string road;
    string crossFrequenceTxT;
    string forwardFrequenceTxT;
    fs::path output;
    Network network;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("road,r", po::value(&road)->required()->default_value("../Date/map/bj-road-epsg3785"),"specify the road shp file")
        ("cross_frequence,c", po::value(&crossFrequenceTxT)->required(), "specify the cross frequecne table")
        ("forward_frequence,f", po::value(&forwardFrequenceTxT)->required() , "specify the forward table")
        ("output,o", po::value(&output)->required(),  "specify the output file");

    po::variables_map vm;
    try{
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if ( vm.count("help") ){
            cout << desc << endl;
            return 0;
        }
        vm.notify();
    }catch( std::exception const& err ){
        cerr << err.what();
        cerr << desc << endl;
        return 1;
    }

    if (  ! network.load(road)){
        cerr << "can not load " << road << endl;
        return 1;
    }
    cout << "load road shp file" << endl;
    TransitionGraph tg(network);
    if ( ! tg.load(crossFrequenceTxT, forwardFrequenceTxT) ){
        cerr << "can not load " << crossFrequenceTxT << ' ' << forwardFrequenceTxT << endl;
        return 1;
    }
    cout << "generator transistion graph" << endl;

    if ( ! fs::exists( output ) ){
        fs::create_directories(output);
    }
    string begin = "59567311549";
    string end =   "59567344279";
    tg.uplift();

    while ( cout << "input begin end" << endl, cin >> begin >> end ){
        int n;
        int total = 0;
        if (  !network.contain_cross(begin) || !network.contain_cross(end)){
            continue;
        }
        cout << "generator from " << begin << " to " << end << endl;
        MaxProbabilitySequenceGenerator generator(tg, network.cross(begin).id, network.cross(end).id);
        while ( cin >> n && n > 0){
            SequencePair const* maxPSeq = nullptr;
            for(; n-- && (maxPSeq = generator.next()) != nullptr; ){
                cout << "get one sequence: p = " << maxPSeq->second << endl;
                drawSequence(*maxPSeq, (output/(begin+"-"+end)).string(), ++total);
            }
        }
    }
    return 0;
}
