#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/program_options.hpp>
#include <boost/range.hpp>
#include <boost/range/algorithm.hpp>
#include "format.h"

#include <proj_api.h>

using namespace std;
struct Point
{
    string time;
    string x;
    string y;
    double mx;
    double my;
};
namespace po = boost::program_options;
namespace fs = boost::filesystem;
int main(int argc, char *argv[])
{

    projPJ pj_wgs84 = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs");
    projPJ pj_mercator = pj_init_plus("+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext  +no_defs");
    fs::path outputDir;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help,h", "show help")
        ("output,o", po::value(&outputDir), "special output directory");

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
        cerr << "need arguement output"<<endl;
        return 1;
    }

    if ( not fs::exists(outputDir) )
    {
        try
        {
            fs::create_directories(outputDir);
        }catch ( fs::filesystem_error const& e)
        {
            cerr << e.what() << endl;
            return 1;
        }
    }
    string line;
    vector<string> split;
    char timeStr[20];
    vector<Point> points;
    list<string> allpaths;
    while ( getline(cin, line))
    {
        allpaths.push_back(std::move(line));
    }

    double allInput = allpaths.size();
    double pd = 0;
    while ( not allpaths.empty() )
    {
        line = std::move(allpaths.front());
        allpaths.pop_front();
        fs::path inputGpsPath(line);
        fs::ifstream intputGpsStream(inputGpsPath);
        if (! intputGpsStream ){
            ++pd;
            continue;
        }

        points.clear();
        while ( getline( intputGpsStream, line ) )
        {
            if (boost::split(split, line, boost::is_any_of(",")).size() == 9)
            {
                tm t;
                strptime(split[3].c_str(), "%Y%m%d%H%M%S", &t);
                strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &t);
                double x = atof(split[4].c_str());
                double y = atof(split[5].c_str());
                const int COUNT_OF_POINT = 1;
                const int DEFAULT_OFFSET = 1;
                x *= DEG_TO_RAD;
                y *= DEG_TO_RAD;
                pj_transform(pj_wgs84, pj_mercator, COUNT_OF_POINT, DEFAULT_OFFSET, &x, &y, nullptr);
                Point p{ timeStr, std::move(split[4]), std::move(split[5]), x, y};
                points.push_back(std::move(p));
            }
        }

        fs::ofstream reformatGpsStream(outputDir / inputGpsPath.filename());
        string id = inputGpsPath.filename().stem().string();
        if ( reformatGpsStream )
        {
            for (auto & e : points)
            {
                fmt::print(reformatGpsStream, "{},{},{},{},{:.2f},{:.2f}\n", id, e.time, e.x, e.y, e.mx, e.my);
            }
        }
        ++pd;
        //cout << "\033[Kprogress:" << setprecision(2) <<  (pd / allInput)*100 << "%\r" << flush;
        printf("progress:%5.2f %%\r", pd / allInput * 100);
        fflush(stdout);
    }
    return 0;
}
