#include  <iostream>
#include  <random>
#include  <string>

#include  <boost/program_options.hpp>
#include  <boost/property_tree/ptree.hpp>
#include  <boost/property_tree/ini_parser.hpp>
#include  <boost/property_tree/json_parser.hpp>
#include  <boost/format.hpp>
#include  <boost/filesystem.hpp>
#include  <boost/exception/all.hpp>
#include  <boost/timer.hpp>

#include    "road.h"
#include    "network.h"
#include    "evaluation.h"
#include    "sample_generator.h"
#include    "ivmm.h"
#include    "json.h"
#include    "debuger.hpp"

namespace po = boost::program_options;
namespace pt = boost::property_tree;
namespace fs = boost::filesystem;
using namespace std;


template<typename Seq>
Json::Value geojson_features(Seq const& points){
    Json::EasyPutValue features;
    for(Point const& p : points){
        features.append(p.geojson_feature());
    }

    return Json::EasyPutValue()
        .add("type", "FeatureCollection")
        .add("features", features);
}

void generator_default_ini(char const* filename) throw (pt::ini_parser_error);
LaunchParam load_launch_param(pt::ptree const& ptree);
IVMMParam load_ivmm_param(pt::ptree const& ptree);

int main(int argc, char *argv[])
{

    seed_seq::result_type seed;
    string config_file;
    po::options_description desc(argv[0]);
    desc.add_options()
        ("help", "print help message")
        ("seed", po::value<seed_seq::result_type>(&seed)->default_value(0), "special seed")
        ("config", po::value<string>(&config_file)->default_value("default.ini"), "load config file");

    po::variables_map vm;
    po::store( po::parse_command_line( argc, argv, desc ), vm );
    po::notify ( vm );

    if ( vm.find("help") != vm.end() ){
        cout << desc << endl;
        return 1;
    }

    pt::ptree ptree;

    ifstream ins(config_file);
    if (not ins  ){
        try{
            generator_default_ini( config_file.c_str() );
            cout << config_file << " not exsit , generator a template " << endl;
        } catch ( pt::ini_parser_error  const& err){
            cerr << err.message() << endl;
            return 1;
        }
        return 0;
    }

    try{
        pt::read_ini( ins, ptree );
    } catch ( pt::ini_parser_error const& err ){
        cerr << err.message() << endl;
        ins.close();
        return 1;
    }
    ins.close();

    string cross_filename = ptree.get<string>("Network.cross", "cross");
    string road_filename = ptree.get<string>("Network.road", "road");

    LaunchParam launch_param = load_launch_param(ptree);
    
    string output_dirname = ptree.get<string>("Result.output", "outs");
    int group_count = ptree.get<int>("Result.group_count", 2000);
    int k_shortest = ptree.get<int>("Result.k_shortest", 5);

    IVMMParam ivmm_param = load_ivmm_param(ptree);

    Network network;
    boost::timer loader_timer;
    if (cross_filename.empty()){
        if ( not  network.load(road_filename ) ){
            cerr << boost::format("load [%s] [%s] fail") % road_filename  % cross_filename << endl;
            return 1;
        }

    }else if ( not network.load( road_filename, cross_filename ) ){
    //if ( not  network.load( cross_filename, road_filename )){
        cerr << boost::format("load [%s] fail") % road_filename << endl;
        return 1;
    }
    cout << "load map in " << loader_timer.elapsed() << " second" << endl;


    if ( seed == 0 ){
        seed = random_device()();
    }

    cout << "seed is " << seed << endl;
    SampleGenerator generator(& network, seed);

    mt19937 random_engin(seed);
    uniform_int_distribution<> uniform(0, network.cross_bound());

    auto random_two_num = [&random_engin, &uniform](){
        int a = uniform(random_engin);
        int b = uniform(random_engin);
        while ( a == b ){
            a = uniform(random_engin);
            b = uniform(random_engin);
        } 
        return make_pair(a, b);
    };

    IVMM ivmm(&network);
    fs::path output_dir(output_dirname);
    fs::path output_label_dir = output_dir / "label";
    fs::path output_sample_dir = output_dir / "sample";
    fs::path output_ivmm_dir = output_dir / "ivmm";
    fs::path output_exception = output_dir / "exception";

    try {
        if ( not fs::exists(output_dir) ){
            fs::create_directories(output_dir);
        }

        if ( not fs::exists(output_label_dir) ){
            fs::create_directories(output_label_dir);
        }

        if ( not fs::exists(output_sample_dir) ){
            fs::create_directories(output_sample_dir);
        }

        if ( not fs::exists(output_ivmm_dir) ){
            fs::create_directories(output_ivmm_dir);
        }

        if ( not fs::exists(output_exception) ){
            fs::create_directories(output_exception);
        }
    } catch ( fs::filesystem_error const& err ){
        cerr << err.what() << endl;
        return 1;
    }

    Evaluation evaluation(&network);
    boost::format file_name_formater("%d_%d.geojson");
    string filename;
    Path ivmm_path;
    Json::FastWriter geojson_string_writer;

    string report = (output_dir / "report").string();
    ofstream report_outs(report);
    CandidateGraph graph;
    vector<size_t> best_index;
    for (int group = 0; group < group_count; ++ group){
        int cross_a;
        int cross_b;
        tie(cross_a, cross_b) = random_two_num();
        vector<SampleResult> results = generator.launch ( cross_a, cross_b, k_shortest, launch_param );
        while ( results.empty() ){
            tie(cross_a, cross_b) = random_two_num();
            results = generator.launch ( cross_a, cross_b, k_shortest, launch_param );
        }

        for (int k = 0; k < results.size(); ++k){
            //SimpleDebugDump debug;
            filename = (file_name_formater % group % k).str();

            ofstream out_label((output_label_dir / filename).string());
            out_label << results[k].path.geojson_feature().toStyledString() << endl;
            out_label.close();


            ivmm_path = ivmm.ivmm( results[k].sample, ivmm_param);// , & graph, & best_index);

            /*
            for(size_t i = 0; i < graph.paths.size(); ++i){
                for (size_t j = 0; j < graph.paths[i].size(); ++j){
                    for(size_t k = 0; k < graph.paths[i][j].size(); ++k){
                        debug.add_path_as_line( graph.paths[i][j][k], (boost::format("path:[%d] %d to [%d] %d") % i % j % (i+1) % k).str() );
                    }
                }
            }

            for(size_t i = 0; i < graph.gps_points.size(); ++i){
                for (size_t j = 0; j < graph.candidates[i].size(); ++j){
                    debug.add_point(graph.candidates[i][j], (boost::format("cp[%d]:%d")%i%j).str());
                }
            }

            vector<CandidatePoint> best_cps;
            for(size_t i = 0; i < best_index.size(); ++i){
               best_cps.push_back( graph.candidates[i][best_index[i]] ); 
            }

            //debug.add_mutilpoints(best_cps, "best candidate");

            debug.write(filename);*/


            ofstream out_sample((output_sample_dir/ filename).string());
            out_sample << geojson_features( results[k].sample ).toStyledString() << endl;
            out_sample.close();

            /*cout << "feature"<<endl;
            cout << geojson_features(results[k].sample).toStyledString() << endl;
            cout << "pass to continue" << endl;cin.get();*/


            ofstream out_ivmm( (output_ivmm_dir / filename ).string());
            out_ivmm << ivmm_path.geojson_feature().toStyledString() << endl;
            out_ivmm.close();
            PathInfo pathinfo1(results[k].path);
            PathInfo pathinfo2(ivmm_path);
            if (pathinfo1.has_exception){
                cout << "label path has exception" << endl;
            }
            if (pathinfo2.has_exception){
                cout << "sample empty?:" << results[k].sample.empty() << endl;
                cout << "ivmm path has exception" << endl;
            }
            if ( pathinfo1.has_exception or pathinfo2.has_exception ){
                ofstream ex1((output_exception/("exception_path_"+filename)).string());
                ex1 << pathinfo1.path->geojson_feature().toStyledString() << endl;
                ex1.close();
                ofstream ex2((output_exception/("exception_ivmm_" + filename)).string());
                ex2 << pathinfo2.path->geojson_feature().toStyledString() << endl;
                ex2.close();
                ofstream p((output_exception/("exception_points_"+ filename)).string());
                p << geojson_features(pathinfo2.path->points) << endl;
                p.close();
                continue;
            }
            EvaluationResult rst = evaluation.correct_rate_by_distance(pathinfo1, pathinfo2);

            //cout << "[report]" << group << "_" << k << " " << rst.labeled_error << "," << rst.test_error << "," << rst.all_error << "\n";
            report_outs << filename << "," << rst.labeled_error << "," << rst.test_error << "," << rst.all_error << "\n";

        }
    }
    report_outs.close();
    return 0;
}

void generator_default_ini(char const* filename)throw(pt::ini_parser_error){
    pt::ptree ini;
    ini.put("Network.cross", "cross");
    ini.put("Network.road", "road");
    ini.put("LaunchParam.speed_white_error_stddev", 3);
    ini.put("LaunchParam.time_sample_mean", 60);
    ini.put("LaunchParam.time_sample_stddev", 15);
    ini.put("LaunchParam.speed_confidence", 0.6);
    ini.put("LaunchParam.speed_similar", 0.7);
    ini.put("LaunchParam.gps_sample_stddev", 50);
    ini.put("Result.output", "outs");
    ini.put("Result.group_count", 2000);
    ini.put("Result.k_shortest", 5);
    ini.put("IVMM.candidate_limit", 5);
    ini.put("IVMM.candidate_query_radious", 100);
    ini.put("IVMM.project_dist_mean", 40);
    ini.put("IVMM.project_dist_stddev", 35.0);
    ini.put("IVMM.beta", 5000);
    pt::write_ini(filename, ini);
}

LaunchParam load_launch_param(pt::ptree const& ptree){
    LaunchParam launch_param;
    launch_param.speed_similar = ptree.get<double>("LaunchParam.speed_white_error_stddev", 3);
    launch_param.time_sample_mean = ptree.get<double>("LaunchParam.time_sample_mean", 60);
    launch_param.time_sample_stddev = ptree.get<double>("LaunchParam.time_sample_stddev", 15);
    launch_param.speed_confidence = ptree.get<double>("LaunchParam.speed_confidence", 0.6);
    launch_param.speed_similar = ptree.get<double>("LaunchParam.speed_similar", 0.7);
    launch_param.gps_sample_stddev = ptree.get<double>("LaunchParam.gps_sample_stddev", 50);
    return launch_param;
}


IVMMParam load_ivmm_param(pt::ptree const& ptree){
    IVMMParam ivmm_param;
    ivmm_param.candidate_limit = ptree.get<int>("IVMM.candidate_limit", 5); 
    ivmm_param.project_dist_mean = ptree.get<double>("IVMM.project_dist_mean", 40);
    ivmm_param.project_dist_stddev = ptree.get<double>("IVMM.project_dist_stddev", 35.0);
    ivmm_param.candidate_query_radious = ptree.get<double>("IVMM.candidate_query_radious",100);
    ivmm_param.beta = ptree.get<double>("IVMM.beta", 5000);
    return ivmm_param;
}
