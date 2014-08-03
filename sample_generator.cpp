#include  <boost/range/algorithm.hpp>
#include    "sample_generator.h"


using namespace std;


vector<SampleResult> 
SampleGenerator::launch(int begin, int end, int k, LaunchParam const& param){
    vector<Path> paths = network->k_shortest_path_Yen(begin, end, k);
    vector<SampleResult> results;
    for(auto& p : paths){
        SampleResult one_sample;
        one_sample.path = std::move(p);
        one_sample.origin = sample(one_sample.path, param);
        one_sample.sample = create(one_sample.origin, param);
        results.push_back(std::move(one_sample));
    }
    return results;
}

#include  <iostream>

vector<SampleOriginalPoint>
SampleGenerator::sample(Path const& path, LaunchParam const& param){
    vector<SampleOriginalPoint> path_points;
    normal_distribution<> time_interval(param.time_sample_mean, param.time_sample_stddev);//采样间隔
    normal_distribution<> error(0, param.speed_white_error_stddev);
double v = param.speed_confidence * path.points.front().belong->speed / 3.6;//初始速度
    double distance_sum = 0.0;
    double t = 0;
    double timestamp = 0;
    do{
        SampleOriginalPoint p = path.point_of_dist(distance_sum);
        p.timestamp = timestamp;
        path_points.push_back(p);

        t = time_interval(_random_engin);
        timestamp += t;
        distance_sum += v * t;
        double c = param.speed_similar;
        v = c * p.belong->speed / 3.6 * param.speed_confidence + (1 - c) * v + error(_random_engin);
    }while(distance_sum < path.length);

    return path_points;
}


vector<GpsPoint> SampleGenerator::create(std::vector<SampleOriginalPoint> const& org, LaunchParam const& param){
    vector<GpsPoint> gps_points;
    normal_distribution<> n;
    double degree_test = 0.00001;
    for(auto& p : org){
        Point lng = p;
        lng.x += degree_test;
        Point lat = p;
        lat.y += degree_test;
        double lng_pre_meter = degree_test / lng.gis_dist(p);
        double lat_pre_meter = degree_test / lat.gis_dist(p);
        normal_distribution<>::param_type xp(p.x, lng_pre_meter * param.gps_sample_stddev);
        normal_distribution<>::param_type yp(p.y, lat_pre_meter * param.gps_sample_stddev);
        GpsPoint gps_p;
        gps_p.timestamp = p.timestamp;
        gps_p.x = n(_random_engin, xp);
        gps_p.y = n(_random_engin, yp);
        gps_points.push_back(gps_p);
    }

    return gps_points;
}
