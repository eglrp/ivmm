#ifndef  SAMPLE_GENERATOR_H
#define  SAMPLE_GENERATOR_H

#include  <random>
#include  <vector>
#include  <boost/range/algorithm_ext/push_back.hpp>
#include    "range_extend.hpp"
#include    "road.h"
#include    "network.h"




//v = C * roadsegment.v + (1 - C) * v_old + E
//C ~ N(speed_similar_mean, speed_similar_stddev)
//E ~ N(0, speed_white_error_variance)
struct LaunchParam{
    double gps_sample_stddev;
    double speed_white_error_stddev;//白噪声方差
    double time_sample_mean;//时间间隔平均值
    double time_sample_stddev;//时间间隔方差
    double speed_confidence;
    double speed_similar;
};


struct SampleOriginalPoint : public PathPoint{
    SampleOriginalPoint()=default;
    SampleOriginalPoint(PathPoint const& p):PathPoint(p){}
    long timestamp;
};


struct SampleResult :boost::equality_comparable<SampleResult>{
    Path path;
    std::vector<SampleOriginalPoint> origin;
    std::vector<GpsPoint> sample;

    bool operator==(SampleResult const& other){
        if( this == &other  ) return true;
        return path == other.path and boost::equal(origin, other.origin) and boost::equal(sample, other.sample);
    }
};


class SampleGenerator{
public:
    SampleGenerator(Network* network, std::random_device::result_type seed):
    network(network),
    _random_engin(seed),
    _seed(seed){}

    SampleGenerator(Network* network):SampleGenerator(network, std::random_device()()){}

    std::vector<SampleResult>
    launch(int begin, int end, int k, LaunchParam const& param);

    template<typename Seq>
    typename std::enable_if<std::is_convertible<typename Seq::value_type, int>::value, SampleResult>::type
    launch(Seq const& seq_of_cross, LaunchParam const& param){
        SampleResult result;
        Path path;
        boost::foreach_adjacent(seq_of_cross,[this,&path](int a, int b){
            auto pth = this->network->shortest_path_Astar(a, b);
            boost::push_back(path.points, pth.points);
                });
        path.update();
        result.origin = sample(path, param);
        result.sample = create(result.origin, param);
        result.path = std::move(path);
        return result;
    }

    std::vector<SampleOriginalPoint> sample(Path const& path, LaunchParam const& param);
    std::vector<GpsPoint> create(std::vector<SampleOriginalPoint> const& org, LaunchParam const& param);
    Network* network;


    std::random_device::result_type seed()const{
        return _seed;
    }
private:
    std::mt19937 _random_engin;
    std::random_device::result_type _seed;
};

#endif  /*SAMPLE_GENERATOR_H*/
