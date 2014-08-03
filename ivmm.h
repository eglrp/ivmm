#ifndef  IVMM_H
#define  IVMM_H

#include  <vector>
#include    "road.h"
#include    "network.h"

struct StaticMatrix{
    StaticMatrix()=default;
    StaticMatrix(size_t ssz, size_t dsz){
        init(ssz, dsz);
    }


    inline void mask_input_except(size_t t){
        _except = t;
        _mask_input = true;
    }

    inline void mask_output_except(size_t t){
        _except = t;
        _mask_output = true;

    }

    inline void restore(){
        _mask_input = _mask_output = false;
    }
    inline double value(size_t s, size_t d)const{
        double _fst = fst(s,d);
        if(_mask_input and d != _except){
            return  - std::numeric_limits<double>::infinity();
        }

        if(_mask_output and s != _except){
            return - std::numeric_limits<double>::infinity();
        }

        return weight * _fst;
    }
    inline size_t row()const{
        return _ft.size();
    }

    inline size_t col()const{
        return _ft[0].size();
    }

    inline double fs(size_t s, size_t d)const{
        return _c[d] * _v[s][d];
    }

    inline double ft(size_t s, size_t d)const{
        return _ft[s][d];
    }

    inline double fst(size_t s, size_t d)const{
        return fs(s, d) * ft(s, d);
    }

    inline void init(size_t ssz, size_t dsz){
        _c.resize(dsz);
        _v.resize(ssz);
        _ft.resize(ssz);
        for(size_t i = 0; i < ssz; ++i){
            _v[i].resize(dsz);
            _ft[i].resize(dsz);
        }
    }
    std::vector<double> _c;
    std::vector< std::vector<double> > _v;
    std::vector< std::vector<double> > _ft;
    double weight = 1.0;
private:
    bool _mask_input = false;
    bool _mask_output = false;
    size_t _except;
};

struct CandidateGraph{
    std::vector<GpsPoint> gps_points;
    std::vector<std::vector<CandidatePoint> > candidates;
    std::vector<std::vector<std::vector<Path> > > paths;
};

struct IVMMParam{
    double candidate_query_radious;
    int candidate_limit;
    double project_dist_mean;
    double project_dist_stddev;
    double beta;
};

class IVMM{
public:
    IVMM(Network* network):_network(network){}

    Path ivmm(std::vector<GpsPoint> const& gps_points, IVMMParam const& param , 
            CandidateGraph * pgraph = nullptr, 
            std::vector<size_t> * pfinal_candidate_index = nullptr)const;

    CandidateGraph
        build_candidate_graph(std::vector<GpsPoint> const& gps_points, IVMMParam const& param)const;

    std::vector<StaticMatrix>
        build_static_matrixs(CandidateGraph const& graph, IVMMParam const& param)const;

    std::vector<CandidatePoint*>
        find_sequence(CandidateGraph & graph, std::vector<StaticMatrix> & matrixs, IVMMParam const& param, size_t i, size_t k)const;

private:
    Network* _network;


};
#endif  /*IVMM_H*/
