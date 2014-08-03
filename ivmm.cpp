#include  <cmath>
#include  <cassert>
#include  <boost/range/algorithm.hpp>
#include  <boost/range/algorithm_ext.hpp>
#include  <boost/range/numeric.hpp>
#include  <numeric>
#include    "ivmm.h"
#include    "network.h"
#include    "range_extend.hpp"

using namespace std;

template<typename HightDimVec>
void init_dim(HightDimVec & vec, vector<vector<CandidatePoint> > const& candidates){
    vec.resize(candidates.size() - 1);
    for(size_t i = 0; i < candidates.size() - 1;++i){
        vec[i].resize(candidates[i].size());
        for(size_t s = 0; s < candidates[i].size(); ++s){
            vec[i][s].resize(candidates[i+1].size());
        }
    }
}

Path IVMM::ivmm(vector<GpsPoint> const& gps_points, IVMMParam const& param, 
        CandidateGraph * pgraph,
        vector<size_t> * pfinal_candidate_index
        )const{
    CandidateGraph graph = build_candidate_graph(gps_points, param);// build graph
    vector<StaticMatrix> matrixs = build_static_matrixs(graph, param);//build matrix
    vector<CandidatePoint*> path;

    for(size_t i = 0; i < gps_points.size(); ++i){
        for(size_t k = 0; k < graph.candidates[i].size(); ++k){
            //set marsk
            if(i == gps_points.size() - 1){
                matrixs[i-1].mask_input_except(k);
            }else{
                matrixs[i].mask_output_except(k);
            }

            //设置加权系数
            for(size_t m = 0; m < matrixs.size(); ++m){
                size_t other = m;
                if( i == m ) other++;
                double dist = graph.gps_points[i].gis_dist(graph.gps_points[other]);
                double weight = exp( - dist * dist / (param.beta*param.beta) );
                matrixs[m].weight = weight;
            }

            path = find_sequence(graph, matrixs, param, i, k);
            //vote
            for(auto p : path)p->vote++;

            //恢复加权系数
            //restore marsk

            if(i == gps_points.size() - 1){
                matrixs[i-1].restore();
            }else{
                matrixs[i].restore();
            }
        }
    }

    vector<size_t> final_point_index(graph.candidates.size());
    for(size_t i = 0; i < final_point_index.size(); ++i){
        final_point_index[i] = boost::max_element<boost::return_begin_found>(graph.candidates[i],
                [](CandidatePoint const& a, CandidatePoint const& b){
                    if(a.vote == b.vote) return a.fvalue < b.fvalue;
                    return a.vote < b.vote;
                }).size();
    }
    Path final_path;

    //generate path
    for(size_t i = 0; i < final_point_index.size() - 1; ++i){
        //boost::push_back(final_path.points, graph.paths[i][final_point_index[i]][final_point_index[i+1]].points);
        assert( final_path.append(graph.paths[i][final_point_index[i]][final_point_index[i+1]]) );
    }

    final_path.update();

    if ( pgraph ) *pgraph = std::move(graph);
    if ( pfinal_candidate_index ) *pfinal_candidate_index = std::move(final_point_index);
    return final_path;
}




inline static double normal(GpsPoint const& gps, CandidatePoint const& cp, double mean, double stddev){
    static double sqrt_2_pi = sqrt( 2 * M_PI );
    double dist = gps.gis_dist(cp) - mean;
    double dist_sqr = dist * dist;
    double i = - dist_sqr / ( 2 * stddev * stddev );
    double c = 1.0 / (sqrt_2_pi * stddev);
    return c * exp(i);
}


#include  <iostream>
CandidateGraph IVMM::build_candidate_graph(vector<GpsPoint> const& gps_points, IVMMParam const& param)const{
    CandidateGraph graph;
    graph.gps_points = gps_points;
    graph.candidates.resize(gps_points.size());
    double test_r = param.candidate_query_radious;
    for(size_t i = 0; i < gps_points.size(); ++i){
        graph.candidates[i] = _network->query(gps_points[i], test_r, param.candidate_limit);
        while (graph.candidates[i].size() == 0){
            test_r = test_r * 1.1 + 1;
            graph.candidates[i] = _network->query(gps_points[i], test_r, param.candidate_limit);
        }
    }


    init_dim(graph.paths, graph.candidates);

    for(size_t i = 0; i < graph.paths.size(); ++i){
        for(size_t s = 0; s < graph.paths[i].size(); ++s){//source
            for(size_t d = 0; d < graph.paths[i][s].size(); ++ d){//target
                CandidatePoint & begin = graph.candidates[i][s];
                CandidatePoint & end = graph.candidates[i+1][d];
                graph.paths[i][s][d] = _network->shortest_path_Astar(begin ,end);
            }
        }
    }

    return graph;
}


//length, speed
vector<pair<double, double> > segments(Path const& p){
    vector<pair<double, double> > r;
    boost::foreach_adjacent(p.points, [&r](PathPoint const& first, PathPoint const& second){
        if(first.belong == second.belong){
            double d = first.gis_dist(second);
            if( d != 0.0 ){
                double v = first.belong->speed;
                r.emplace_back(d, v);
            }
        }
    });
    return r;
}
vector<StaticMatrix> IVMM::build_static_matrixs(CandidateGraph const& graph, IVMMParam const& param)const{
    vector<StaticMatrix> matrixs(graph.gps_points.size() - 1);
    for(size_t i = 0; i < graph.paths.size(); ++i){
        GpsPoint const& gps_s = graph.gps_points[i];
        GpsPoint const& gps_d = graph.gps_points[i+1];
        matrixs[i].init(graph.candidates[i].size(), graph.candidates[i+1].size());
        double dist = gps_s.gis_dist(gps_d);
        for(size_t s = 0; s < graph.paths[i].size(); ++s){
            for(size_t d = 0; d < graph.paths[i][s].size() ; ++d){
                Path const& p = graph.paths[i][s][d];
                //CandidatePoint const& cs = graph.candidates[i][s];
                CandidatePoint const& cd = graph.candidates[i+1][d];
                double c = normal(gps_d, cd, param.project_dist_mean, param.project_dist_stddev);
                double v = (dist + 1)/ (p.length + 1);
                auto edges = segments(p);
                double v_bar = p.length / (gps_d.timestamp - gps_s.timestamp);
                double vsum = boost::accumulate(edges, 0.0,[v_bar](double init , pair<double, double> const& pa){
                        return init + pa.second * v_bar;
                        });
                double vsum_sqr = boost::accumulate(edges, 0.0,[](double init, pair<double, double> const& pa){
                        return init + pa.second * pa.second;
                        });
                double v_bar_sum_sqr = boost::accumulate(edges, 0.0,[v_bar](double init, pair<double, double>const&){
                        return init + v_bar*v_bar;
                        });
                double ft = vsum / (sqrt(vsum_sqr) + sqrt(v_bar_sum_sqr));
                matrixs[i]._c[d] = c;
                matrixs[i]._v[s][d] = v;
                matrixs[i]._ft[s][d] = ft;
            }
        }
    }

    return matrixs;
}


struct Node{
    typedef shared_ptr<Node> ptr;
    CandidatePoint* point = nullptr;
    ptr pre = nullptr;
    static Node::ptr pointer(CandidatePoint* c, Node::ptr pre = nullptr){
        Node::ptr ptr(new Node);
        ptr->point = c;
        ptr->pre = pre;
        return ptr;
    }
};

#include  <iostream>
vector<CandidatePoint*>
IVMM::find_sequence(CandidateGraph & graph, vector<StaticMatrix> & matrixs, IVMMParam const& param, size_t i, size_t k)const{    
    vector<double> f;
    vector<Node::ptr> points;

    for(size_t i = 0; i < graph.candidates[0].size(); ++i){
        f.push_back(normal(graph.gps_points[0], graph.candidates[0][i], param.project_dist_mean, param.project_dist_stddev));
        points.push_back(Node::pointer(&graph.candidates[0][i]));
    }

    vector<double> tmpf;
    vector<Node::ptr> tmp_points;
    for(size_t m = 0; m < matrixs.size(); ++m){

        assert(matrixs[m].row() == graph.candidates[m].size());
        assert(matrixs[m].col() == graph.candidates[m+1].size());

        tmpf.resize(matrixs[m].col());
        tmp_points.resize(matrixs[m].col());

        for(size_t d = 0; d < matrixs[m].col(); ++d){
            Node::ptr max_pre_node;
            double max_f = - numeric_limits<double>::infinity();
            for(size_t s = 0; s < matrixs[m].row(); ++s){
                double acc_f = matrixs[m].value(s, d) + f[s];
                if(acc_f > max_f){
                    max_f = acc_f;
                    max_pre_node = points[s];
                }
            }
            tmp_points[d] = Node::pointer(&graph.candidates[m+1][d], max_pre_node);
            tmpf[d] = max_f;
        }
        f = std::move(tmpf);
        points = std::move(tmp_points);
    }

    size_t idx = boost::max_element<boost::return_begin_found>(f).size();
    graph.candidates[i][k].fvalue = f[idx];
    vector<CandidatePoint*> path;
    Node::ptr p = points[idx];
    while(p){
        path.push_back(p->point);
        p = p->pre;
    }
    boost::reverse(path);
    return path;
}
