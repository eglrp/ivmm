#ifndef  UTIL_H
#define  UTIL_H


#include  <cmath>

inline bool double_nearly_equal(double a, double b, double epsilon = 1e-6){
    if(a == b){
        return true;
    }
    double diff = std::abs(a - b);
    return diff < epsilon;
}
inline bool is_equal(double a, double b, double epsilon = 1e-6){
    if(a == b) return true;
    double diff = std::abs(a-b);
    return diff < epsilon;
}

#ifndef M_PI
#define M_PI          3.14159265358979323846
#endif
#define DEG2RAD(x)    ((x)*M_PI/180)
double gis_distance(double x1,double y1, double x2, double y2);


double normal(double dist, double mu, double sigma);


#ifdef DEBUG
    #include <boost/format.hpp>
    #include  <iostream>
    #define debug(fmt) std::cout  << boost::format("\033[31m[%s:%d]:\033[0m") % __FILE__ % __LINE__ << boost::format(fmt"\n")
#else
    struct __none{
        template<typename T>
        __none& operator<<(const T&){ return *this; }
        template<typename T>
        __none& operator%(const T&){ return *this; }
    };
    #define debug(fmt) __none()
#endif

#endif  /*UTIL_H*/
