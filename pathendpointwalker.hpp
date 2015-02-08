#ifndef  PATHENDPOINTWALKER_HPP
#define  PATHENDPOINTWALKER_HPP

#include "network.h"
template<typename PathType>
class PathEndpointWalker
{
public:
    PathEndpointWalker(PathType & path):_path(&path)
    {
        first = _path->points.begin() - 1;
        second = _path->points.begin();
    }

    PathEndpointWalker& operator++()
    {
        while ( first != _path->points.end())
        {
            ++first;
            ++second;
            if (first == _path->points.end() or second == _path->points.end())
                break;
            if(first->cid != -1 && second->cid != -1 && first->belong != second->belong)
                break;
        }
        return *this;
    }
    PathEndpointWalker operator++(int)
    {
        PathEndpointWalker walker(*this);
        ++*this;
        return walker;

    }
    PathEndpointWalker& operator--()
    {
        while( second != _path->points.begin())
        {
            --first;
            --second;
            if ( second == _path->points.begin() )
                break;
            if(first->cid != -1 && second->cid != -1 && first->belong != second->belong)
                break;
        }
        return *this;
    }
    PathEndpointWalker operator--(int)
    {
        PathEndpointWalker walker(*this);
        --*this;
        return walker;
    }
    bool isFirst()const
    {
        return first == _path->points.begin() - 1 and second == _path->points.begin();
    }
    bool isLast()const
    {
        return first == _path->points.end() - 1 and second == _path->points.end();
    }
    bool isEnd()const
    {
        return _path->points.empty() or first == _path->points.end();
    }

    PathPoint pathpoint()const{
        if ( isFirst() ){
            return *second;
        }
        return *first;
    }
    typedef decltype(std::declval<PathType>().points.begin()) iterator;
    iterator first;
    iterator second;
private:
    PathType * _path;
};




#endif  /*PATHENDPOINTWALKER_HPP*/
