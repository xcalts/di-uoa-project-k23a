/**********************/
/* Standard Libraries */
/**********************/
#include <string>

/**********************/
/* Project Components */
/**********************/
#include "data.h"

Point::Point(int _index, const std::vector<float> &_vec, float _C, float _T)
{
    index = _index;
    dimensions = _vec.size();
    vec = _vec;
    C = _C;
    T = _T;
}

Point::Point(int _index, const std::vector<float> &_vec)
{
    index = _index;
    dimensions = _vec.size();
    vec = _vec;
}

Point::Point() {}

void Point::addNeighbor(int neighbor_index)
{
    // Avoid self loops.
    if (neighbor_index != this->index)
        neighbors.insert(neighbor_index);
}

Query::Query(int _index, const std::vector<float> &_vec, float _query_type, float _v, float _l, float _r)
{
    index = _index;
    dimensions = _vec.size();
    vec = _vec;
    query_type = _query_type;
    v = _v;
    l = _l;
    r = _r;
}

Query::Query(int _index, const std::vector<float> &_vec)
{
    index = _index;
    dimensions = _vec.size();
    vec = _vec;
    query_type = NO_FILTER;
}

Query::Query(const std::vector<float> &_vec)
{
    dimensions = _vec.size();
    vec = _vec;
}

Groundtruth::Groundtruth(int _query_index, const std::vector<int> &_nn_indices)
{
    query_index = _query_index;
    nn_indices = _nn_indices;
}

std::set<int> Groundtruth::getNNSet()
{
    return std::set<int>(nn_indices.begin(), nn_indices.end());
}