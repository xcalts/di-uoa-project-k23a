#ifndef DATA_H
#define DATA_H

/**********************/
/* Standard Libraries */
/**********************/
#include <vector>
#include <set>

/**
 * @brief
 * It represents a point in the dataset.
 */
class Point
{
public:
    /**
     * @brief
     * Index of the point in the dataset.
     */
    int index;

    /**
     * @brief
     * The dimensions of the point's vector.
     */
    int dimensions;

    /**
     * @brief
     * The discretized categorical attribute of the point.
     */
    float C;

    /**
     * @brief
     * The normalized timestamp attribute T of the point.
     */
    float T;

    /**
     * @brief
     * The vector data of the point.
     */
    std::vector<float> vec;

    /**
     * @brief
     * The neighbors of the point.
     */
    std::set<int> neighbors;

    /**
     * @brief
     * Construct a new point.
     * @param _index
     * Index of the point in the dataset.
     * @param _vec
     * The vector data of the point.
     * @param _C
     * The discretized categorical attribute of the point.
     * @param _T
     * The normalized timestamp attribute T of the point.
     */
    Point(int _index, const std::vector<float> &_vec, float _C, float _T);

    /**
     * @brief
     * Construct a new point.
     * @param _index
     * Index of the point in the dataset.
     * @param _vec
     * The vector data of the point.
     */
    Point(int _index, const std::vector<float> &_vec);

    /**
     * @brief
     * Construct a new point.
     */
    Point();

    /**
     * @brief
     * Add a new neighbor to the point.
     * @param neighbor_index
     * The neighbor to add.
     */
    void addNeighbor(int neighbor_index);
};

/**
 * @brief Definition of different filter types based on the query type.
 *
 */
enum QueryType
{
    NO_FILTER = 0,                         // For `query_type == 0`, no filter is applied (only the vector is used).
    CATEGORY_CONSTRAINT = 1,               // For `query_type == 1`, filter for C = v (categorical attribute).
    TIMESTAMP_CONSTRAINT = 2,              // **IGNORE** For `query_type == 2`, filter for l ≤ T ≤ r (timestamp constraint).
    CATEGORY_AND_TIMESTAMP_CONSTRAINTS = 3 // **IGNORE** For `query_type == 3`, filter for both C = v and l ≤ T ≤ r (combined constraints).
};

/**
 * @brief
 * It represents a point that is queried against the dataset.
 */
class Query
{
public:
    /**
     * @brief
     * The index of the query.
     */
    int index;

    /**
     * @brief
     * The dimensions of the query's vector.
     */
    int dimensions;

    /**
     * @brief
     * The type of the query.
     *
     * There are four types of queries, i.e., the `query_type` takes values from `[0, 1, 2, 3]`.
     *
     * The 4 types of queries correspond to:
     *
     * - `0`: Vector-only query, i.e., the conventional approximate nearest neighbor (ANN) search query.
     *
     * - `1`: Vector query with categorical attribute constraint, i.e., ANN search for data points satisfying `C=v`.
     *
     * - `2`: Vector query with timestamp attribute constraint, i.e., ANN search for data points satisfying `l≤T≤r`.
     *
     * - `3`: Vector query with both categorical and timestamp attribute constraints, i.e. ANN search for data points satisfying `C=v` and `l≤T≤r`.
     *
     * The predicate for the categorical attribute is an equality predicate, i.e., `C=v`.
     *
     * And the predicate for the timestamp attribute is a range predicate, i.e., `l≤T≤r`.
     */
    float query_type;

    /**
     * @brief
     * The specific query value `v` for the categorical attribute.
     *
     * If not queried, it takes `-1`.
     */
    float v;

    /**
     * @brief
     * The specific query value `l` for the timestamp attribute.
     * If not queried, it takes `-1`.
     */
    float l;

    /**
     * @brief
     * The specific query value `r` for the timestamp attribute.
     * If not queried, it takes `-1`.
     */
    float r;

    /**
     * @brief
     * The vector data of the point.
     */
    std::vector<float> vec;

    /**
     * @brief
     * Construct a new query.
     * @param idx
     * The index of the query.
     * @param vector_data
     * The vector data of the query.
     * @param _query_type
     * The type of the query.
     * @param _v
     * The specific query value `v` for the categorical attribute.
     * @param _l
     * The specific query value `l` for the timestamp attribute.
     * @param _r
     * The specific query value `r` for the timestamp attribute.
     */
    Query(int _index, const std::vector<float> &_vec, float _query_type, float _v, float _l, float _r);

    /**
     * @brief
     * Construct a new query.
     * @param idx
     * The index of the query.
     * @param vector_data
     * The vector data of the query.
     */
    Query(int _index, const std::vector<float> &_vec);

    /**
     * @brief
     * Construct a new query.
     * @param vector_data
     * The vector data of the query.
     */
    Query(const std::vector<float> &_vec);
};

/**
 * @brief
 * It represents the true nearest neighbors of a query.
 */
class Groundtruth
{
public:
    /**
     * @brief
     * The index of the query.
     */
    int query_index;

    /**
     * @brief
     *
     * The vector containing the indices of the nearest neighbors.
     */
    std::vector<int> nn_indices;

    /**
     * @brief
     * Construct a new `Groundtruth` object.
     * @param query_index
     * The index of the query.
     * @param nn_indices
     * The true nearest neighbors
     */
    Groundtruth(int query_index, const std::vector<int> &nn_indices);

    /**
     * @brief
     * It will return the true nearest neighbors as a set.
     * @return std::set<int>
     */
    std::set<int> getNNSet();
};

#endif // DATA_H