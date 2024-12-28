#ifndef BRUTE_H
#define BRUTE_H

/**********************/
/* Standard Libraries */
/**********************/
#include <vector>
#include <map>

/**********************/
/* External Libraries */
/**********************/

/**********************/
/* Project Components */
/**********************/
#include "data.h"

/**
 * @brief
 * Implementation of the brute force algorithm for kNN search.
 */
/**
 * @brief
 * Implementation of the brute force algorithm for kNN search.
 */
class Brute
{
public:
    /**
     * @brief
     * It holds the k nearest neighbors for each and every one of the query points.
     */
    std::map<int, std::vector<int>> query_gnn;

    /**
     * @brief
     * It contains all the points of the dataset.
     */
    std::vector<Point> dataset;

    /**
     * @brief
     * Construct the brute force algorithm.
     * @param _dataset
     * It contains all the points of the dataset.
     */
    Brute(std::vector<Point> &_dataset);

    /**
     * @brief
     * Get the k nearest neighbors of a given query index from the ground truth as `std::set`.
     * @param query_index
     * The index of the query.
     * @param k
     * The number of nearest neighbors to return.
     * @return std::set<int>
     * The indices of the `k` nearest neighbors.
     */
    std::set<int> getTrueNearestNeighborsAsSet(int query_index, int k);

    /**
     * @brief
     * Get the k nearest neighbors of a given query index from the ground truth.
     * @param query_index
     * The index of the query.
     * @param k
     * The number of nearest neighbors to return.
     * @return std::vector<int>
     * The indices of the `k` nearest neighbors.
     */
    std::vector<int> getTrueNearestNeighbors(int query_index, int k);

    /**
     * @brief
     * Search for kNN using brute force.
     * @param q
     * The query vector to search kNN for.
     * @param k
     * The number of nearest neighbors to search.
     * @return std::vector<int>
     * The indices of the `k` nearest neighbors.
     */
    std::vector<int> bruteForceNearestNeighbors(const Query &q, int k);

    /**
     * @brief
     * Calculate the ground truth kNNs for the dummy queries.
     * @param queries
     * The dummy queries.
     * @param k
     * The number k of nearest neighbors.
     */
    void calculateDummyGroundTruth(const std::vector<Query> &queries, int k);

    /**
     * @brief
     * Save the ground truth kNNs to a file.
     * @param filename
     * The filename to save the ground truth kNNs to.
     */
    void save(const std::string &filename);

    /**
     * @brief
     * Load the ground truth kNNs from a file.
     * @param filename
     * The filename to load the ground truth kNNs from.
     */
    void load(const std::string &filename);
};

#endif