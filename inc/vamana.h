#ifndef VAMANA_H
#define VAMANA_H

/**********************/
/* Standard Libraries */
/**********************/
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

/**********************/
/* Project Components */
/**********************/
#include "data.h"

/**
 * @brief
 * Perfomance metrics of the Vamana algorithm.
 */
class VamanaStatistics
{
public:
    /**
     * @brief
     * The time in seconds of the medoid's calculation.
     */
    std::uint64_t medoid_calculation_time;

    /**
     * @brief
     * The time in seconds of the vamana index algorithm.
     */
    std::uint64_t vamana_indexing_time;

    /**
     * @brief Construct a new `VamanaStatistics` object.
     */
    VamanaStatistics();
};

/**
 * @brief
 * It respresents the Vamana algorithm.
 */
class Vamana
{
public:
    /**
     * @brief
     * It contains all the points of the dataset.
     */
    std::vector<Point> dataset;

    /**
     * @brief
     * The medoid index of the dataset.
     */
    int s;

    /**
     * @brief
     * Construct a new `Vamana` object.
     */
    Vamana();

    /**
     * @brief
     * Generates initial random outgoing edges for each point in the dataset.
     * Each point will have exactly `R` outgoing edges to random other points (excluding itself).
     * @param R
     * The number of outgoing edges (neighbors) each point should have.
     */
    void generateRandomGraphEdges(int R);

    /**
     * @brief
     * It prunes the list of candidate neighbors for a given `Point` based on a
     * pruning condition that involves an `alpha` scaling factor.
     * It ensures that the `Point` has at most R outgoing edges to its nearest neighbors.
     * @param p
     * The point p ∈ P.
     * @param V
     * The candinate set.
     * @param a
     * The distance threshold.
     * @param R
     * The degree bound.
     */
    void robustPrune(const Point &p, std::set<int> &V, float a, int R);

    /**
     * @brief
     * Implementation of the Greedy Search algorithm.
     * @param s
     * The start node.
     * @param x_q
     * The query.
     * @param k
     * The result size.
     * @param L_
     * The search list size L ≥ k.
     * @return std::pair<std::vector<int>, std::vector<int>>
     * A pair of vectors: the list of the `L_size` nearest neighbors and the list of visited nodes.
     */
    std::pair<std::set<int>, std::set<int>> greedySearch(const Point &s, const Query &x_q, int k, int L_);

    /**
     * @brief
     * Creates an in-memory index on the supplied nodes to efficiently answer approximate nearest neighbor queries.
     * The N nodes must already be initialized as a random graph of outgoing edges with maximum of log(N) outgoing edges per node.
     * @param P
     * The dataset of points.
     * @param a
     * Scaling factor to prune outgoing eges of a node (alpha).
     * @param L
     * Maximum list of search candidates to use in graph traversal.
     * @param R
     * Maximum number of outgoing edges of a node. Must be less than log(N) for good results.
     */
    VamanaStatistics index(const std::vector<Point> &P, float a, int L, int R);

    /**
     * @brief
     * Saves the graph to a file.
     * @param filepath
     * The file's path to save the graph.
     */
    void saveGraph(const std::string &filepath);

    /**
     * @brief
     * Loads the graph from a file.
     * @param filepath
     * The file's path to load the graph.
     */
    void loadGraph(const std::string &filepath);
};

#endif // VAMANA_H