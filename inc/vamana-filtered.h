/**********************/
/* Standard Libraries */
/**********************/
#include <map>
#include <string>
#include <unordered_set>
#include <vector>
#include <cstdint>

/**********************/
/* Project Components */
/**********************/
#include "data.h"

/**
 * @brief
 * Perfomance metrics of the Filtered Vamana algorithm.
 */
class FilteredVamanaStatistics
{
public:
    /**
     * @brief
     * The time in seconds of the medoid's calculation.
     */
    std::uint64_t medoid_calculation_time;

    /**
     * @brief
     * The time in seconds of the filtered vamana index algorithm.
     */
    std::uint64_t filtered_vamana_indexing_time;

    /**
     * @brief Construct a new `FilteredVamanaStatistics` object.
     */
    FilteredVamanaStatistics();
};

/**
 * @brief
 * It represents the Filtered Vamana algorithm.
 */
class FilteredVamana
{
public:
    /**
     * @brief
     * It contains all the points of the dataset.
     */
    std::vector<Point> dataset;

    /**
     * @brief
     * The points that belong to each and every filter.
     */
    std::map<float, std::vector<int>> P_f;

    /**
     * @brief
     * The set of all filters.
     */
    std::set<float> F;

    /**
     * @brief
     * The medoid of the dataset.
     */
    int s;

    /**
     * @brief
     * The medoid of each filter.
     */
    std::map<float, int> st;

    /**
     * @brief
     * Construct a new `FilteredMana` object.
     * @param _dataset
     * The dataset.
     */
    FilteredVamana(const std::vector<Point> &_dataset);

    /**
     * @brief
     * Initialize some filtered vamanna structures.
     */
    void initializingEmptyGraph();

    /**
     * @brief
     * Get a random sample of a vector of size `tau`.
     * @param vec
     * The vector to take a sample from.
     * @param tau
     * The size of the sample.
     * @return std::vector<int>
     */
    std::vector<int> randomSample(std::vector<int> &P_f, int tau);

    /**
     * @brief
     * Get the points that match that filter.
     * @param f
     * The filter.
     * @return std::vector<int>
     */
    std::vector<int> getPointsWithFilter(float f);

    /**
     * @brief
     * Find the medoids per filter.
     * @param P
     * The dataset P with associated filters for all the points.
     * @param tau
     * The threshold.
     */
    std::map<float, int> findMedoids(int tau);

    /**
     * @brief
     * Implementation of the Filtered Greedy Search algorithm.
     * @param S
     * The set of the initial nodes. **Only 1 filter so we have only 1 starting node.**
     * @param x_q
     * The query point.
     * @param k
     * `k` approximate nearest neighbors.
     * @param L_
     * Search list size.
     * @param F_q
     * The query filter(s).
     * @return std::pair<std::set<int>, std::set<int>>
     * A pair of vectors: the list of the `L_size` nearest neighbors and the list of visited nodes.
     */
    std::pair<std::set<int>, std::set<int>> filteredGreedySearch(const std::set<int> &S, const Query &x_q, int k, int L_, const std::set<float> &F_q);

    /**
     * @brief
     * Filtered Robust Prune Algorithm.
     * @param p
     * The point p ∈ P
     * @param V
     * The candinate set.
     * @param a
     * The distance threshold a ≥ 1
     * @param R
     * Max outdegree bound.
     */
    void filteredRobustPrune(int p, std::set<int> &V, float a, int R);

    /**
     * @brief
     * Impleentation of the filtered vamana indexing algorithm.
     * @param a
     * @param L
     * @param R
     */
    FilteredVamanaStatistics index(int tau, float a, int L, int R);

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