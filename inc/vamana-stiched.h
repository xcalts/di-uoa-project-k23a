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
#include "vamana.h"

/**
 * @brief
 * Perfomance metrics of the Stiched Vamana algorithm.
 */
class StichedVamanaStatistics
{
public:
    /**
     * @brief
     * The time in seconds of the medoid's calculation.
     */
    std::uint64_t medoid_calculation_time;

    /**
     * @brief
     * The time in seconds of the stiched vamana index algorithm.
     */
    std::uint64_t stiched_vamana_indexing_time;

    /**
     * @brief Construct a new `StichedVamanaStatistics` object.
     */
    StichedVamanaStatistics();
};

/**
 * @brief
 * It represents the Stiched Vamana algorithm.
 */
class StichedVamana
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
    std::map<float, std::vector<Point>> P_f;

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
     * A Vamana Index graph for each and every f in F.
     */
    std::map<float, Vamana> G_f;

    /**
     * @brief
     * Construct a new `StichedVamana` object.
     * @param _dataset
     * The dataset.
     */
    StichedVamana(const std::vector<Point> &_dataset);

    /**
     * @brief
     * Initialize some stiched vamanna structures.
     */
    void initializingEmptyGraph();

    /**
     * @brief
     * Impleentation of the stiched vamana indexing algorithm.
     * @param a
     * @param L
     * @param R
     */
    StichedVamanaStatistics index(float a, int L_small, int R_small, int R_stiched);

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