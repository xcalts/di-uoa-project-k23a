#ifndef DATA_H
#define DATA_H

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <queue>
#include <limits>

#include "log.h"

/**
 * @brief
 * It represents a directed edge from one node to another with an associated weight.
 */
class Edge
{
public:
    /**
     * @brief The index of the `Point` that the `Edge` points to.
     */
    int to_index;

    /**
     * @brief The weight of the `Edge`.
     */
    int weight;

    /**
     * @brief Base constructor.
     */
    Edge() {}

    /**
     * @brief Construct a new `Edge` object.
     * @param _to_index The index of the `Point` that the `Edge` points to.
     * @param _weight The weight of the `Edge`.
     */
    Edge(int _to_index, int _weight)
    {
        to_index = _to_index;
        weight = _weight;
    }
};

/**
 * @brief
 * It represents a point in the form of a vector of X dimensions.
 */
class Point
{
public:
    /**
     * @brief Unique identifier of the `Point` in the dataset.
     */
    int index;

    /**
     * @brief The dimensions of the `Point` vector.
     */
    int dimensions;

    /**
     * @brief The vector data of the `Point`.
     */
    std::vector<float> vec;

    /**
     * @brief The outgoing points.
     */
    std::vector<Point> outgoing_edges;

    /**
     * @brief Base constructor.
     */
    Point() {}

    /**
     * @brief Construct a new `Point` object.
     * @param idx Unique identifier of the `Point` in the dataset.
     * @param vector_data The vector data of the `Point`.
     */
    Point(int idx, const std::vector<float> &vector_data)
    {
        index = idx;
        dimensions = vector_data.size();
        vec = vector_data;
    }

    /**
     * @brief Prints the data vector in a table format.
     */
    void printTable() const
    {
        std::cout << "[ ";
        for (size_t i = 0; i < vec.size(); ++i)
        {
            std::cout << vec[i];
            if (i < vec.size() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << " ]" << std::endl;
    }

    /**
     * @brief Prints the data vector in a histogram format.
     */
    void printHistogram() const
    {
        const size_t height = 20; // Height of histogram in rows
        float max_value = *std::max_element(vec.begin(), vec.end());

        // Loop over each row from top to bottom
        for (size_t row = height; row > 0; --row)
        {
            float threshold = (static_cast<float>(row) / height) * max_value;

            for (size_t i = 0; i < vec.size(); ++i)
            {
                if (vec[i] >= threshold)
                {
                    std::cout << "#";
                }
                else
                {
                    std::cout << " ";
                }
            }
            std::cout << std::endl;
        }

        // Print the x-axis with index labels
        for (size_t i = 0; i < vec.size(); ++i)
        {
            std::cout << "-";
        }
        std::cout << std::endl;
    }

    /**
     * @brief Generates a histogram of the data vector as a formatted string.
     * @return std::string
     */
    std::string getHistogram() const
    {
        const size_t height = 20; // Height of histogram in rows
        float max_value = *std::max_element(vec.begin(), vec.end());
        std::ostringstream oss;

        // Loop over each row from top to bottom
        for (size_t row = height; row > 0; --row)
        {
            float threshold = (static_cast<float>(row) / height) * max_value;

            for (size_t i = 0; i < vec.size(); ++i)
            {
                if (vec[i] >= threshold)
                {
                    oss << ".";
                }
                else
                {
                    oss << " ";
                }
            }
            oss << "\n";
        }

        // Append the x-axis with index labels
        for (size_t i = 0; i < vec.size(); ++i)
        {
            oss << "-";
        }
        oss << "\n";

        // Append index labels below the histogram
        for (size_t i = 0; i < vec.size(); ++i)
        {
            oss << i % 10; // Single-digit labels for compactness
        }
        oss << "\n";

        return oss.str();
    }
};

/**
 * @brief Calculate the euclidean distance between two vectors `a` and `b`.
 * @param a The first `vector`.
 * @param b The second `vector`.
 * @return float
 */
float euclideanDistance(const std::vector<float> &a, const std::vector<float> &b)
{
    float sum = 0.0f;

    for (size_t i = 0; i < a.size(); ++i)
    {
        float diff = a[i] - b[i];
        sum += diff * diff;
    }

    return std::sqrt(sum);
}

/**
 * @brief Medoids are similar in concept to means or centroids, but medoids are always restricted to be members of the data set.
 * @param dataset The dataset that you want to find the mediod for.
 * @return Point&
 */
Point &calculateMedoid(std::vector<Point> &dataset)
{
    int mediod_index;
    size_t total_points = dataset.size();
    float minimum = std::numeric_limits<float>::max();

    for (int i = 0; i < total_points; i++)
        for (int j = 0; j < total_points; j++)
            if (i != j)
            {
                Point i_point = dataset[i];
                Point j_point = dataset[j];

                float distance = euclideanDistance(i_point.vec, j_point.vec);

                if (distance < minimum)
                {
                    minimum = distance;
                    mediod_index = i;
                }
            }

    return dataset[mediod_index];
}

/**
 * @brief The results of the Greedy Search algorithm.
 */
struct GreedySearchResults
{
    /**
     * @brief The k approximate nearest neighbors.
     */
    std::vector<Point> knn_points;
    /**
     * @brief The visited points during the search.
     */
    std::vector<Point> visited_points;
};

GreedySearchResults &greedySearch(Point source, Point query, int k, int max_candinates)
{
    GreedySearchResults results;

    std::vector<Point> candinates;
    std::vector<Point> visited;

    candinates.push_back(source);

    return results;
}

/**
 * @brief Creates an in-memory index on the supplied points to efficiently answer approximate nearest neighbor queries.
 * The N points must already be initialized as a random graph of outgoing edges with maximum of log(N) outgoing edges per point.
 * @param dataset List of N points.
 * @param alpha Scaling factor to prune outgoing eges of a node.
 * @param max_candinates Maximum list of search candidates to use in graph traversal.
 * @param max_neighbors Maximum number of outgoing edges of a node. Must be less than log(N) for good results.
 */
void vamanaIndex(std::vector<Point> dataset, int alpha, int max_candinates, int max_neighbors)
{
    // Calculating the Mediod of the dataset.
    // Point mediod = calculateMediod(dataset);
    Point mediod = dataset.at(5762);
    print_verbose("(data.h) (vamanaIndex) Mediod's Index: " + std::to_string(mediod.index) + ".");

    // Generating a vector of randomly shuffled nodes for random insertion in the graph.
    std::vector<int> sigma(dataset.size());
    std::iota(sigma.begin(), sigma.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd()); // Mersenne Twister engine seeded with rd()
    std::shuffle(sigma.begin(), sigma.end(), g);

    // Iterating through each and every dataset point.
    for (int i = 0; i < dataset.size(); i++)
    {
    }
}

#endif // DATA_H