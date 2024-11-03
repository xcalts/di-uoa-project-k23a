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
 * @brief It represents a directed edge from one node to another with an associated weight.
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
 * @brief It represents a point in the form of a vector of X dimensions.
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
     * @brief The outgoing edges.
     */
    std::vector<Edge> outgoing_edges;

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

/**
 * @brief Graph.
 */
class Graph
{
public:
    /**
     * @brief It contains all the points that make up the graph.
     */
    std::vector<Point> dataset;

    /**
     * @brief Base Constructor
     */
    Graph() {}

    /**
     * @brief Construct a new `Graph` object.
     * @param _dataset The dataset of `Point`s that are inside the `Graph`.
     */
    Graph(std::vector<Point> _dataset)
    {
        dataset = _dataset;
    }

    /**
     * @brief It prunes the list of candidate neighbors for a given source_node based on a pruning condition that involves an alpha scaling factor.
     * It ensures that the source_node has at most R outgoing edges to its nearest neighbors.
     * @param source_node The node for which we are pruning the candidate neighbors
     * @param candidates A set of candidate node indices.
     * @param alpha A scaling factor used in the pruning condition.
     * @param R The maximum number of outgoing edges (neighbors) the source_node should have.
     */
    void robustPrune(Point &source_node, std::unordered_set<int> &candidates, float alpha, int R)
    {
        // Add neighbors of source_node to candidates
        for (const Edge &edge : source_node.outgoing_edges)
        {
            candidates.insert(edge.to_index);
        }

        // Remove source_node from candidates if present
        candidates.erase(source_node.index);

        // Clear the current neighbors of source_node
        source_node.outgoing_edges.clear();

        // Define a lambda function to compute distance to source_node
        auto distance_to_source_node = [this, &source_node](int p_index) -> float
        {
            return euclideanDistance(dataset[p_index].vec, source_node.vec);
        };

        while (!candidates.empty())
        {
            // Find p_star: the candidate closest to source_node
            int p_star_index = -1;
            float min_distance = std::numeric_limits<float>::max();

            for (int c : candidates)
            {
                float distance = distance_to_source_node(c);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    p_star_index = c;
                }
            }

            // Safeguard in case p_star_index wasn't set
            if (p_star_index == -1)
                break;

            // Get current neighbors of source_node
            std::unordered_set<int> new_neighbors_indices;
            for (const Edge &edge : source_node.outgoing_edges)
            {
                new_neighbors_indices.insert(edge.to_index);
            }

            // Add p_star to new_neighbors
            new_neighbors_indices.insert(p_star_index);

            // Update the neighbors of source_node
            source_node.outgoing_edges.clear();
            for (int idx : new_neighbors_indices)
            {
                // Assuming the weight is the Euclidean distance
                float weight = euclideanDistance(source_node.vec, dataset[idx].vec);
                source_node.outgoing_edges.push_back(Edge(idx, weight));
            }

            // If the desired number of neighbors is reached, exit the loop
            if (new_neighbors_indices.size() == static_cast<size_t>(R))
                break;

            // Remove p_star from candidates
            candidates.erase(p_star_index);

            // Prepare to remove nodes from candidates based on the pruning condition
            std::vector<int> to_remove;
            for (int other_index : candidates)
            {
                float distance_p_star_other = euclideanDistance(dataset[p_star_index].vec, dataset[other_index].vec);
                float distance_source_other = euclideanDistance(source_node.vec, dataset[other_index].vec);

                // Prune candidates using the alpha scaling factor
                if (alpha * distance_p_star_other <= distance_source_other)
                {
                    to_remove.push_back(other_index);
                }
            }

            // Remove the pruned nodes from candidates
            for (int idx : to_remove)
            {
                candidates.erase(idx);
            }
        }
    }

    /**
     * @brief
     *
     * @param source_point
     * @param query_point
     * @param k
     * @param max_candinates
     * @return GreedySearchResults
     */
    GreedySearchResults greedySearch(Point &source_point, Point &query_point, int k, int max_candidates)
    {
        GreedySearchResults results;
        std::unordered_set<int> candidates; // Set of candidate indices
        std::unordered_set<int> visited;    // Set of visited indices

        // Initialize candidates with the source point's index
        candidates.insert(source_point.index);

        while (true)
        {
            // Find unvisited candidates (candidates - visited)
            std::unordered_set<int> unvisited_candidates;
            for (int c : candidates)
            {
                if (visited.find(c) == visited.end())
                    unvisited_candidates.insert(c);
            }

            // Exit if there are no unvisited candidates
            if (unvisited_candidates.empty())
                break;

            // Find the candidate with the minimum distance to the query point
            int p_star_index = -1;
            float min_distance = std::numeric_limits<float>::max();

            for (int c : unvisited_candidates)
            {
                float distance = euclideanDistance(dataset[c].vec, query_point.vec);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    p_star_index = c;
                }
            }

            // Safeguard in case p_star_index wasn't set
            if (p_star_index == -1)
                break;

            // Add neighbors of p_star to candidates
            for (Edge &edge : dataset[p_star_index].outgoing_edges)
            {
                candidates.insert(edge.to_index);
            }

            // Mark p_star as visited
            visited.insert(p_star_index);

            // Limit the number of candidates to max_candidates (L)
            if (candidates.size() > static_cast<size_t>(max_candidates))
            {
                // Convert candidates to a vector for sorting
                std::vector<int> candidate_vector(candidates.begin(), candidates.end());

                // Partially sort to find the max_candidates closest to the query point
                std::nth_element(
                    candidate_vector.begin(),
                    candidate_vector.begin() + max_candidates,
                    candidate_vector.end(),
                    [this, &query_point](int a, int b)
                    {
                        return euclideanDistance(dataset[a].vec, query_point.vec) < euclideanDistance(dataset[b].vec, query_point.vec);
                    });

                // Keep only the first max_candidates elements
                candidates.clear();
                candidates.insert(candidate_vector.begin(), candidate_vector.begin() + max_candidates);
            }
        }

        // Find the k nearest neighbors among candidates
        std::vector<int> candidate_vector(candidates.begin(), candidates.end());

        if (candidate_vector.size() > static_cast<size_t>(k))
        {
            // Partially sort to get the k closest candidates
            std::nth_element(
                candidate_vector.begin(),
                candidate_vector.begin() + k,
                candidate_vector.end(),
                [this, &query_point](int a, int b)
                {
                    return euclideanDistance(dataset[a].vec, query_point.vec) < euclideanDistance(dataset[b].vec, query_point.vec);
                });

            candidate_vector.resize(k);
        }

        // Fill the knn_points with the k nearest neighbors
        for (int idx : candidate_vector)
        {
            results.knn_points.push_back(dataset[idx]);
        }

        // Fill the visited_points
        for (int idx : visited)
        {
            results.visited_points.push_back(dataset[idx]);
        }

        return results;
    }

    /**
     * @brief Creates an in-memory index on the supplied points to efficiently answer approximate nearest neighbor queries.
     * The N points must already be initialized as a random graph of outgoing edges with maximum of log(N) outgoing edges per point.
     * @param alpha Scaling factor to prune outgoing eges of a node.
     * @param max_candinates Maximum list of search candidates to use in graph traversal.
     * @param max_neighbors Maximum number of outgoing edges of a node. Must be less than log(N) for good results.
     */
    void vamanaIndex(float alpha, int max_candinates, int max_neighbors)
    {
        // Calculating the Mediod of the dataset.
        // Point mediod = calculateMedoid(dataset);
        Point medoid = dataset.at(5762);
        print_verbose("(data.h) (vamanaIndex) Medoid's Index: " + std::to_string(medoid.index) + ".");

        // Generating a vector of randomly shuffled nodes for random insertion in the graph.
        std::vector<int> sigma(dataset.size());
        std::iota(sigma.begin(), sigma.end(), 0);
        std::random_device rd;
        std::mt19937 g(rd()); // Mersenne Twister engine seeded with rd()
        std::shuffle(sigma.begin(), sigma.end(), g);

        // Iterating through each and every dataset point.
        for (int i = 0; i < dataset.size(); ++i)
        {
            int sigma_idx = sigma[i];
            Point &current_point = dataset[sigma_idx];

            // Perform greedy search from medoid to current_point
            GreedySearchResults search_results = greedySearch(medoid, current_point, 1, max_candinates);

            // Convert visited_points to a set of indices
            std::unordered_set<int> visited_indices;
            for (const Point &p : search_results.visited_points)
            {
                visited_indices.insert(p.index);
            }

            // Run robustPrune on current_point with visited indices
            robustPrune(current_point, visited_indices, alpha, max_neighbors);

            // For each neighbor of current_point
            for (const Edge &edge : current_point.outgoing_edges)
            {
                int neighbor_idx = edge.to_index;
                Point &neighbor = dataset[neighbor_idx];

                // Get outgoing neighbors of neighbor
                std::unordered_set<int> outgoing_indices;
                for (const Edge &e : neighbor.outgoing_edges)
                {
                    outgoing_indices.insert(e.to_index);
                }

                // Add current_point to outgoing neighbors
                outgoing_indices.insert(current_point.index);

                if (outgoing_indices.size() > static_cast<size_t>(max_neighbors))
                {
                    // Run robustPrune on neighbor with outgoing indices
                    robustPrune(neighbor, outgoing_indices, alpha, max_neighbors);
                }
                else
                {
                    // Set neighbors of neighbor to outgoing_indices
                    neighbor.outgoing_edges.clear();
                    for (int idx : outgoing_indices)
                    {
                        float weight = euclideanDistance(neighbor.vec, dataset[idx].vec);
                        neighbor.outgoing_edges.push_back(Edge(idx, weight));
                    }
                }
            }
        }
    }
};
#endif // DATA_H