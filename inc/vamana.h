#ifndef VAMANA_H
#define VAMANA_H

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <queue>
#include <limits>
#include <numeric>
#include <string>

#include <assert.h>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

int intersectionSize(const std::vector<int> &a, const std::vector<int> &b);
std::vector<int> generateSigma(int n);
float euclideanDistance(const std::vector<float> &a, const std::vector<float> &b);
std::vector<Point> parseFvecsFile(const std::string &fvecs_filepath);
std::vector<std::vector<int>> parseIvecsFile(const std::string &ivecs_filepath);

class Edge;
class Point;
class Vamana;

int intersectionSize(const std::vector<int> &a, const std::vector<int> &b)
{
    // Convert vectors to unordered sets to remove duplicates
    std::unordered_set<int> set_a(a.begin(), a.end());
    std::unordered_set<int> set_b(b.begin(), b.end());

    int count = 0;

    // Iterate through the smaller set for efficiency
    if (set_a.size() > set_b.size())
        std::swap(set_a, set_b);

    for (const int &e : set_a)
        if (set_b.find(e) != set_b.end())
            count++;

    return count;
}

/**
 * @brief
 * Parse a `.fvecs` file.
 * @param fvecs_filepath
 * The path to the `.fvecs` file.
 * @return std::vector<Point>
 */
std::vector<Point> parseFvecsFile(const std::string &fvecs_filepath)
{
    std::vector<Point> points;
    std::ifstream fvecs(fvecs_filepath, std::ios::binary);
    int idx = 0;

    while (fvecs)
    {
        int dimensions;

        // 1. The first four bytes(int) represent the number of dimensions of the vector data.
        fvecs.read(reinterpret_cast<char *>(&dimensions), sizeof(int));

        if (!fvecs)
            break;

        std::vector<float> v(dimensions);

        // 2. Read the rest of the values as a vector of size `vector_no_dimensions`.
        fvecs.read(reinterpret_cast<char *>(v.data()), dimensions * sizeof(float));

        // 3. Create a new `Point` object.
        Point new_point = Point(idx, v);

        // 4. Add it to the database.
        points.push_back(new_point);

        // 5. Incremnt index.
        idx++;
    }

    fvecs.close();

    return points;
}

/**
 * @brief
 * Parse a `.ivecs` file.
 * @param ivecs_filepath
 * The path to the `.ivecs` file.
 * @return std::vector<std::vector<int>>
 */
std::vector<std::vector<int>> parseIvecsFile(const std::string &ivecs_filepath)
{
    std::vector<std::vector<int>> ground_truth;

    std::ifstream ivecs(ivecs_filepath, std::ios::binary);
    while (ivecs)
    {
        // 1. The first four bytes(int) represent the number of dimensions of the vector data.
        int dimensions = 0;
        ivecs.read(reinterpret_cast<char *>(&dimensions), sizeof(int));

        if (!ivecs)
            break;

        std::vector<int> v(dimensions);

        // 2. Read the rest of the values as a vector of size `_dimensions`.
        ivecs.read(reinterpret_cast<char *>(v.data()), dimensions * sizeof(int));

        // 4. Add it to the database.
        ground_truth.push_back(v);
    }

    ivecs.close();

    return ground_truth;
}

/**
 * @brief
 * Generating a vector that contains all numbers from 1..n shuffled.
 * @param n
 * The size of the vector.
 * @return
 * std::vector<int>
 */
std::vector<int> generateSigma(int n)
{
    //
    std::vector<int> sigma(n);
    for (int i = 0; i < n; ++i)
        sigma[i] = i;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(sigma.begin(), sigma.end(), g);

    return sigma;
}

/**
 * @brief
 * Calculate the euclidean distance between two vectors `a` and `b`.
 * @param a
 * The first `vector`.
 * @param b
 * The second `vector`.
 * @return float
 */
float euclideanDistance(const std::vector<float> &a, const std::vector<float> &b)
{
    float sum = 0.0f;

    for (size_t i = 0; i < a.size(); ++i)
        sum += std::pow(a[i] - b[i], 2);

    return std::sqrt(sum);
}

/**
 * @brief
 * It represents a directed edge from one node to another with an associated weight.
 */
class Edge
{
public:
    /**
     * @brief
     * The index of the `Point` that the `Edge` points to.
     */
    int to_index;

    /**
     * @brief
     * The weight of the `Edge`.
     */
    float weight;

    // Default constructor
    Edge() : to_index(-1), weight(0.0f) {}

    /**
     * @brief
     * Construct a new `Edge` object.
     * @param
     * _to_index The index of the `Point` that the `Edge` points to.
     * @param
     * _weight The weight of the `Edge`.
     */
    Edge(int _to_index, float _weight)
    {
        to_index = _to_index;
        weight = _weight;
    }
};

/**
 * @brief
 * It represents a point in the dataset that contains a vector of X dimensions.
 */
class Point
{
public:
    /**
     * @brief Unique identifier of the `Point` in the dataset.
     */
    int index;

    /**
     * @brief The dimensions of the `Point`'s vector.
     */
    int dimensions;

    /**
     * @brief Query type for hybrid vector queries.
     *
     * If 0: Vector-only query.
     *
     * If 1: Query with categorical constraint ( C = v ).
     *
     * If 2: Query with timestamp range ( l <= T <= r ).
     *
     * If 3: Query with both constraints ( C = v ) and ( l <= T <= r ).
     */
    int query_type;

    /**
     * @brief Categorical attribute associated with the `Point`.
     *  The specific query value v for the categorical attribute.
     *  For queries: used as the equality predicate ( C = v ).
     *  For dataset points: represents the category of the point.
     *  If not queried,takes -1.
     */
    int category;

    /**
     * @brief Lower bound of the timestamp range (for queries).
     *  Used only when ( query_type = 2 ) or ( query_type = 3 ).
     *  If not queried,takes -1.
     */
    float lower_timestamp;

    /**
     * @brief Upper bound of the timestamp range (for queries).
     *  Used only when ( query_type = 2 ) or ( query_type = 3 ).
     *  If not queried,takes -1.
     */
    float upper_timestamp;

    /**
     * @brief The vector data of the `Point`.
     */
    std::vector<float> vec;

    /**
     * @brief The outgoing edges.
     */
    std::vector<Edge> outgoing_edges;

    /**
     * @brief Construct a new `Point` object.
     * @param idx Unique identifier of the `Point` in the dataset.
     * @param vector_data The vector data of the `Point`.
     * @param q_type Query type (0, 1, 2, 3).
     * @param cat Categorical attribute predicate (or -1 if not applicable).
     * @param ts_low Lower bound of timestamp range (or -1 if not applicable).
     * @param ts_high Upper bound of timestamp range (or -1 if not applicable).
     */
    Point(int idx, const std::vector<float> &vector_data, int q_type, int cat, float lower, float upper)
    {
        index = idx;
        dimensions = vector_data.size();
        vec = vector_data;
        query_type = q_type;
        category = cat;
        lower_timestamp = lower;
        upper_timestamp = upper;
    }

    /**
     * @brief Construct a new `Point` simplified object.
     * @param idx Unique identifier of the `Point` in the dataset.
     * @param vector_data The vector data of the `Point`.
     */
    Point(int idx, const std::vector<float> &vector_data)
    {
        index = idx;
        dimensions = vector_data.size();
        vec = vector_data;
        query_type = 0;
        category = -1;
        lower_timestamp = -1;
        upper_timestamp = -1;
    }

    /**
     * @brief Add a new neighbor/edge to this `Point`.
     * @param neighbor
     * @param weight
     */
    void addNeighbor(Point &neighbor, float weight)
    {
        // Ensure we're not adding a self-loop
        if (neighbor.index != this->index)
        {
            // Check if an edge to this neighbor already exists
            bool already_connected = false;
            for (const Edge &edge : outgoing_edges)
            {
                if (edge.to_index == neighbor.index)
                {
                    already_connected = true;
                    break;
                }
            }

            if (!already_connected)
                outgoing_edges.emplace_back(neighbor.index, weight);
        }
    }
};

/**
 * @brief
 * It respresents a set of utilities required by the Vamana indexing algorithm.
 */
class Vamana
{
private:
    // giving full access to filtered vamana class
    friend class FilteredVamana;

    /**
     * @brief
     * Generates initial random outgoing edges for each point in the dataset.
     * Each point will have exactly R outgoing edges to random other points (excluding itself).
     * @param R
     * The number of outgoing edges (neighbors) each point should have.
     */
    void generateGraphEdges(int R)
    {
        int size = dataset_size;

        // Random number generation setup
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(0, dataset_size - 1);

        // For each point in the dataset
        for (int i = 0; i < dataset_size; ++i)
        {
            Point &p = dataset[i];
            int num_of_edges = R;

            std::unordered_set<int> neighbors;

            // Keep adding random neighbors until we have R neighbors (excluding self)
            while (neighbors.size() < num_of_edges)
            {
                int j = distrib(gen);

                // Exclude the point itself
                if (j != p.index)
                {
                    neighbors.insert(j);
                }
            }

            // Add outgoing edges to the selected neighbors
            for (int neighbor_idx : neighbors)
            {
                Point &neighbor = dataset[neighbor_idx];
                p.addNeighbor(neighbor, euclideanDistance(p.vec, neighbor.vec));
            }
        }
    }

    /**
     * @brief
     * Get a visual representation of the `Point`'s neighbors.
     * Useful for debugging.
     * @param p_idx
     * The index of the `Point`.
     * @return std::string
     */
    std::string neighborsTable(int p_idx)
    {
        std::ostringstream oss;
        oss << "[ ";

        // Loop through each outgoing edge for the specified point
        for (size_t i = 0; i < dataset[p_idx].outgoing_edges.size(); ++i)
        {
            const Edge &edge = dataset[p_idx].outgoing_edges[i];
            oss << "(" << edge.to_index << ", " << edge.weight << ")";

            // Add a comma separator if it's not the last element
            if (i < dataset[p_idx].outgoing_edges.size() - 1)
            {
                oss << ", ";
            }
        }

        oss << " ]";
        return oss.str();
    }

    /**
     * @brief
     * Prints the vector data in a table format.
     * @param vec
     * The vector data.
     * @return std::string
     */
    std::string vectorTable(std::vector<int> vec)
    {
        std::ostringstream oss;
        oss << "[ ";
        for (size_t i = 0; i < vec.size(); ++i)
        {
            oss << vec[i];
            if (i < vec.size() - 1)
            {
                oss << ", ";
            }
        }
        oss << " ]";

        return oss.str();
    }

public:
    /**
     * @brief
     * It contains all the `Point`s of the `dataset`.
     */
    std::vector<Point> dataset;

    /**
     * @brief
     * It contains all the `Point`s of the `queryset`.
     */
    std::vector<Point> queryset;

    /**
     * @brief
     * The number of `Point` in the `dataset`.
     */
    int dataset_size, queryset_size;

    /**
     * @brief
     * The index of the medoid `Point` of the `dataset`.
     */
    int medoid_idx;

    /**
     * @brief
     * Construct a new `Vamana` object.
     * @param _dataset
     * The dataset.
     */
    Vamana(std::vector<Point> &_dataset)
    {
        dataset = _dataset;
        dataset_size = _dataset.size();
    }

    /**
     * @brief
     * Construct a new `Vamana` object.
     * @param _dataset
     * The dataset.
     * @param _queryset
     * The queries set.
     */
    Vamana(std::vector<Point> &_dataset, std::vector<Point> &_queryset)
    {
        dataset = _dataset;
        dataset_size = _dataset.size();
        queryset = _queryset;
        dataset_size = _queryset.size();
    }

    /**
     * @brief
     * Calculate the medoid of the dataset.
     * The medoid of the dataset is similar in concept to mean or centroid, but the medoid is always restricted to be a member of the data set.
     * This function calculates the total distance from each point to all other points and then select the point with the smallest total distance as the medoid.
     * @param dataset
     * The dataset that you want to calculate the medoid for.
     * @return int
     * The index to the Medoid.
     */
    void calculateMedoid()
    {
        float minimum_dist = std::numeric_limits<float>::max();
        int _medoid_idx = -1;

        // Iterating through the points to calculate the node with the smallest total distance from the all rest points.
        for (int i = 0; i < dataset.size(); i++)
        {
            float total_distance = 0.0f;

            for (int j = 0; j < dataset.size(); j++)
            {
                if (i != j)
                {
                    float distance = euclideanDistance(dataset[i].vec, dataset[j].vec);
                    total_distance += distance;
                }
            }

            if (total_distance < minimum_dist)
            {
                minimum_dist = total_distance;
                _medoid_idx = i;
            }
        }

        medoid_idx = _medoid_idx;
    }

    /**
     * @brief
     * It prunes the list of candidate neighbors for a given `Point` based on a
     * pruning condition that involves an `alpha` scaling factor.
     * It ensures that the `Point` has at most R outgoing edges to its nearest neighbors.
     * @param p_idx
     * The node for which we are pruning the candidate neighbors.
     * @param V
     * The set of candidate nodes.
     * @param alpha
     * The distance threshold.
     * @param R
     * The maximum number of outgoing edges.
     */
    void robustPrune(int p_idx, std::vector<int> &V, float a, int R)
    {
        // V ← (V ∪ Nout(p)) \ {p}
        std::vector<Edge> &p_edges = dataset[p_idx].outgoing_edges;
        for (Edge &e : p_edges)
            if ((std::find(V.begin(), V.end(), e.to_index)) == V.end())
                V.push_back(e.to_index);

        V.erase(std::remove(V.begin(), V.end(), p_idx), V.end());

        // Nout(p) ← {}
        p_edges.clear();

        // while V != {} do
        while (!V.empty())
        {
            // p∗ ← min(d(p, v), v ∈ V)
            int pstar_idx = -1;
            float pstar_dist = std::numeric_limits<float>::max();
            for (int v_idx : V)
            {
                float distance = euclideanDistance(dataset[v_idx].vec, dataset[p_idx].vec);
                if (distance < pstar_dist)
                {
                    pstar_dist = distance;
                    pstar_idx = v_idx;
                }
            }

            // Nout(p) ← Nout(p) ∪ {p∗} (check that p* not in Nout(p))
            if (std::find_if(
                    p_edges.begin(),
                    p_edges.end(),
                    [pstar_idx](const Edge &edge)
                    { return edge.to_index == pstar_idx; }) == p_edges.end())
                p_edges.push_back(Edge(pstar_idx, pstar_dist));

            // if |Nout(p)| = R then break
            if (p_edges.size() == R)
                break;

            // for v ∈ V do.
            std::vector<int> to_remove;
            for (int v_idx : V)
            {
                float pstar_to_v = euclideanDistance(dataset[pstar_idx].vec, dataset[v_idx].vec);
                float p_to_v = euclideanDistance(dataset[p_idx].vec, dataset[v_idx].vec);

                // if α · d(p∗, v) ≤ d(p, v) then remove v from V
                if (a * pstar_to_v <= p_to_v)
                    to_remove.push_back(v_idx);
            }

            // remove v from V
            for (int remove_idx : to_remove)
                V.erase(std::remove(V.begin(), V.end(), remove_idx), V.end());
        }
    }

    /**
     * @brief
     * Greedy search algorithm.
     * @param source_idx
     * Index of the source node.
     * @param query_idx
     * Index of the query node.
     * @param k
     * Number of nearest neighbors to find.
     * @param L_size
     * Maximum size of the candidate list L.
     * @return std::pair<std::vector<int>, std::vector<int>>
     * A pair of vectors: the list of the `L_size` nearest neighbors and the list of visited nodes.
     */
    std::pair<std::vector<int>, std::vector<int>> greedySearch(int source_idx, int query_idx, int k, int L_size)
    {
        std::vector<int> L;         // Candidate points
        std::vector<int> V;         // Visited points
        std::vector<int> L_minus_V; // L \ V

        // L ← {s}
        L.push_back(source_idx);

        // While L\V != {}
        while (true)
        {
            // L\V
            L_minus_V.clear();
            for (int l_idx : L)
                if (l_idx != query_idx && std::find(V.begin(), V.end(), l_idx) == V.end())
                    L_minus_V.push_back(l_idx);

            // L\V ← {}, break
            if (L_minus_V.empty())
                break;

            // p* ← min(||xp −xq||, p ∈ L\V)
            int pstar_idx = -1;
            float min_dist = std::numeric_limits<float>::max();
            for (int u_idx : L_minus_V)
            {
                float d = euclideanDistance(dataset[u_idx].vec, dataset[query_idx].vec);
                if (d < min_dist)
                {
                    min_dist = d;
                    pstar_idx = u_idx;
                }
            }

            // update L ← L ∪ Nout(p∗)
            for (Edge &edge : dataset[pstar_idx].outgoing_edges)
                if (edge.to_index != query_idx && std::find(L.begin(), L.end(), edge.to_index) == L.end())
                    L.push_back(edge.to_index);

            // update V ← V ∪ {p∗}
            if (std::find(V.begin(), V.end(), pstar_idx) == V.end())
                V.push_back(pstar_idx);

            // if |L| > L then update L to retain closest L points to q
            if (L.size() > L_size)
            {
                // Sort L based on distance to the query node
                std::sort(
                    L.begin(),
                    L.end(),
                    [this, query_idx](int a_idx, int b_idx)
                    {
                        return euclideanDistance(dataset[a_idx].vec, dataset[query_idx].vec) < euclideanDistance(dataset[b_idx].vec, dataset[query_idx].vec);
                    });

                // Resize L to L_size
                L.resize(L_size);
            }
        }

        // return [closest k points from L; V]
        if (L.size() > k)
            L.resize(k);

        return std::make_pair(L, V);
    }

    /**
     * @brief
     * Get the k nearest neighbors using the Greedy Search algorithm.
     * @param source_idx
     * Index of the source node.
     * @param query_point
     * The query `Point`.
     * @param k
     * Number of nearest neighbors to find.
     * @param L_size
     * Maximum size of the candidate list L.
     * @return A pair of vectors: the list of found nodes and the list of visited nodes.
     */
    std::vector<int> greedySearchNearestNeighbors(int source_idx, Point &query_point, int k, int L_size)
    {
        std::vector<int> L;
        std::vector<int> V;
        std::vector<int> L_minus_V; // L \ V

        // L ← {s}
        L.push_back(source_idx);

        // While L\V != {}
        while (true)
        {
            // L\V
            L_minus_V.clear();
            for (int l_idx : L)
                if (std::find(V.begin(), V.end(), l_idx) == V.end())
                    L_minus_V.push_back(l_idx);

            // L\V ← {}, break
            if (L_minus_V.empty())
                break;

            // p* ← min(||xp −xq||, p ∈ L\V)
            int pstar_idx = -1;
            float min_dist = std::numeric_limits<float>::max();
            for (int u_idx : L_minus_V)
            {
                float d = euclideanDistance(dataset[u_idx].vec, query_point.vec);
                if (d < min_dist)
                {
                    min_dist = d;
                    pstar_idx = u_idx;
                }
            }

            // update L ← L ∪ Nout(p∗)
            for (Edge &edge : dataset[pstar_idx].outgoing_edges)
                if (std::find(L.begin(), L.end(), edge.to_index) == L.end())
                    L.push_back(edge.to_index);

            // update V ← V ∪ {p∗}
            if (std::find(V.begin(), V.end(), pstar_idx) == V.end())
                V.push_back(pstar_idx);

            // if |L| > L then update L to retain closest L points to q
            if (L.size() > L_size)
            {
                // Sort L based on distance to the query node
                std::sort(
                    L.begin(),
                    L.end(),
                    [this, query_point](int a_idx, int b_idx)
                    {
                        return euclideanDistance(dataset[a_idx].vec, query_point.vec) < euclideanDistance(dataset[b_idx].vec, query_point.vec);
                    });

                // Resize L to L_size
                L.resize(L_size);
            }
        }

        // return [closest k points from L; V]
        if (L.size() > k)
            L.resize(k);

        return L;
    }
    /**
     * @brief
     * Brute force calculation of the k nearest neighbors of a given query point.
     * @param query_point
     * The query `Point` (can be a new point or one from the dataset).
     * @param k
     * The number of nearest neighbors to find.
     * @return
     * A vector of indices of the k nearest neighbors in the dataset.
     */
    std::vector<int> bruteForceNearestNeighbors(const Point &query_point, int k)
    {
        // Vector to store pairs of distance and index
        std::vector<std::pair<float, int>> distances;

        // Compute distance from the query point to every point in the dataset
        for (const Point &p : dataset)
        {
            float dist = euclideanDistance(query_point.vec, p.vec);
            distances.push_back(std::make_pair(dist, p.index));
        }

        // Sort the distances in ascending order
        std::sort(
            distances.begin(),
            distances.end(),
            [](const std::pair<float, int> &a, const std::pair<float, int> &b)
            { return a.first < b.first; });

        // Extract the indices of the k nearest neighbors
        std::vector<int> neighbors;
        for (int i = 0; i < k && i < distances.size(); ++i)
        {
            neighbors.push_back(distances[i].second);
        }

        return neighbors;
    }

    /**
     * @brief
     * Creates an in-memory index on the supplied nodes to efficiently answer approximate nearest neighbor queries.
     * The N nodes must already be initialized as a random graph of outgoing edges with maximum of log(N) outgoing edges per node.
     * @param a
     * Scaling factor to prune outgoing eges of a node (alpha).
     * @param L
     * Maximum list of search candidates to use in graph traversal.
     * @param R
     * Maximum number of outgoing edges of a node. Must be less than log(N) for good results.
     */
    void index(float a, int L, int R)
    {
        spdlog::debug("+---------------------------+");
        spdlog::debug("| Vamana Indexing Algorithm |");
        spdlog::debug("+---------------------------+");
        spdlog::debug("# a ← {} & L_size ← {} & R ← {} & s ← {}", a, L, R, medoid_idx);

        // initialize G to a random R-regular directed graph
        generateGraphEdges(R);

        // let s denote the medoid of dataset P
        int s = medoid_idx;

        // let sigma denote a random permutation of 1..n
        std::vector<int> sigma = generateSigma(dataset_size);

        // for 1 ≤ i ≤ n do
        for (int i = 0; i < dataset_size; ++i)
        {

            // let [L; V] ← GreedySearch(s, σ(i), 1, L_size)
            std::pair<std::vector<int>, std::vector<int>> r = greedySearch(s, sigma[i], 1, L);

            // run RobustPrune(σ(i), V, a, R) to update out-neighbors of σ(i)
            robustPrune(sigma[i], r.second, a, R);

            spdlog::debug("=====================================================================");
            spdlog::debug("+ GreedySearch(s, σ({}), 1, {})", i, L);
            spdlog::debug("# V ← {}", vectorTable(r.second));
            spdlog::debug("+ RobustPrune(σ({}), V, a, R)", i);
            spdlog::debug("# Neighbors(σ({}))) ← {}", i, neighborsTable(sigma[i]));

            // for all points j in Neighbors(σ(i))
            for (Edge &e : dataset[sigma[i]].outgoing_edges)
            {
                int j_idx = e.to_index;

                // W ← Nout(j) ∪ {σ(i)}
                std::vector<int> W(dataset[j_idx].outgoing_edges.size());

                std::transform(
                    dataset[j_idx].outgoing_edges.begin(),
                    dataset[j_idx].outgoing_edges.end(),
                    W.begin(),
                    [](const Edge &edge)
                    { return edge.to_index; });

                W.push_back(sigma[i]);

                spdlog::debug("---------------------------------------------------------------------");
                spdlog::debug("# j ← {}", j_idx);
                spdlog::debug("# Neighbors(j) = {}", neighborsTable(j_idx));

                // |W| > R => run RobustPrune(j, W, a, R) to update out-neighbors of j
                if (W.size() > R)
                {
                    robustPrune(j_idx, W, a, R);

                    spdlog::debug("+ |W| > R => RobustPrune(j, W, {}, {})", j_idx, a, R);
                    spdlog::debug("# Neighbors(j) = {}", neighborsTable(j_idx));
                }
                // |W| < R | => update Neighbors(j) ← Neighbors(j) U {sigma(i)}
                else
                {
                    dataset[j_idx].outgoing_edges.push_back(Edge(sigma[i], euclideanDistance(dataset[sigma[i]].vec, dataset[j_idx].vec)));

                    spdlog::debug("+ |W| < R => Neighbors(j) ← Neighbors(j) U {}", R, j_idx, j_idx, sigma[i]);
                    spdlog::debug("# Neighbors(j) = {}", neighborsTable(j_idx));
                }

                spdlog::debug("---------------------------------------------------------------------");
            }
        }

        spdlog::debug("+---------------------------------------------------------------------------------+");
    }
};

#endif // VAMANA_H