#ifndef VAMANA_FILTERED_H
#define VAMANA_FILTERED_H

/**********************/
/* Standard Libraries */
/**********************/

#include <vector>
#include <set>

/**********************/
/* External Libraries */
/**********************/

#include "progressbar.h"

/**********************/
/* Project Components */
/**********************/

#include "misc.h"

/***************/
/* Definitions */
/***************/

class F_Edge;
class F_Point;
class F_Query;
class F_Vamana;
// class S_Vamana;

std::vector<F_Point> parseDummyData(std::string &, int);
std::vector<F_Query> parseDummyQueries(std::string &, int);

/*******************/
/* Implementations */
/*******************/

/**
 * @brief
 * It represents a directed edge from one point to another with an associated weight.
 */
class F_Edge
{
public:
    /**
     * @brief
     * The index of the point that the edge points to.
     */
    int to_index;

    /**
     * @brief
     * The weight of the `Edge`.
     */
    float weight;

    /**
     * @brief
     * Construct a new edge.
     * @param _to_index
     * The index of the point that the edge points to.
     * @param _weight
     * The weight of the edge.
     */
    F_Edge(int _to_index, float _weight)
    {
        to_index = _to_index;
        weight = _weight;
    }
};

/**
 * @brief
 * It represents a point in the dataset.
 */
class F_Point
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
     * The edges to neighbor points.
     */
    std::vector<F_Edge> edges;

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
    F_Point(int _index, std::vector<float> &_vec, float _C, float _T)
    {
        index = _index;
        dimensions = _vec.size();
        vec = _vec;
        C = _C;
        T = _T;
    }

    /**
     * @brief
     * Add a new neighbor to the point.
     * @param neighbor
     * The neighbor to add.
     * @param weight
     * The weight of the edge.
     */
    void addNeighborPoint(F_Point &neighbor, float weight)
    {
        // Ensure we're not adding a self-loop
        if (neighbor.index != this->index)
        {
            // Check if an edge to this neighbor already exists
            bool already_connected = false;
            for (F_Edge edge : edges)
            {
                if (edge.to_index == neighbor.index)
                {
                    already_connected = true;
                    break;
                }
            }

            if (!already_connected)
                edges.emplace_back(neighbor.index, weight);
        }
    }

    /**
     * @brief
     * Get the indices of the point's neighbors.
     * @return std::vector<int>
     */
    std::vector<int> getNeighborIndices() const
    {
        std::vector<int> neighbor_indices;
        neighbor_indices.reserve(edges.size()); // Reserve space to avoid unnecessary reallocations

        for (F_Edge edge : edges)
            neighbor_indices.push_back(edge.to_index);

        return neighbor_indices;
    }

    /**
     * @brief
     * Get the indices of the point's neighbors as a set.
     * @return std::set<int>
     */
    std::set<int> getNeighborIndicesSet() const
    {
        std::set<int> neighbor_indices;
        for (F_Edge edge : edges)
            neighbor_indices.insert(edge.to_index);
        return neighbor_indices;
    }
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
class F_Query
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
     * The outgoing edges.
     */
    std::vector<F_Edge> edges;

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
    F_Query(
        int _index,
        std::vector<float> &_vec,
        float _query_type,
        float _v,
        float _l,
        float _r)
    {
        index = _index;
        dimensions = _vec.size();
        vec = _vec;
        query_type = _query_type;
        v = _v;
        l = _l;
        r = _r;
    }
};

/**
 * @brief
 * It represents the Vamana algorithm with filters.
 */
class F_Vamana
{
private:
    /**
     * @brief
     * Get a random sample of a vector of size `tau`.
     * @param vec
     * The vector to take a sample from.
     * @param tau
     * The size of the sample.
     * @return std::vector<int>
     */
    std::vector<int> randomSample(std::vector<int> &vec, int tau)
    {
        std::vector<int> R_f;
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(vec.begin(), vec.end(), g);

        for (int i = 0; i < tau; i++)
            R_f.push_back(vec[i]);

        return R_f;
    }

    /**
     * @brief
     * Select the point with the smallest load from the vector rf according to the map T
     * @param load
     * A map with the index of points and their load.
     * @param vec
     * A vector with the indexes of points.
     * @return int
     */
    int selectLeastLoaded(std::map<int, int> &load, std::vector<int> &vec)
    {
        // If vec is empty, decide on a fallback (e.g., return -1)
        if (vec.empty())
            return -1;

        // Initialize with the first element in vec
        int selected_point = vec[0];
        int min_load = INT_MAX;

        // Try to find the load for the first element
        auto it = load.find(selected_point);
        if (it != load.end())
            min_load = it->second;

        // Iterate through the rest of the elements
        for (size_t i = 1; i < vec.size(); ++i)
        {
            int point = vec[i];
            auto load_it = load.find(point);
            if (load_it != load.end() && load_it->second < min_load)
            {
                min_load = load_it->second;
                selected_point = point;
            }
        }

        return selected_point;
    }

public:
    /**
     * @brief
     * It contains all the points of the dataset.
     */
    std::vector<F_Point> dataset;

    /**
     * @brief
     * The index of the medoid in the dataset.
     */
    int medoid_idx;

    /**
     * @brief
     * The medoid indices per filter.
     */
    std::map<int, int> medoid_indices;

    /**
     * @brief
     * It maps the filters to the corresponding points.
     */
    std::map<int, std::vector<int>> P_f;

    /**
     * @brief
     * The set of all filters.
     */
    std::set<int> F;

    /**
     * @brief
     * The label-set for every x in P.
     */
    std::map<int, float> Fx;

    /**
     * @brief
     * Construct a new `F_Vamana` object.
     * @param _dataset
     * The dataset.
     */
    F_Vamana(std::vector<F_Point> &_dataset)
    {
        dataset = _dataset;
    }

    /**
     * @brief
     * Initialize the set of all filters F and map the corresponding points.
     */
    void mapFilters()
    {
        for (auto &p : dataset)
        {
            P_f[p.C].push_back(p.index);
            F.insert(p.C);
            Fx[p.index] = p.C;
        }
    }

    /**
     * @brief
     * Calculate the medoid of the dataset.
     * @param dataset
     * The dataset that you want to calculate the medoid for.
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
     * IGNORE
     *
     * Find the medoids per filter.
     *
     * @param P
     * The dataset P with associated filters for all the points.
     * @param tau
     * The threshold.
     */
    void calculateFilterMedoids(int tau)
    {
        // Initialize M be an empty map, and T to an zero map; T is intended as a counter.
        std::map<int, int> M;
        std::map<int, int> T;

        // foreach f ∈ F, the set of all flters do
        for (auto &f : F)
        {
            // Let P_f ← the ids of all points matching flter f
            std::vector<int> P = P_f[f];
            // Let R_f ← τ randomly sampled data point IDs from P_f.
            std::vector<int> R = randomSample(P, tau);
            // p∗ ← arg min(p ∈ R_f)T[p]
            int p_star = selectLeastLoaded(T, R);
            // M[f] ← p∗ and T[p*] ← T[p*] + 1
            M[f] = p_star;
            T[p_star] = T[p_star] + 1;

            // spdlog::info("+-------------------------------------+");
            // spdlog::info("f  ← {}", f);
            // spdlog::info("P  ← {}", toString(P));
            // spdlog::info("R  ← {}", toString(R));
            // spdlog::info("p* ← {}", p_star);
        }

        medoid_indices = M;
    }

    /**
     * @brief
     * Greedy search algorithm.
     * @param S
     * ~The set of the initial nodes.~ Only 1 filter so we have only 1 starting node.
     * @param x_q
     * The query point.
     * @param k
     * `k` approximate nearest neighbors.
     * @param L_
     * Search list size.
     * @param F_q
     * The query filter(s).
     * @return std::pair<std::vector<Edge>, std::vector<int>>
     * A pair of vectors: the list of the `L_size` nearest neighbors and the list of visited nodes.
     */
    std::pair<std::set<int>, std::set<int>> filteredGreedySearch(int S, F_Point &x_q, int k, int L_) // std::vector<int> F_q)
    {
        // Initialize sets L ← ∅ and V ← ∅
        std::set<int> L;
        std::set<int> V;

        // +-------------------------------------+
        // **IMPORTANT**
        // We take into account only 1 filter!
        // +-------------------------------------+
        // for s ∈ S do
        // if F_s ∩ F_x ≠ ∅ then
        //     L ← L ∪ {s}
        // for (int s : S)
        // {
        //     // if F_s ∩ F_x ≠ ∅ then
        //     bool passFilter = true;
        //     for (int f : F_q)
        //     {
        //         if (dataset[s].C != f)
        //             passFilter = false;
        //     }

        //     if (passFilter)
        //         L.insert(s);
        // }

        if (dataset[S].C == x_q.C)
            L.insert(S);

        //  while L \ V ≠ ∅ do
        while (true)
        {
            std::set<int> L_minus_V = getSetDifference(L, V);
            if (L_minus_V.empty())
                break;

            // Let p* ← arg min(p ∈ L\V) ||x_p − x_q||
            int p_star = -1;
            float min_dist = std::numeric_limits<float>::max();
            for (auto p_idx : L_minus_V)
            {
                float d = euclideanDistance(dataset[p_idx].vec, x_q.vec);
                if (d < min_dist)
                {
                    min_dist = d;
                    p_star = p_idx;
                }
            }

            // V ← V ∪ {p*}
            V.insert(p_star);

            // Let N'out(p*) ← {p' ∈ Nout(p*) : F_p' ∩ F_q ≠ ∅, p' ∉ V}
            std::vector<int> Nout = dataset[p_star].getNeighborIndices();
            std::vector<int> Nout_tune;
            for (int p_tune : Nout)
            {
                bool passFilter = true;
                // +-------------------------------------+
                // **IMPORTANT**
                // We take into account only 1 filter!
                // +-------------------------------------+
                // for (int f : F_q)
                //     if (dataset[p_tune].C != f)
                passFilter = false;

                if (dataset[p_tune].C == x_q.C && V.find(p_tune) == V.end())
                    Nout_tune.push_back(p_tune);
            }

            // L ← L ∪ N'out(p*)
            for (int p_new : Nout_tune)
                L.insert(p_new);

            // if |L| > L_ then
            if ((int)L.size() > L_)
            {
                // Update L with the closest L nodes to x_q
                std::vector<int> L_vec(L.begin(), L.end());
                std::vector<std::pair<float, int>> dist_nodes;
                dist_nodes.reserve(L_vec.size());

                for (int node_idx : L_vec)
                {
                    float d = euclideanDistance(dataset[node_idx].vec, x_q.vec);
                    dist_nodes.emplace_back(d, node_idx);
                }

                std::sort(dist_nodes.begin(), dist_nodes.end(), [](auto &a, auto &b)
                          { return a.first < b.first; });

                dist_nodes.resize(L_);

                L.clear();

                for (auto &dn : dist_nodes)
                    L.insert(dn.second);
            }
        }

        // return [kNNs from L; V]
        return {L, V};
    }

    /**
     * @brief
     * Greedy search algorithm.
     * @param S
     * ~The set of the initial nodes.~ Only 1 filter so we have only 1 starting node.
     * @param x_q
     * The query point.
     * @param k
     * `k` approximate nearest neighbors.
     * @param L_
     * Search list size.
     * @param F_q
     * The query filter(s).
     * @return std::pair<std::vector<Edge>, std::vector<int>>
     * A pair of vectors: the list of the `L_size` nearest neighbors and the list of visited nodes.
     */
    std::pair<std::set<int>, std::set<int>> filteredGreedySearch(int S, F_Query &x_q, int k, int L_) // std::vector<int> F_q)
    {
        // Initialize sets L ← ∅ and V ← ∅
        std::set<int> L;
        std::set<int> V;

        // +-------------------------------------+
        // **IMPORTANT**
        // We take into account only 1 filter!
        // +-------------------------------------+
        // for s ∈ S do
        // if F_s ∩ F_x ≠ ∅ then
        //     L ← L ∪ {s}
        // for (int s : S)
        // {
        //     // if F_s ∩ F_x ≠ ∅ then
        //     bool passFilter = true;
        //     for (int f : F_q)
        //     {
        //         if (dataset[s].C != f)
        //             passFilter = false;
        //     }

        //     if (passFilter)
        //         L.insert(s);
        // }

        if (dataset[S].C == x_q.v)
            L.insert(S);

        //  while L \ V ≠ ∅ do
        while (true)
        {
            std::set<int> L_minus_V = getSetDifference(L, V);
            if (L_minus_V.empty())
                break;

            // Let p* ← arg min(p ∈ L\V) ||x_p − x_q||
            int p_star = -1;
            float min_dist = std::numeric_limits<float>::max();
            for (auto p_idx : L_minus_V)
            {
                float d = euclideanDistance(dataset[p_idx].vec, x_q.vec);
                if (d < min_dist)
                {
                    min_dist = d;
                    p_star = p_idx;
                }
            }

            // V ← V ∪ {p*}
            V.insert(p_star);

            // Let N'out(p*) ← {p' ∈ Nout(p*) : F_p' ∩ F_q ≠ ∅, p' ∉ V}
            std::vector<int> Nout = dataset[p_star].getNeighborIndices();
            std::vector<int> Nout_tune;
            for (int p_tune : Nout)
            {
                bool passFilter = true;
                // +-------------------------------------+
                // **IMPORTANT**
                // We take into account only 1 filter!
                // +-------------------------------------+
                // for (int f : F_q)
                //     if (dataset[p_tune].C != f)
                passFilter = false;

                if (dataset[p_tune].C == x_q.v && V.find(p_tune) == V.end())
                    Nout_tune.push_back(p_tune);
            }

            // L ← L ∪ N'out(p*)
            for (int p_new : Nout_tune)
                L.insert(p_new);

            // if |L| > L_ then
            if ((int)L.size() > L_)
            {
                // Update L with the closest L nodes to x_q
                std::vector<int> L_vec(L.begin(), L.end());
                std::vector<std::pair<float, int>> dist_nodes;
                dist_nodes.reserve(L_vec.size());

                for (int node_idx : L_vec)
                {
                    float d = euclideanDistance(dataset[node_idx].vec, x_q.vec);
                    dist_nodes.emplace_back(d, node_idx);
                }

                std::sort(dist_nodes.begin(), dist_nodes.end(), [](auto &a, auto &b)
                          { return a.first < b.first; });

                dist_nodes.resize(L_);

                L.clear();

                for (auto &dn : dist_nodes)
                    L.insert(dn.second);
            }
        }

        // return [kNNs from L; V]
        return {L, V};
    }

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
    void filteredRobustPrune(F_Point &p, std::set<int> &V, float a, int R)
    {
        // V ← V ∪ Nout(p) \ {p}
        std::set<int> Nout = p.getNeighborIndicesSet();
        for (int n : Nout)
            V.insert(n);
        V.erase(p.index);

        // Nout(p) ← ∅
        p.edges.clear();

        // while V ≠ ∅ do
        while (!V.empty())
        {
            // p* ← arg min(p′ ∈ V) || d(p, p') ||
            int p_star = -1;
            float min_dist = std::numeric_limits<float>::max();
            for (int p_idx : V)
            {
                float d = euclideanDistance(dataset[p.index].vec, dataset[p_idx].vec);
                if (d < min_dist)
                {
                    min_dist = d;
                    p_star = p_idx;
                }
            }

            // Nout(p) ← Nout(p) ∪ {p*}
            p.addNeighborPoint(dataset[p_star], min_dist);

            // if |Nout(p)| = R then
            if (p.edges.size() == R)
                break;

            // for p' ∈ V do
            std::set<int> V_copy = V;
            std::vector<int> to_remove;
            for (int p_tone : V_copy)
            {
                // +-------------------------------------+
                // **IMPORTANT**
                // We take into account only 1 filter!
                // +-------------------------------------+
                // if F_p′ ∩ F_p ⊄ F_p* then
                //     continue
                if (dataset[p_tone].C != p.C && dataset[p_star].C != dataset[p_tone].C)
                    continue;

                // if a · d(p∗, p′) ≤ d(p, p′) then remove p′ from V
                float pstar_to_ptone = euclideanDistance(dataset[p_star].vec, dataset[p_tone].vec);
                float p_to_ptone = euclideanDistance(p.vec, dataset[p_tone].vec);

                if (a * pstar_to_ptone <= p_to_ptone)
                    to_remove.push_back(p_tone);
            }

            // remove v from V
            for (int remove_idx : to_remove)
                V.erase(remove_idx);
        }
    }

    /**
     * @brief
     * Filtered Vamana Indexing Algorithm.
     * @param P
     * @param a
     * @param L
     * @param R
     */
    void filteredVamanaIndexing(float a, int L, int R)
    {
        std::vector<F_Point> P = dataset;
        int n = dataset.size();

        // Initialize G to an empty graph

        // Let s denote the medoid of P
        F_Point s = dataset[medoid_idx];

        // +-------------------------------------+
        // **IMPORTANT**
        // We take into account only 1 filter!
        // +-------------------------------------+
        // Let st(f) denote the start node for flter label f for every f ∈ F
        // std::map<int, int> st = medoid_indices;

        // Let σ be a random permutation of [n]
        std::vector<int> sigma = generateSigma(n);

        // foreach i ∈ [n] do
        progressbar bar(n);
        for (int i = 0; i < n; i++)
        {
            bar.update();

            // +-------------------------------------+
            // **IMPORTANT**
            // We take into account only 1 filter!
            // +-------------------------------------+
            // // Let S_F_x_σ[i] = { st(f) : f in F_X_σ[i] }
            F_Point &x = dataset[sigma[i]];
            // int F_x_sigma_i = Fx[x.index];
            // std::vector<int> S_F_x_sigma_i;
            // S_F_x_sigma_i.push_back(st[F_x_sigma_i]);
            // std::vector<int> query_filters;
            // query_filters.push_back(F_x_sigma_i);
            // // Let [∅; V_F_x_σ(i)] ← FilteredGreedySearch(S_F_x_σ(i), x_σ(i), 0, L, F_x_σ(i))
            // auto p = filteredGreedySearch(S_F_x_sigma_i, x, 0, L, query_filters);
            // std::set<int> ignored_L = p.first;
            // std::set<int> V_F_x_sigma_i = p.second;
            // V ← V ∪ V_F_x_σ(i)
            // Note: the V is missing in the pseudocode.
            // V.insert(V_F_x_sigma_i.begin(), V_F_x_sigma_i.end());

            auto p = filteredGreedySearch(medoid_idx, dataset[sigma[i]], 0, L);

            std::set<int> V = p.second;

            // Run FilteredRobustPrune(x_σ(i), V_F_x_σ(i), a, R) to update neighbors of σ(i)
            // filteredRobustPrune(x, V_F_x_sigma_i, a, R);

            filteredRobustPrune(dataset[sigma[i]], V, a, R);

            // foreach j ∈ Nout(σ(i)) do
            std::vector<int> Nout_x = x.getNeighborIndices();
            for (int j : Nout_x)
            {
                // Update Nout(j) ← Nout(j) ∪ {σ(i)}
                dataset[j].addNeighborPoint(x, euclideanDistance(dataset[j].vec, x.vec));

                // if |Nout(j)| > R then
                if (dataset[j].edges.size() > R)
                {
                    std::set<int> Nout_j = dataset[j].getNeighborIndicesSet();
                    // Run FilteredRobustPrune(j, Nout(j), a, R) to update out-neighbors of j
                    filteredRobustPrune(dataset[j], Nout_j, a, R);
                }
            }

            // Run Filtered Greedy Search with S = S_F_x_sigma[i], query = x_sigm[i],
            // and query filters = F_x_sigma[i]
            // Let [∅; V_F_x_σ(i)] ← FilteredGreedySearch(S_F_x_σ(i), x_σ(i), 0, L, F_x_σ(i))
            // V ← V ∪ V_F_x_σ(i)
            // Run FilteredRobustPrune(x_σ(i), V_F_x_σ(i), a, R) to update neighbors of σ(i)
            // foreach j ∈ Nout(σ(i)) do
            //     Update Nout(j) ← Nout(j) ∪ {σ(i)}
            //     if |Nout(j)| > R then
            //         Run FilteredRobustPrune(j, Nout(j), a, R) to update out-neighbors of j
        }
    }

    /**
     * @brief
     * Brute force nearest neighbors search.
     * @param q
     * The query vector.
     * @param k
     * `k` approximate nearest neighbors.
     * @return std::vector<int>
     * The indices of the `k` nearest neighbors.
     */
    std::vector<int> bruteForceNearestNeighbors(F_Query &q, int k)
    {
        // Vector to store pairs of distance and index
        std::vector<std::pair<float, int>> distances;

        // Compute distance from the query point to every point in the dataset
        for (F_Point &p : dataset)
        {
            float dist = euclideanDistance(q.vec, p.vec);
            distances.push_back(std::make_pair(dist, p.index));
        }

        // Sort the distances in ascending order
        std::sort(
            distances.begin(),
            distances.end(),
            [](std::pair<float, int> &a, std::pair<float, int> &b)
            { return a.first < b.first; });

        // Extract the indices of the k nearest neighbors
        std::vector<int> neighbors;
        for (int i = 0; i < k && i < distances.size(); ++i)
        {
            neighbors.push_back(distances[i].second);
        }

        return neighbors;
    }
};

/**
 * @brief
 * Parse the sigmod-contest dummy data.
 * @param filepath
 * The path to the file.
 * @param no_dimensions
 * The dimensions of each point's vector.
 * @return std::vector<F_Point>
 * The dataset of points.
 */
std::vector<F_Point> parseDummyData(std::string &filepath, int no_dimensions)
{
    std::ifstream ifs;
    ifs.open(filepath, std::ios::binary);
    assert(ifs.is_open());

    // 1. The first four bytes(int) represent the number of points in the file.
    uint32_t N;
    ifs.read((char *)&N, sizeof(uint32_t));

    std::vector<F_Point> points;
    std::vector<float> buff(no_dimensions);

    // 2. Read and parse each point.
    for (int _idx = 0; _idx < N; ++_idx)
    {
        // 3. Read point and store it in buff.
        if (!ifs.read(reinterpret_cast<char *>(buff.data()), no_dimensions * sizeof(float)))
        {
            std::cerr << "Error reading point data at index " << _idx << std::endl;
            break;
        }

        // 4. Parse metadata.
        float _c = buff[0];
        float _t = buff[1];

        // 5. Parse vector data (excluding the first 4 metadata values)
        std::vector<float> _vec(buff.begin() + 2, buff.end());

        // 6. Create a new point placing the category in the category argument an timestamp in both bounds(lower , upper).
        F_Point new_point = F_Point(_idx, _vec, _c, _t);

        // 7. Add it to the database.
        points.push_back(new_point);
    }

    // 8. Close file and return the database
    ifs.close();

    return points;
}

/**
 * @brief
 * Parse the sigmod-contest dummy queries.
 * @param filepath
 * The path to the file.
 * @param no_dimensions
 * The dimensions of each query's vector.
 * @return std::vector<F_Query>
 * The dataset of queries.
 */
std::vector<F_Query> parseDummyQueries(std::string &filepath, int no_dimensions)
{
    std::ifstream ifs;
    ifs.open(filepath, std::ios::binary);
    assert(ifs.is_open());

    // 1. The first four bytes(int) represent the number of points in the file.
    uint32_t N;
    ifs.read((char *)&N, sizeof(uint32_t));

    std::vector<F_Query> queries;
    std::vector<float> buff(no_dimensions);

    // 2. Read and parse each point.
    for (int _idx = 0; _idx < N; ++_idx)
    {
        // 3. Read point and store it in buff.
        if (!ifs.read(reinterpret_cast<char *>(buff.data()), no_dimensions * sizeof(float)))
            break;

        // 4. Parse metadata.
        QueryType _query_type = static_cast<QueryType>(buff[0]);
        float _v = buff[1];
        float _l = buff[2];
        float _r = buff[3];

        // 5. Parse vector data (excluding the first 4 metadata values)
        std::vector<float> _vec(buff.begin() + 4, buff.end());

        // 6. Create a new F_Point object.
        F_Query new_query = F_Query(
            _idx,
            _vec,
            _query_type,
            _v,
            _l,
            _r);

        // 7. Add it to the database.
        queries.push_back(new_query);
    }

    // 8. Close file and return the database
    ifs.close();

    return queries;
}

#endif // VAMANA_FILTERED_H
