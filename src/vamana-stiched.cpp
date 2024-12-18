#ifndef VAMANA_STICHED_H
#define VAMANA_STICHED_H

/**********************/
/* Standard Libraries */
/**********************/

#include <vector>
#include <set>

/**********************/
/* External Libraries */
/**********************/

#include "progressbar.h"
#include "vamana.h"

/**********************/
/* Project Components */
/**********************/

#include "misc.h"

/***************/
/* Definitions */
/***************/

class F_Point;
class F_Query;
class F_Vamana;
// class S_Vamana;

void saveResultsBinary(const std::map<int, std::set<int>> &results, const std::string &filename);
std::vector<F_Point> parseDummyData(std::string &, int);
std::vector<F_Query> parseDummyQueries(std::string &, int);

/*******************/
/* Implementations */
/*******************/

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
     * @param neighbor_index
     * The neighbor to add.
     */
    void addNeighbor(int neighbor_index)
    {
        // Ensure we're not adding a self-loop
        if (neighbor_index != this->index)
        {
            neighbors.insert(neighbor_index);
        }
    }
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
    std::vector<int> randomSample(std::vector<int> &P_f, int tau)
    {
        std::vector<int> R_f;
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(P_f.begin(), P_f.end(), g);

        for (int i = 0; i < tau; i++)
            R_f.push_back(P_f[i]);

        return R_f;
    }

    /**
     * @brief
     * Get the points that match that filter.
     * @param f
     * The filter.
     * @return std::vector<int>
     */
    std::vector<int> getPointsWithFilter(float f)
    {
        std::vector<int> _f;

        for (F_Point &p : dataset)
        {
            if (p.C == f)
                _f.push_back(p.index);
        }

        return _f;
    }

public:
    /**
     * @brief
     * It contains all the points of the dataset.
     */
    std::vector<F_Point> dataset;

    /**
     * @brief
     * Filter ---> Points.
     */
    std::map<float, std::vector<int>> P_f;

    /**
     * @brief
     * The set of all filters.
     */
    std::set<float> F;

    int s;

    std::map<float, int> st;

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
     * Calculate the medoid of the dataset.
     * @param dataset
     * The dataset that you want to calculate the medoid for.
     */
    int findMedoid()
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

        return _medoid_idx;
    }

    /**
     * @brief
     * Find the medoids per filter.
     * @param P
     * The dataset P with associated filters for all the points.
     * @param tau
     * The threshold.
     */
    std::map<float, int> findMedoids(int tau)
    {
        // Initialize M be an empty map, and T to an zero map; T is intended as a counter.
        std::map<float, int> M;
        std::map<int, int> T;

        // foreach f ∈ F, the set of all flters do
        for (auto &f : F)
        {
            // Let P_f ← the ids of all points matching flter f
            std::vector<int> P_f = getPointsWithFilter(f);

            // Let R_f ← τ randomly sampled data point IDs from P_f.
            std::vector<int> R_f = randomSample(P_f, std::min<int>(tau, P_f.size()));

            // p∗ ← arg min(p ∈ R_f)T[p]
            int p_star = R_f[0];
            for (int p : R_f)
            {
                if (T[p] < T[p_star])
                    p_star = p;
            }

            // M[f] ← p∗ and T[p*] ← T[p*] + 1
            M[f] = p_star;
            T[p_star]++;

            // spdlog::info("+-------------------------------------+");
            // spdlog::info("f  ← {}", f);
            // spdlog::info("P  ← {}", toString(P));
            // spdlog::info("R  ← {}", toString(R));
            // spdlog::info("p* ← {}", p_star);
        }

        // return M
        return M;
    }

    /**
     * @brief
     * Implementation of the greedy search algorithm.
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
    std::pair<std::set<int>, std::set<int>> filteredGreedySearch(std::set<int> S, int x_q, std::vector<float> x_vec, int k, int L_, std::set<float> F_q)
    {
        // Initialize sets L ← ∅ and V ← ∅
        std::set<int> L;
        std::set<int> V;

        // +---------------------------------+
        // **IMPORTANT**
        // We take into account only 1 filter!
        // +---------------------------------+

        // for s ∈ S do
        for (int s : S)
        {
            // if F_s ∩ F_x ≠ ∅ then
            bool _p = false;
            for (int f : F_q)
            {
                if (dataset[s].C == f) // <-- :(
                {
                    _p = true;
                    break;
                }
            }

            // L ← L ∪ { s }
            if (_p)
                L.insert(s);
        }

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
                float d = euclideanDistance(dataset[p_idx].vec, x_vec);
                if (d < min_dist)
                {
                    min_dist = d;
                    p_star = p_idx;
                }
            }

            // V ← V ∪ {p*}
            V.insert(p_star);

            // +---------------------------------+
            // **IMPORTANT**
            // We take into account only 1 filter!
            // +---------------------------------+
            // Let N'out(p*) ← {p' ∈ Nout(p*) : F_p' ∩ F_q ≠ ∅, p' ∉ V}
            std::set<int> Nout = dataset[p_star].neighbors;
            std::set<int> Nout_tune;
            for (int p_tune : Nout)
            {
                bool _p = false;

                for (int f : F_q)
                {
                    if (dataset[p_tune].C == f)
                    {
                        _p = true; // <-- :(
                        break;
                    }
                }

                if (_p && V.count(p_tune) == 0)
                    Nout_tune.insert(p_tune);
            }

            // L ← L ∪ N'out(p*)
            for (int p_new : Nout_tune)
                L.insert(p_new);

            // if |L| > L_ then
            //     Update L with the closest L nodes to x_q
            if (L.size() > L_)
            {
                // Calculate the distance for each point in L.
                std::multiset<std::pair<int, float>> point_distance;

                for (int l : L)
                    point_distance.emplace(l, euclideanDistance(dataset[l].vec, dataset[x_q].vec));

                // Keep only the closest L points.
                std::set<int> new_L;
                int count = 0;
                for (const auto p_d : point_distance)
                {
                    if (count >= L_)
                        break; // Stop after X elements
                    new_L.insert(p_d.first);
                    ++count;
                }

                L = std::move(new_L);
            }
        }

        // return the k closest points of L
        // Get the k closest points from L
        std::multiset<std::pair<int, float>> point_distance;
        for (int l : L)
            point_distance.emplace(l, euclideanDistance(dataset[l].vec, x_vec));

        std::set<int> k_closest_points;
        int count = 0;
        for (const auto p_d : point_distance)
        {
            if (count >= k)
                break; // Stop after k elements
            k_closest_points.insert(p_d.first);
            ++count;
        }

        // return the k closest points of L and V

        // return [kNNs from L; V]
        return {k_closest_points, V};
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
    void filteredRobustPrune(int p, std::set<int> &V, float a, int R)
    {
        // V ← V ∪ Nout(p) \ {p}
        std::set<int> Nout = dataset[p].neighbors;
        for (int n : Nout)
            V.insert(n);
        V.erase(p);

        // Nout(p) ← ∅
        dataset[p].neighbors.clear();

        // while V ≠ ∅ do
        while (!V.empty())
        {
            // p* ← arg min(p′ ∈ V) || d(p, p') ||
            int p_star = -1;
            float min_d = std::numeric_limits<float>::max();
            for (int p_tune : V)
            {
                float d = euclideanDistance(dataset[p].vec, dataset[p_tune].vec);
                if (d < min_d)
                {
                    min_d = d;
                    p_star = p_tune;
                }
            }

            // Nout(p) ← Nout(p) ∪ {p*}
            dataset[p].addNeighbor(p_star);

            // if |Nout(p)| = R then
            if (dataset[p].neighbors.size() == R)
                break;

            // for p' ∈ V do
            std::set<int> V_copy = V;
            std::vector<int> to_remove;
            for (int p_tune : V_copy)
            {
                // +-------------------------------------+
                // **IMPORTANT**
                // We take into account only 1 filter!
                // +-------------------------------------+
                // if F_p′ ∩ F_p ⊄ F_p* then
                //     continue
                if (dataset[p_tune].C != dataset[p].C)
                    if ((int)dataset[p_star].C == -1)
                        continue;
                    else if (dataset[p_star].C != dataset[p_tune].C)
                        continue;

                if (dataset[p_tune].C != dataset[p].C && dataset[p_star].C != dataset[p_tune].C)
                    continue;

                // if a · d(p∗, p′) ≤ d(p, p′) then remove p′ from V
                float pstar_to_ptone = euclideanDistance(dataset[p_star].vec, dataset[p_tune].vec);
                float p_to_ptone = euclideanDistance(dataset[p].vec, dataset[p_tune].vec);

                if (a * pstar_to_ptone <= p_to_ptone)
                    to_remove.push_back(p_tune);
            }

            // remove v from V
            for (int remove_idx : to_remove)
                V.erase(remove_idx);
        }
    }

    /**
     * @brief
     * Impleentation of the filtered vamana indexing algorithm.
     * @param a
     * @param L
     * @param R
     */
    void filteredVamanaIndexing(int tau, float a, int L, int R)
    {
        // Initialize G to an empty graph
        for (auto &p : dataset)
        {
            F.insert(p.C);
            P_f[p.C].push_back(p.index);
        }

        // Let s denote the medoid of P
        // int s = findMedoid();
        s = 5234;

        // +-------------------------------------+
        // **IMPORTANT**
        // We take into account only 1 filter!
        // +-------------------------------------+
        // Let st(f) denote the start node for flter label f for every f ∈ F
        st = findMedoids(tau);

        // Let σ be a random permutation of [n]
        int n = dataset.size();
        std::vector<int> sigma = generateSigma(n);

        // Let F_x be the label-set for every x ∈ P
        std::map<int, float> F_x;
        for (F_Point &p : dataset)
            F_x[p.index] = p.C;

        // foreach i ∈ [n] do
        progressbar bar(n);
        for (int i = 0; i < n; i++)
        {
            bar.update();
            // +-------------------------------------+
            // **IMPORTANT**
            // We take into account only 1 filter!
            // +-------------------------------------+
            // Let S_F_x_σ[i] = { st(f) : f in F_X_σ[i] }
            std::set<int> S_F_sigma_i;
            std::set<float> F_sigma_i = {F_x[sigma[i]]};
            S_F_sigma_i.insert(st[F_x[sigma[i]]]);

            // Let [∅; V_F_x_σ(i)] ← FilteredGreedySearch(S_F_x_σ(i), x_σ(i), 0, L, F_x_σ(i))
            auto p = filteredGreedySearch(S_F_sigma_i, sigma[i], dataset[sigma[i]].vec, 0, L, F_sigma_i);

            // V ← V ∪ V_F_x_σ(i)
            // V is missing from the pseudocode
            std::set<int> V = p.second;

            // Run FilteredRobustPrune(x_σ(i), V_F_x_σ(i), a, R)
            // to update neighbors of σ(i)
            filteredRobustPrune(sigma[i], V, a, R);

            // foreach j ∈ Nout(σ(i)) do
            std::set<int> Nout_x = dataset[sigma[i]].neighbors;
            for (int j : Nout_x)
            {
                // Update Nout(j) ← Nout(j) ∪ {σ(i)}
                dataset[j].addNeighbor(sigma[i]);

                // if |Nout(j)| > R then
                if (dataset[j].neighbors.size() > R)
                {
                    // Run FilteredRobustPrune(j, Nout(j), a, R) to update out-neighbors of j
                    // std::set<int> &Nout_j = dataset[j].neighbors;
                    filteredRobustPrune(j, dataset[j].neighbors, a, R);
                }
            }
        }
    }

    /**
     * @brief
     * Creates a Graph out of a dataset with filters.
     * Calls vamana index for each filter's dataset of compatible points. Running with smaller database can be more efficient.
     * Connects all the smaller graphs to one.
     * @param a
     * Scaling factor to prune outgoing edges of a node (alpha).
     * @param L_small
     * Maximum list of search candidates to use in graph traversal, must be small to be more efficient.
     * @param R_small
     * Maximum number of outgoing edges of a node, must be small to be more efficient it is used in smaller graphs.
     * @param R_stiched
     * Maximum number of outgoing edges of a node, after the smaller graphs are stiched back together.
     * @param tau
     * The threshold.
     */
    void StichedVamanaIndex(float a, int L_small, int R_small, int R_stiched, int tau)
    {
        // A map to save the results of the stiched vamana and record the to a file
        // std::map<int, std::set<int>> results;

        // initialize G  = (V , E) to an empty graph
        for (auto &p : dataset)
        {
            F.insert(p.C);
            P_f[p.C].push_back(p.index);
        }
        // Let st ⊆ F be the label-set for every x ∈ P
        st = findMedoids(tau);

        // Let Pf ⊆ P be the set of points with label f ∈ F
        std::vector<Point> pf;

        // foreach f ∈ F do
        progressbar bar(st.size());
        for (auto f : st)
        {
            bar.update();
            // map with real id and new id
            std::map<int, int> old_Id;

            // empty pf vector
            pf.clear();
            if (P_f[f.first].size() == 1)
            {
                // empty list of neighbors just the medoid
                // results[f.second] = {};
                continue;
            }
            // fill pf with points of filter f and add to each point a new id according to current position, we will restore it later
            int new_id = 0;
            for (int p_f : P_f[f.first])
            {
                // create new point , add new point to pf , store old_Id of p to a map in order to restore
                Point new_p = Point(new_id, dataset[p_f].vec);
                pf.push_back(new_p);
                old_Id[new_id] = p_f;
                new_id++;
            }
            // Let Gf ← Vamana(Pf, a, R_small ,L_small)
            Vamana Pf = Vamana(pf);
            Pf.calculateMedoid();

            if (R_small >= pf.size())
                // If R_small is larger that the database re arrange R_small
                Pf.index(a, L_small, pf.size() - 1);
            else
                Pf.index(a, L_small, R_small);

            std::vector<Point> Gf = Pf.dataset;

            // combine Gf with G
            for (Point &p : Gf)
            {
                int old_id = old_Id.at(p.index);
                std::set<int> p_neighbors;
                for (Edge &neighbor : p.outgoing_edges)
                {
                    p_neighbors.emplace(neighbor.to_index);
                }
                dataset[old_id].neighbors = p_neighbors;
                // results[old_id] = p_neighbors;
            }
        }

        // foreach v ∈ V do   // this can be ignored because this points dont overlap through filters
        // for(Point &p : dataset){

        // filteredRobustPrune(v , Nout(v) , a , R_stiched)
        //    FilteredRobustPrune(p.index , p.neighbors , a , R_stiched);
        //}

        // Save results to a binary file
        // saveResultsBinary(results, "stitched_vamana_results.bin");
    }
};

/**
 * @brief
 * Saves map to a file
 * @param results
 * A map that contains keys and a set of values about the key
 * @param filename
 * Name of the file to store the data
 */
void saveResultsBinary(const std::map<int, std::set<int>> &results, const std::string &filename)
{
    std::ofstream outputFile(filename, std::ios::binary);
    if (!outputFile.is_open())
    {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }

    for (const auto &pair : results)
    {
        int key = pair.first;
        size_t setSize = pair.second.size();
        outputFile << "Point " << key << ": ";
        for (int neighbor : pair.second)
        {
            outputFile << neighbor << " ";
        }
        outputFile << "\n";
    }
    outputFile.close();
}

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

#endif // VAMANA_STICHED_H
