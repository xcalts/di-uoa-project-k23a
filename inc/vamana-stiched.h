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
