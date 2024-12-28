/**********************/
/* Standard Libraries */
/**********************/
#include <random>
#include <chrono>
#include <thread>
#include <set>
#include <fstream>

/**********************/
/* External Libraries */
/**********************/
#include "stopwatch.h"
#include <indicators/block_progress_bar.hpp>

/**********************/
/* Project Components */
/**********************/
#include "vamana-filtered.h"
#include "math.h"
#include "sets.h"
#include "vectors.h"

/**************/
/* Namespaces */
/**************/
using namespace indicators;

FilteredVamanaStatistics::FilteredVamanaStatistics() {}

FilteredVamana::FilteredVamana(const std::vector<Point> &_dataset)
{
    dataset = _dataset;
}

void FilteredVamana::initializingEmptyGraph()
{
    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::blue},
        option::PrefixText{"   Initializing empty G graph"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{dataset.size()}};

    int i = 0;
    for (auto &p : dataset)
    {
        F.insert(p.C);
        P_f[p.C].push_back(p.index);

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(dataset.size())});

        // update progress bar
        bar.tick();

        i++;
    }

    bar.mark_as_completed();
}

std::vector<int> FilteredVamana::randomSample(std::vector<int> &P_f, int tau)
{
    std::vector<int> R_f;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(P_f.begin(), P_f.end(), g);

    for (int i = 0; i < tau; i++)
        R_f.push_back(P_f[i]);

    return R_f;
}

std::vector<int> FilteredVamana::getPointsWithFilter(float f)
{
    std::vector<int> _f;

    for (Point &p : dataset)
    {
        if (p.C == f)
            _f.push_back(p.index);
    }

    return _f;
}

std::map<float, int> FilteredVamana::findMedoids(int tau)
{
    // Initialize M be an empty map, and T to an zero map; T is intended as a counter.
    std::map<float, int> M;
    std::map<int, int> T;

    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::yellow},
        option::PrefixText{"   Finding Medoids per Filter"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{F.size()}};

    // foreach f ∈ F, the set of all flters do
    int i = 0;
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

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(F.size())});

        // update progress bar
        bar.tick();

        i++;
    }

    bar.mark_as_completed();

    // return M
    return M;
}

std::pair<std::set<int>, std::set<int>> FilteredVamana::filteredGreedySearch(const std::set<int> &S, const Query &x_q, int k, int L_, const std::set<float> &F_q)
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
        bool _p = true;
        for (int f : F_q)
            if (dataset[s].C != f) // <-- :(
                _p = false;

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
            float d = euclideanDistance(dataset[p_idx].vec, x_q.vec);
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
        std::set<int> Nout_tune;
        for (int p_tune : dataset[p_star].neighbors)
        {
            bool _p = true;

            for (int f : F_q)
                if (dataset[p_tune].C != f)
                    _p = false; // <-- :(

            if (_p && V.find(p_tune) == V.end())
                // L ← L ∪ N'out(p*)
                L.insert(p_tune);
        }

        // if |L| > L_ then
        //     Update L with the closest L nodes to x_q
        if (L.size() > L_)
        {
            // Calculate the distance for each point in L.
            std::vector<std::pair<int, float>> point_distance;

            for (int l : L)
                point_distance.emplace_back(l, euclideanDistance(dataset[l].vec, x_q.vec));

            // Sort the point_distance by ascending distance (the float part).
            std::sort(point_distance.begin(), point_distance.end(), [](const auto &a, const auto &b)
                      { return a.second < b.second; });

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

    // Calculate the distance for each point in L.
    std::vector<std::pair<int, float>> point_distance;

    for (int l : L)
        point_distance.emplace_back(l, euclideanDistance(dataset[l].vec, x_q.vec));

    // Sort the point_distance by ascending distance (the float part).
    std::sort(point_distance.begin(), point_distance.end(), [](const auto &a, const auto &b)
              { return a.second < b.second; });

    std::set<int> new_L;
    int count = 0;
    for (const auto &p_d : point_distance)
    {
        if (count >= k)
            break;
        new_L.insert(p_d.first);
        ++count;
    }

    L = std::move(new_L);

    // return [kNNs from L; V]
    return std::make_pair(L, V);
}

void FilteredVamana::filteredRobustPrune(int p, std::set<int> &V, float a, int R)
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

FilteredVamanaStatistics FilteredVamana::index(int tau, float a, int L, int R)
{
    sw::Stopwatch stopwatch;
    FilteredVamanaStatistics statistics;

    // Initialize G to an empty graph
    initializingEmptyGraph();

    // Let s denote the medoid of P
    // s = findMedoid(dataset);
    stopwatch.start();
    s = 5234;
    statistics.medoid_calculation_time = stopwatch.elapsed<sw::s>();

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
    for (const Point &p : dataset)
        F_x[p.index] = p.C;

    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::red},
        option::PrefixText{"     Filtered Vamana Indexing"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{dataset.size()}};

    // foreach i ∈ [n] do
    stopwatch.start();
    for (int i = 0; i < n; i++)
    {
        // +-------------------------------------+
        // **IMPORTANT**
        // We take into account only 1 filter!
        // +-------------------------------------+
        // Let S_F_x_σ[i] = { st(f) : f in F_X_σ[i] }
        std::set<int> S_F_sigma_i;
        std::set<float> F_sigma_i = {F_x[sigma[i]]};
        S_F_sigma_i.insert(st[F_x[sigma[i]]]);

        // Let [∅; V_F_x_σ(i)] ← FilteredGreedySearch(S_F_x_σ(i), x_σ(i), 0, L, F_x_σ(i))
        Query x_q = Query(dataset[sigma[i]].vec);
        auto p = filteredGreedySearch(S_F_sigma_i, x_q, 0, L, F_sigma_i);

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

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(dataset.size())});

        // update progress bar
        bar.tick();
    }
    statistics.filtered_vamana_indexing_time = stopwatch.elapsed<sw::s>();

    bar.mark_as_completed();

    return statistics;
}

void FilteredVamana::saveGraph(const std::string &filepath)
{
    std::ofstream ofs(filepath);
    if (!ofs.is_open())
    {
        std::cerr << "Error: Could not open file for saving: " << filepath << std::endl;
        return;
    }

    // For each point in dataset, write:
    // index:neighbor1,neighbor2,neighbor3,...
    for (int i = 0; i < static_cast<int>(dataset.size()); i++)
    {
        // Print the index
        ofs << dataset[i].index << ":";

        // Print neighbors in comma-separated fashion
        bool first = true;
        for (int neighbor : dataset[i].neighbors)
        {
            if (!first)
                ofs << ",";
            ofs << neighbor;
            first = false;
        }

        ofs << "\n"; // New line for each point
    }

    ofs.close();
}

void FilteredVamana::loadGraph(const std::string &filepath)
{
    std::ifstream ifs(filepath);
    if (!ifs.is_open())
    {
        std::cerr << "Error: Could not open file for loading: " << filepath << std::endl;
        return;
    }

    // We'll read each line in "index:neighbor1,neighbor2,..." format
    std::string line;
    // Temporary adjacency list to store what we parse from file
    std::vector<std::set<int>> adjacency;

    while (std::getline(ifs, line))
    {
        // Find position of ':'
        std::size_t colonPos = line.find(':');
        if (colonPos == std::string::npos)
        {
            // Malformed line or empty
            continue;
        }

        // Parse the index
        int idx = std::stoi(line.substr(0, colonPos));

        // Parse the neighbors substring (everything after ':')
        std::string neighborsStr = line.substr(colonPos + 1);

        // Split neighbors on comma
        std::set<int> neighborSet;
        {
            std::stringstream ss(neighborsStr);
            std::string segment;
            while (std::getline(ss, segment, ','))
            {
                if (!segment.empty())
                {
                    neighborSet.insert(std::stoi(segment));
                }
            }
        }

        // Resize adjacency vector if needed
        if (idx >= static_cast<int>(adjacency.size()))
        {
            adjacency.resize(idx + 1);
        }
        adjacency[idx] = neighborSet;
    }

    ifs.close();

    // Ensure our main dataset can hold [0..(adjacency.size()-1)] as indexes
    // If dataset is not yet sized or is smaller than we need, we resize here.
    if (dataset.size() < adjacency.size())
    {
        dataset.resize(adjacency.size());
        for (int i = 0; i < static_cast<int>(dataset.size()); i++)
        {
            dataset[i].index = i;
        }
    }

    // Now copy the adjacency into dataset
    for (int i = 0; i < static_cast<int>(adjacency.size()); i++)
    {
        dataset[i].neighbors = adjacency[i];
    }
}
