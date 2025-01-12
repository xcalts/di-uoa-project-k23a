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
#include "vamana.h"
#include "vamana-stiched.h"
#include "math.h"
#include "sets.h"
#include "vectors.h"

/**************/
/* Namespaces */
/**************/
using namespace indicators;

StichedVamanaStatistics::StichedVamanaStatistics() {}

StichedVamana::StichedVamana(std::vector<Point> &_dataset) : dataset(_dataset) {}

void StichedVamana::initializingEmptyGraph()
{
    for (Point &p : dataset)
    {
        F.insert(p.C);
        P_f[p.C].push_back(p);
    }
}

std::vector<int> StichedVamana::randomSample(std::vector<int> &P_f, int tau)
{
    std::vector<int> R_f;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(P_f.begin(), P_f.end(), g);

    for (int i = 0; i < tau; i++)
        R_f.push_back(P_f[i]);

    return R_f;
}

std::vector<int> StichedVamana::getPointsWithFilter(float f)
{
    std::vector<int> _f;

    for (Point &p : dataset)
    {
        if (p.C == f)
            _f.push_back(p.index);
    }

    return _f;
}

std::map<float, int> StichedVamana::findMedoids(int tau)
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

std::pair<std::set<int>, std::set<int>> StichedVamana::filteredGreedySearch(const std::set<int> &S, const Query &x_q, int k, int L_, const std::set<float> &F_q)
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

StichedVamanaStatistics StichedVamana::index(float a, int L_small, int R_small, int R_stiched)
{
    sw::Stopwatch stopwatch;
    StichedVamanaStatistics statistics;

    stopwatch.start();

    // Initialize G = (V, E) to an empty graph
    // Let F_x ⊆ F be the label-set for every x ∈ P
    // Let P_f ⊆ P be the set of points with label f ∈ F
    initializingEmptyGraph();

    int i = 0;
    // foreach f ∈ F do
    for (float f : F)
    {
        std::cout << "== Filter: " << i << " ==" << std::endl;

        i++;

        // Let G_f ← Vamana(P_f, a, Rsmall , Lsmall )
        G_f[f] = Vamana();
        if (P_f[f].size() <= 1)
            continue;

        G_f[f].index(dataset, P_f[f], a, L_small, R_small);
    }

    statistics.stiched_vamana_indexing_time = stopwatch.elapsed<sw::s>();

    return statistics;

    // Bad results
    // foreach v ∈ V do
    //     FilteredRobustPrune (V, Nout(v), a, Rstitched )
}

void StichedVamana::saveGraph(const std::string &filepath)
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

void StichedVamana::loadGraph(const std::string &filepath)
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
