/**********************/
/* Standard Libraries */
/**********************/
#include <algorithm>
#include <random>
#include <sstream>
#include <chrono>
#include <thread>
#include <set>
#include <fstream>
#include <limits>

/**********************/
/* External Libraries */
/**********************/
#include "stopwatch.h"
#include <indicators/block_progress_bar.hpp>

/**********************/
/* Project Components */
/**********************/
#include "vamana.h"
#include "math.h"
#include "sets.h"
#include "vectors.h"

/**************/
/* Namespaces */
/**************/
using namespace indicators;

VamanaStatistics::VamanaStatistics() {}

Vamana::Vamana() {}

void Vamana::generateRandomGraphEdges(std::vector<Point> &dataset, std::vector<Point> &P, int R)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, P.size() - 1);

    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::grey},
        option::PrefixText{"Generating Random Graph Edges"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{P.size()}};

    int i = 0;
    for (const Point &p : P)
    {
        int attempts = 0;
        int max_attempts = P.size() * 2; // Arbitrary large number to prevent infinite loop
        while (dataset[p.index].neighbors.size() < R && attempts < max_attempts)
        {
            dataset[p.index].neighbors.insert(distrib(gen));
            attempts++;
        }

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(P.size())});

        // update progress bar
        bar.tick();

        i++;
    }

    bar.mark_as_completed();
}

void Vamana::robustPrune(std::vector<Point> &dataset, Point &p, std::set<int> &V, float a, int R)
{
    // V ← (V ∪ Nout(p)) \ {p}
    for (int n : dataset[p.index].neighbors)
        V.insert(n);

    V.erase(p.index);

    // Nout(p) ← ∅
    dataset[p.index].neighbors.clear();

    // while V != ∅ do
    while (!V.empty())
    {
        // p∗ ← arg min p'∈V d(p, p')
        int pstar_idx = -1;
        float pstar_dist = std::numeric_limits<float>::max();
        for (int ptune_idx : V)
        {
            float d = euclideanDistance(dataset[ptune_idx].vec, p.vec);
            if (d < pstar_dist)
            {
                pstar_dist = d;
                pstar_idx = ptune_idx;
            }
        }

        // Nout(p) ← Nout(p) ∪ {p∗}
        dataset[p.index].neighbors.insert(pstar_idx);

        // if |Nout(p)| = R then break
        if (dataset[p.index].neighbors.size() == R)
            break;

        // for p' ∈ V do
        std::vector<int> to_remove;
        for (int ptune_idx : V)
        {
            float pstar_to_ptune = euclideanDistance(dataset[pstar_idx].vec, dataset[ptune_idx].vec);
            float p_to_ptune = euclideanDistance(dataset[p.index].vec, dataset[ptune_idx].vec);

            // if α · d(p∗, p') ≤ d(p, p') then remove p' from V
            if (a * pstar_to_ptune <= p_to_ptune)
                to_remove.push_back(ptune_idx);
        }

        for (auto idx : to_remove)
            V.erase(idx);
    }
}

std::pair<std::set<int>, std::set<int>> Vamana::greedySearch(std::vector<Point> &dataset, Point &s, Query &x_q, int k, int L_)
{
    // initialize sets L ← {s} and V ← ∅
    std::set<int> L = {s.index};
    std::set<int> V;

    // While L\V != {}
    std::set<int> L_minus_V;
    while (true)
    {
        L_minus_V = getSetDifference(L, V);

        // L\V ← {}, break
        if (L_minus_V.empty())
            break;

        // p* ← min(||xp −xq||, p ∈ L\V)
        int pstar_idx = -1;
        float min_dist = std::numeric_limits<float>::max();
        for (int p_idx : L_minus_V)
        {
            float d = euclideanDistance(dataset[p_idx].vec, x_q.vec);
            if (d < min_dist)
            {
                min_dist = d;
                pstar_idx = p_idx;
            }
        }

        // update L ← L ∪ Nout (p∗ ) and V ← V ∪ {p∗ }
        for (int n : dataset[pstar_idx].neighbors)
            L.insert(n);
        V.insert(pstar_idx);

        // If |L| > L_
        // +> update L to retain closest L points to x_q
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
            for (const auto &p_d : point_distance)
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

    return std::make_pair(L, V);
}

VamanaStatistics Vamana::index(std::vector<Point> &dataset, std::vector<Point> &P, float a, int L, int R)
{
    sw::Stopwatch stopwatch;
    VamanaStatistics statistics;

    // initialize G to a random R-regular directed graph
    generateRandomGraphEdges(dataset, P, R);

    // let s denote the medoid of dataset P
    stopwatch.start();
    S = findMedoid(P);
    statistics.medoid_calculation_time = stopwatch.elapsed<sw::s>();

    // let σ denote a random permutation of 1..n
    std::vector<int> sigma = generateSigma(P.size());

    // for 1 ≤ i ≤ n do
    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::blue},
        option::PrefixText{"              Vamana Indexing"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{P.size()}};

    stopwatch.start();
    for (int i = 0; i < P.size(); ++i)
    {
        Point &s_i = dataset[sigma[i]];

        // let [L; V] ← GreedySearch(s, xσ(i) , 1, L)
        Query x_q(s_i.vec);
        std::pair<std::set<int>, std::set<int>> r = greedySearch(dataset, S, x_q, 1, L);

        // run RobustPrune(σ(i), V, α, R) to update out-neighbors of σ(i)
        robustPrune(dataset, s_i, r.second, a, R);

        // for all points j in Nout (σ(i)) do
        for (int j : s_i.neighbors)
        {
            Point &J = dataset[j];

            // if |Nout(j) ∪ {σ(i)}| > R then
            std::set<int> U = J.neighbors;
            U.insert(sigma[i]);

            if (U.size() > R)
                robustPrune(dataset, J, U, a, R);
            // else update Nout (j) ← Nout (j) ∪ σ(i)
            else
                J.neighbors.insert(sigma[i]);
        }

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(P.size())});

        // update progress bar
        bar.tick();
    }
    statistics.vamana_indexing_time = stopwatch.elapsed<sw::s>();

    bar.mark_as_completed();

    return statistics;
}

void Vamana::saveGraph(std::vector<Point> &dataset, const std::string &filepath)
{
    std::ofstream ofs(filepath);
    if (!ofs.is_open())
    {
        std::cerr << "Error: Could not open file for saving: " << filepath << std::endl;
        return;
    }

    // Save the medoid index as the first line
    ofs << S.index << "\n";

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

void Vamana::loadGraph(std::vector<Point> &dataset, const std::string &filepath)
{
    std::ifstream ifs(filepath);
    if (!ifs.is_open())
    {
        std::cerr << "Error: Could not open file for loading: " << filepath << std::endl;
        return;
    }

    // Read the first line to get the medoid index
    std::string line;
    if (std::getline(ifs, line))
    {
        S.index = std::stoi(line); // Set the medoid index
    }
    else
    {
        std::cerr << "Error: File is empty or missing medoid index." << std::endl;
        return;
    }

    // Temporary adjacency list to store what we parse from file
    std::vector<std::set<int>> adjacency;

    // Read the rest of the lines in "index:neighbor1,neighbor2,..." format
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
