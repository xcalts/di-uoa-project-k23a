/**********************/
/* Standard Libraries */
/**********************/
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>
#include <cstdint>

/**********************/
/* External Libraries */
/**********************/
#include <indicators/block_progress_bar.hpp>

/**********************/
/* Project Components */
/**********************/
#include "math.h"
#include "brute.h"

/**************/
/* Namespaces */
/**************/
using namespace indicators;

Brute::Brute(std::vector<Point> &_dataset)
{
    dataset = _dataset;
}

std::set<int> Brute::getTrueNearestNeighborsAsSet(int query_index, int k)
{
    std::set<int> neighborsSet;

    auto it = query_gnn.find(query_index);
    if (it != query_gnn.end())
    {
        const std::vector<int> &gt_neighbors = it->second;
        for (int i = 0; i < k && i < static_cast<int>(gt_neighbors.size()); ++i)
        {
            neighborsSet.insert(gt_neighbors[i]);
        }
    }

    return neighborsSet;
}

std::vector<int> Brute::getTrueNearestNeighbors(int query_index, int k)
{
    std::vector<int> neighbors;
    auto it = query_gnn.find(query_index);
    if (it != query_gnn.end())
    {
        const std::vector<int> &gt_neighbors = it->second;
        for (int i = 0; i < k && i < static_cast<int>(gt_neighbors.size()); ++i)
        {
            neighbors.push_back(gt_neighbors[i]);
        }
    }
    return neighbors;
}

std::vector<int> Brute::bruteForceNearestNeighbors(const Query &q, int k)
{
    // Vector to store pairs of distance and index
    std::vector<std::pair<float, int>> distances;

    if (q.query_type == 0)
    {
        distances.reserve(dataset.size());
        for (const auto &p : dataset)
        {
            float dist = euclideanDistance(q.vec, p.vec);
            distances.push_back(std::make_pair(dist, p.index));
        }
    }
    else if (q.query_type == 1)
    {
        distances.reserve(dataset.size());
        for (const auto &p : dataset)
            if (q.v == p.C)
            {
                float dist = euclideanDistance(q.vec, p.vec);
                distances.push_back(std::make_pair(dist, p.index));
            }
    }
    else
    {
        std::vector<int> x;
        x.resize(k, 0);

        return x;
    }

    // Partially sort the distances so that the first k elements are the smallest
    std::nth_element(
        distances.begin(),
        distances.begin() + k,
        distances.end(),
        [](const std::pair<float, int> &a, const std::pair<float, int> &b)
        {
            return a.first < b.first;
        });

    // Now sort the first k elements to ensure they are in ascending order
    std::sort(
        distances.begin(),
        distances.begin() + k,
        [](const std::pair<float, int> &a, const std::pair<float, int> &b)
        {
            return a.first < b.first;
        });

    // Extract the indices of the k nearest neighbors in order
    std::vector<int> neighbors;
    neighbors.reserve(k);
    for (int i = 0; i < k; ++i)
    {
        neighbors.push_back(distances[i].second);
    }

    // free the memory
    distances.clear();

    return neighbors;
}

std::set<int> Brute::bruteForceNNs(const Query &q, int k)
{
    std::vector<std::pair<float, int>> distances;

    distances.reserve(dataset.size());
    for (const auto &p : dataset)
    {
        float dist = euclideanDistance(q.vec, p.vec);
        distances.push_back(std::make_pair(dist, p.index));
    }

    std::nth_element(
        distances.begin(),
        distances.begin() + k,
        distances.end(),
        [](const std::pair<float, int> &a, const std::pair<float, int> &b)
        {
            return a.first < b.first;
        });

    // Extract the indices of the k nearest neighbors in order
    std::set<int> neighbors;
    for (int i = 0; i < k; ++i)
    {
        neighbors.insert(distances[i].second);
    }

    // free the memory
    distances.clear();

    return neighbors;
}

void Brute::calculateDummyGroundTruth(const std::vector<Query> &dummyQueries, int k)
{
    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::yellow},
        option::PrefixText{"Calculating the kNN by Brute Force"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{dataset.size()}};

    for (const Query &q : dummyQueries)
    {
        std::vector<int> kNNs = bruteForceNearestNeighbors(q, k);
        query_gnn[q.index] = kNNs;

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(q.index + 1) + "/" + std::to_string(dataset.size())});

        // update progress bar
        bar.tick();
    }

    bar.mark_as_completed();
}

void Brute::save(const std::string &filename)
{
    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }

    for (const auto &pair : query_gnn)
    {
        outFile << pair.first << ":";
        for (size_t i = 0; i < pair.second.size(); ++i)
        {
            outFile << pair.second[i];
            if (i < pair.second.size() - 1)
            {
                outFile << ",";
            }
        }
        outFile << "\n";
    }

    outFile.close();
}

void Brute::load(const std::string &filename)
{
    std::ifstream inFile(filename);
    if (!inFile.is_open())
    {
        throw std::runtime_error("Could not open file for reading: " + filename);
    }

    std::string line;
    while (std::getline(inFile, line))
    {
        std::istringstream iss(line);
        std::string key, values;
        if (std::getline(iss, key, ':') && std::getline(iss, values))
        {
            int keyInt = std::stoi(key);
            std::vector<int> vec;
            std::istringstream valStream(values);
            std::string val;
            while (std::getline(valStream, val, ','))
            {
                if (!val.empty())
                {
                    vec.push_back(std::stoi(val));
                }
            }
            query_gnn[keyInt] = vec;
        }
    }

    inFile.close();
}