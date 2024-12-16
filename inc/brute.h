#ifndef BRUTE_H
#define BRUTE_H

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
#include <algorithm> // Added for std::sort and std::nth_element

/**********************/
/* External Libraries */
/**********************/

#include "progressbar.h"

/**********************/
/* Project Components */
/**********************/

#include "misc.h"
#include "vamana-filtered.h"

/***************/
/* Definitions */
/***************/

class Brute;

/*******************/
/* Implementations */
/*******************/

/**
 * @brief
 * Implementation of the brute force algorithm for kNN search.
 */
/**
 * @brief
 * Implementation of the brute force algorithm for kNN search.
 */
class Brute
{
public:
    /**
     * @brief
     * It holds the kNN for each and every one of the query points.
     */
    std::map<int, std::vector<int>> ground_truth;

    /**
     * @brief
     * It contains all the points of the dataset.
     */
    std::vector<F_Point> dataset;

    /**
     * @brief
     * Construct the brute force algorithm.
     * @param _dataset
     * It contains all the points of the dataset.
     */
    Brute(std::vector<F_Point> &_dataset)
    {
        dataset = _dataset;
    }

    /**
     * @brief
     * Get the k nearest neighbors of a given query index from the ground truth.
     * @param query_index
     * The index of the query.
     * @param k
     * The number of nearest neighbors to return.
     * @return std::vector<int>
     * The indices of the `k` nearest neighbors.
     */
    std::vector<int> getGtNNs(int query_index, int k)
    {
        std::vector<int> neighbors;
        auto it = ground_truth.find(query_index);
        if (it != ground_truth.end())
        {
            const std::vector<int> &gt_neighbors = it->second;
            for (int i = 0; i < k && i < static_cast<int>(gt_neighbors.size()); ++i)
            {
                neighbors.push_back(gt_neighbors[i]);
            }
        }
        return neighbors;
    }

    /**
     * @brief
     * Search for kNN using brute force.
     * @param q
     * The query vector to search kNN for.
     * @param k
     * The number of nearest neighbors to search.
     * @return std::vector<int>
     * The indices of the `k` nearest neighbors.
     */
    std::vector<int> bruteForceNearestNeighbors(F_Query &q, int k)
    {
        // Vector to store pairs of distance and index
        std::vector<std::pair<float, int>> distances;

        if (q.query_type == 0)
        {
            distances.reserve(dataset.size());
            for (auto p : dataset)
            {
                float dist = euclideanDistance(q.vec, p.vec);
                distances.emplace_back(dist, p.index);
            }
        }
        else if (q.query_type == 1)
        {
            for (auto p : dataset)
                if (q.v == p.C)
                {
                    float dist = euclideanDistance(q.vec, p.vec);
                    distances.emplace_back(dist, p.index);
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

        return neighbors;
    }

    /**
     * @brief
     * Calculate the ground truth kNNs for the dummy queries.
     * @param dummyQueries
     * The dummy queries.
     * @param kNN
     * The number k of nearest neighbors.
     */
    void calculateDummyGroundTruth(std::vector<F_Query> &dummyQueries, int kNN)
    {
        progressbar bar(dummyQueries.size());
        for (F_Query &q : dummyQueries)
        {
            bar.update();
            std::vector<int> kNNs = bruteForceNearestNeighbors(q, kNN);
            ground_truth[q.index] = kNNs;
        }
    }

    /**
     * @brief
     * Save the ground truth kNNs to a file.
     * @param filename
     * The filename to save the ground truth kNNs to.
     */
    void save(const std::string &filename)
    {
        std::ofstream outFile(filename);
        if (!outFile.is_open())
        {
            throw std::runtime_error("Could not open file for writing: " + filename);
        }

        for (const auto &pair : ground_truth)
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

    /**
     * @brief
     * Load the ground truth kNNs from a file.
     * @param filename
     * The filename to load the ground truth kNNs from.
     */
    void load(const std::string &filename)
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
                ground_truth[keyInt] = vec;
            }
        }

        inFile.close();
    }
};

#endif // BRUTE_H
