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

StichedVamana::StichedVamana(const std::vector<Point> &_dataset)
{
    dataset = _dataset;
}

void StichedVamana::initializingEmptyGraph()
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
        P_f[p.C].push_back(p);

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(dataset.size())});

        // update progress bar
        bar.tick();

        i++;
    }

    bar.mark_as_completed();
}

StichedVamanaStatistics StichedVamana::index(float a, int L_small, int R_small, int R_stiched)
{
    sw::Stopwatch stopwatch;
    StichedVamanaStatistics statistics;

    // Initialize G = (V, E) to an empty graph
    // Let F_x ⊆ F be the label-set for every x ∈ P
    // Let P_f ⊆ P be the set of points with label f ∈ F
    initializingEmptyGraph();

    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::red},
        option::PrefixText{"      Stiched Vamana Indexing"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{dataset.size()}};
    int i = 0;

    // foreach f ∈ F do
    for (float f : F)
    {
        // Let G_f ← Vamana(P_f, a, Rsmall , Lsmall )
        G_f[f] = Vamana();
        G_f[f].index(P_f[f], a, L_small, R_small);

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(dataset.size())});

        // update progress bar
        bar.tick();

        i++;
    }

    bar.mark_as_completed();

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
