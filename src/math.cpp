/**********************/
/* Standard Libraries */
/**********************/
#include <vector>
#include <limits>
#include <cmath>
#include <atomic>
#include <thread>
#include <mutex>

/**********************/
/* External Libraries */
/**********************/
#include <indicators/block_progress_bar.hpp>

/**********************/
/* Project Components */
/**********************/
#include "data.h"
#include "math.h"

/**************/
/* Namespaces */
/**************/
using namespace indicators;

Point findMedoid(std::vector<Point> &dataset)
{
    float minimum_dist = std::numeric_limits<float>::max();
    int _medoid_idx = -1;

    std::mutex mtx; // To protect shared resources
    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::cyan},
        option::PrefixText{"               Finding Medoid"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{dataset.size()}};

    // Total number of threads
    const int num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    // Shared minimum distance and corresponding medoid index
    std::atomic<float> global_minimum_dist{std::numeric_limits<float>::max()};
    std::atomic<int> global_medoid_idx{-1};

    // Function for each thread to process a range of indices
    auto worker = [&](int start, int end)
    {
        for (int i = start; i < end; ++i)
        {
            float total_distance = 0.0f;

            for (int j = 0; j < dataset.size(); ++j)
            {
                if (i != j)
                {
                    float distance = euclideanDistance(dataset[i].vec, dataset[j].vec);
                    total_distance += distance;
                }
            }

            // Update global minimum distance in a thread-safe manner
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (total_distance < global_minimum_dist)
                {
                    global_minimum_dist = total_distance;
                    global_medoid_idx = i;
                }
            }

            // Update progress bar (also thread-safe)
            {
                std::lock_guard<std::mutex> lock(mtx);
                bar.set_option(option::PostfixText{
                    std::to_string(i + 1) + "/" + std::to_string(dataset.size())});
                bar.tick();
            }
        }
    };

    // Divide work among threads
    int chunk_size = dataset.size() / num_threads;
    for (int t = 0; t < num_threads; ++t)
    {
        int start = t * chunk_size;
        int end = (t == num_threads - 1) ? dataset.size() : start + chunk_size;
        threads.emplace_back(worker, start, end);
    }

    // Wait for all threads to complete
    for (auto &t : threads)
    {
        t.join();
    }

    bar.mark_as_completed();

    // Return the medoid
    return dataset[global_medoid_idx];
}

float euclideanDistance(const std::vector<float> &a, const std::vector<float> &b)
{
    float sum = 0.0f;

    for (std::size_t i = 0; i < a.size(); ++i)
        sum += std::pow(a[i] - b[i], 2);

    return std::sqrt(sum);
}