/**********************/
/* Standard Libraries */
/**********************/
#include <vector>
#include <limits>
#include <cmath>

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

    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::cyan},
        option::PrefixText{"               Finding Medoid"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{dataset.size()}};

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

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(dataset.size())});

        // update progress bar
        bar.tick();
    }

    bar.mark_as_completed();

    return dataset[_medoid_idx];
}

float euclideanDistance(const std::vector<float> &a, const std::vector<float> &b)
{
    float sum = 0.0f;

    for (std::size_t i = 0; i < a.size(); ++i)
        sum += std::pow(a[i] - b[i], 2);

    return std::sqrt(sum);
}