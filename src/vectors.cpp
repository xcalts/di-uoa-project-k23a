/**********************/
/* Standard Libraries */
/**********************/
#include <random>

/**********************/
/* External Libraries */
/**********************/
#include <indicators/block_progress_bar.hpp>

/**********************/
/* Project Components */
/**********************/
#include "vectors.h"

/**************/
/* Namespaces */
/**************/
using namespace indicators;

std::vector<int> generateSigma(int n)
{
    std::vector<int> sigma(n);

    BlockProgressBar bar{
        option::BarWidth{80},
        option::ForegroundColor{Color::green},
        option::PrefixText{"      Generating Sigma Vector"},
        option::FontStyles{
            std::vector<FontStyle>{FontStyle::bold}},
        option::MaxProgress{n}};

    for (int i = 0; i < n; ++i)
    {
        sigma[i] = i;

        // Show iteration as postfix text
        bar.set_option(option::PostfixText{
            std::to_string(i + 1) + "/" + std::to_string(n)});

        // update progress bar
        bar.tick();
    }

    bar.mark_as_completed();

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(sigma.begin(), sigma.end(), g);

    return sigma;
}