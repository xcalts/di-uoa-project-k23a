/**********************/
/* Standard Libraries */
/**********************/
#include <algorithm>
#include <set>

/**********************/
/* Project Components */
/**********************/
#include "sets.h"

std::set<int> getSetDifference(const std::set<int> &set_a, const std::set<int> &set_b)
{
    std::set<int> result;

    std::set_difference(
        set_a.begin(), set_a.end(),
        set_b.begin(), set_b.end(),
        std::inserter(result, result.begin()));

    return result;
}

std::set<int> getSetUnion(const std::set<int> &set_a, const std::set<int> &set_b)
{
    std::set<int> result;

    std::set_union(
        set_a.begin(), set_a.end(),
        set_b.begin(), set_b.end(),
        std::inserter(result, result.begin()));

    return result;
}

int getIntersectionSize(const std::set<int> &set_a, const std::set<int> &set_b)
{
    std::set<int> result;
    std::set_intersection(

        set_a.begin(), set_a.end(),
        set_b.begin(), set_b.end(),
        std::inserter(result, result.begin())

    );

    // Compute the intersection of sets X and G, and insert the result into 'result'
    std::set_intersection(set_a.begin(), set_a.end(), set_b.begin(), set_b.end(), std::inserter(result, result.begin()));

    return result.size();
}