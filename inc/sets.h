#ifndef SET_H
#define SET_H

/**********************/
/* Standard Libraries */
/**********************/
#include <set>

/**
 * @brief
 * Get the set `a` - `b`
 * @param set_a
 * The set `a`.
 * @param set_b
 * The set `b`.
 * @return std::set<int>
 */
std::set<int> getSetDifference(const std::set<int> &set_a, const std::set<int> &set_b);

/**
 * @brief
 * Get the set `a` ∪ `b`
 * @param set_a
 * The set `a`.
 * @param set_b
 * The set `b`.
 * @return std::set<int>
 */
std::set<int> getSetUnion(const std::set<int> &set_a, const std::set<int> &set_b);

/**
 * @brief
 * Computes the size of the intersection of two integer sets.
 *
 * This function takes two sets of integers and calculates the number of elements
 * that are present in both sets.
 *
 * @param a The first set of integers.
 * @param b The second set of integers.
 * @return The number of elements that are present in both sets.
 */
int getIntersectionSize(const std::set<int> &a, const std::set<int> &b);

#endif // SET_H