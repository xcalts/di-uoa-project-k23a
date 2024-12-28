#ifndef MATH_H
#define MATH_H

/**********************/
/* Standard Libraries */
/**********************/
#include <vector>

/**********************/
/* Project Components */
/**********************/
#include "data.h"

/**
 * @brief
 * Calculates the medoid of the dataset.
 * The medoid of the dataset is similar in concept to mean or centroid, but the medoid is always restricted to be a member of the data set.
 * This function calculates the total distance from each point to all other points and then select the point with the smallest total distance as the medoid.
 * @param dataset
 * The dataset that you want to calculate the medoid for.
 * @return Point &
 * The `Point` that is the medoid of the dataset.
 */
int findMedoid(const std::vector<Point> &dataset);

/**
 * @brief
 * Calculate the euclidean distance between two points `a` and `b`.
 * @param a
 * The first `Point`.
 * @param b
 * The second `Point`.
 * @return float
 * The euclidean distance between the two points.
 */
float euclideanDistance(const std::vector<float> &a, const std::vector<float> &b);

#endif // MATH_H