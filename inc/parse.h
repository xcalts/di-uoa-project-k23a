#ifndef PARSE_H
#define PARSE_H

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
 * Parse a `.fvecs` points file.
 * @param fvecs_filepath
 * The path to the `.fvecs` points file.
 * @return std::vector<Point>
 * A vector containings the parsed points.
 */
std::vector<Point> parsePointsFvecsFile(std::string &fvecs_filepath);

/**
 * @brief
 * Parse a `.fvecs` queries file.
 * @param fvecs_filepath
 * The path to the `.fvecs` queries file.
 * @return std::vector<Query>
 * A vector containings the parsed queries.
 */
std::vector<Query> parseQueriesFvecsFile(std::string &fvecs_filepath);

/**
 * @brief
 * Parse a `.ivecs` dataset file.
 * @param ivecs_filepath
 * The path to the `.ivecs` dataset file.
 * @return std::vector<std::vector<int>>
 * A vector containing the vector with the true nearest neighbors for each and every point in the dataset.
 */
std::vector<Groundtruth> parseIvecsFile(std::string &ivecs_filepath);

/**
 * @brief
 * Parse the sigmod-contest dummy data.
 * @param filepath
 * The path to the file.
 * @param no_dimensions
 * The dimensions of each point's vector.
 * @return std::vector<F_Point>
 * The dataset of points.
 */
std::vector<Point> parseDummyData(std::string &filepath, int no_dimensions);

/**
 * @brief
 * Parse the sigmod-contest dummy queries.
 * @param filepath
 * The path to the file.
 * @param no_dimensions
 * The dimensions of each query's vector.
 * @return std::vector<F_Query>
 * The dataset of queries.
 */
std::vector<Query> parseDummyQueries(std::string &filepath, int no_dimensions);

#endif // PARSE_H