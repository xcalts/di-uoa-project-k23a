#ifndef MISC_H
#define MISC_H

/**********************/
/* Standard Libraries */
/**********************/

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <set>
#include <unordered_set>

/**********************/
/* External Libraries */
/**********************/

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

/***************/
/* Definitions */
/***************/

void setupLogging();
void debug(const std::string &, const std::string &);
std::string readFileContents(const std::string &);
void validateFileExists(const std::string &);
std::string toString(std::vector<int>);
int intersectionSize(std::vector<int> &, std::vector<int> &);
float euclideanDistance(const std::vector<float> &, const std::vector<float> &);
std::vector<int> generateSigma(int);
std::set<int> getSetDifference(std::set<int> &, std::set<int> &);
int getSetItemAtIndex(int, std::set<int> &);

/*******************/
/* Implementations */
/*******************/

/**
 * @brief
 * Setup file and console logging.
 */
void setupLogging()
{
    // Setup two sinks: console & file.
    std::vector<spdlog::sink_ptr> sinks{
        std::make_shared<spdlog::sinks::stdout_color_sink_mt>(),
        std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs.txt"),
    };

    // Create a logger to use those two sinks.
    auto spd_logger = std::make_shared<spdlog::logger>("combined_logger", begin(sinks), end(sinks));
    spdlog::register_logger(spd_logger);

    // Set it up as the default logger.
    spdlog::set_default_logger(spd_logger);

    // Change the 'printing' pattern.
    // spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%l]%$ %v");
    spdlog::set_pattern("%v");
}

/**
 * @brief
 * If the `-DDEBUG` flag is passed to the compiler, then this function will output debug messages.
 * @param location
 * The location in the code where the debug message is being printed.
 * @param message
 * The message to print.
 */
void debug(const std::string &location, const std::string &message)
{
#ifdef DEBUG
    std::cout << location << " - " << message << std::endl;
#endif
}

/**
 * @brief
 * This function reads the contents of the file at `filePath`.
 * Note that the function does not perform validation checks.
 *
 * @param filePath
 * The filepath to the file that will be parsed.
 *
 * @return std::string
 */
std::string readFileContents(const std::string &filePath)
{
    std::ifstream file(filePath);
    std::stringstream buffer;

    buffer << file.rdbuf();

    return buffer.str();
}

/**
 * @brief
 * This function validates whether the file at the provided `filepath` exists or not.
 * If it does not exist, then it raises a `std::runtime_error`.
 * @param filepath
 * The filepath to the file in question.
 */
void validateFileExists(const std::string &filepath)
{
    struct stat buffer;
    int fileExists = (stat(filepath.c_str(), &buffer) == 0);

    if (fileExists == 0)
        throw std::runtime_error("There is no existing file at: " + filepath);
}

/**
 * @brief
 * Prints the vector data in a table format.
 * @param vec
 * The vector data.
 * @return std::string
 */
std::string toString(std::vector<int> vec)
{
    std::ostringstream oss;
    oss << "[ ";
    for (size_t i = 0; i < vec.size(); ++i)
    {
        oss << vec[i];
        if (i < vec.size() - 1)
        {
            oss << ", ";
        }
    }
    oss << " ]";

    return oss.str();
}

/**
 * @brief
 * Computes the size of the intersection of two integer vectors.
 *
 * This function takes two vectors of integers, converts them to unordered sets
 * to remove duplicates, and then calculates the number of elements that are
 * present in both sets.
 *
 * @param a The first vector of integers.
 * @param b The second vector of integers.
 * @return The number of elements that are present in both vectors.
 */
int intersectionSize(std::vector<int> &a, std::vector<int> &b)
{
    // Convert vectors to unordered sets to remove duplicates
    std::unordered_set<int> set_a(a.begin(), a.end());
    std::unordered_set<int> set_b(b.begin(), b.end());

    int count = 0;

    // Iterate through the smaller set for efficiency
    if (set_a.size() > set_b.size())
        std::swap(set_a, set_b);

    for (int e : set_a)
        if (set_b.find(e) != set_b.end())
            count++;

    return count;
}

/**
 * @brief
 * Calculate the euclidean distance between two vectors `a` and `b`.
 * @param a
 * The first `vector`.
 * @param b
 * The second `vector`.
 * @return float
 */
float euclideanDistance(const std::vector<float> &a, const std::vector<float> &b)
{
    float sum = 0.0f;

    for (size_t i = 0; i < a.size(); ++i)
        sum += std::pow(a[i] - b[i], 2);

    return std::sqrt(sum);
}

/**
 * @brief
 * Generating a vector that contains all numbers from 1..n shuffled.
 * @param n
 * The size of the vector.
 * @return
 * std::vector<int>
 */
std::vector<int> generateSigma(int n)
{
    //
    std::vector<int> sigma(n);
    for (int i = 0; i < n; ++i)
        sigma[i] = i;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(sigma.begin(), sigma.end(), g);

    return sigma;
}

/**
 * @brief
 * Get the set `a` - `b`
 * @param set_a
 * The set `a`.
 * @param set_b
 * The set `b`.
 * @return std::set<int>
 */
std::set<int> getSetDifference(std::set<int> &set_a, std::set<int> &set_b)
{
    std::set<int> result;

    std::set_difference(
        set_a.begin(), set_a.end(),
        set_b.begin(), set_b.end(),
        std::inserter(result, result.begin()));

    return result;
}

/**
 * @brief
 * Get the set's item at index `index`.
 * @param index
 * The index of the item.
 * @param set
 * The set to get the item from.
 * @return int
 */
int getSetItemAtIndex(int index, std::set<int> &set)
{
    if (index >= set.size())
        throw std::invalid_argument("Index is not valid");

    auto it = set.begin();

    std::advance(it, index);

    return *it;
}

#endif // MISC_H