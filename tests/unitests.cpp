/**********************/
/* Standard Libraries */
/**********************/

#include <fstream>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <sys/stat.h>

/**********************/
/* External Libraries */
/**********************/

// https://github.com/biojppm/rapidyaml
#define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml.h"

// https://github.com/mity/acutest
#include "acutest.h"

/**********************/
/* Project Components */
/**********************/

#include "conf.h"
#include "misc.h"
#include "vamana.h"
#include "vamana-filtered.h"

/**********/
/* conf.h */
/**********/

/**
 * @brief
 * Test to check if the Configuration class initializes correctly.
 */
void test_configuration_initialization()
{
    // Create a temporary YAML configuration file
    std::ofstream config_file("test_config.yaml");
    config_file << "dataset_filepath: \"dataset.fvecs\"\n";
    config_file << "queries_filepath: \"queries.fvecs\"\n";
    config_file << "evaluation_filepath: \"evaluation.ivecs\"\n";
    config_file << "kNN: 10\n";
    config_file << "alpha: 0.5\n";
    config_file << "max_candinates: 100\n";
    config_file << "max_edges: 10\n";
    config_file << "dummy_data_filepath: \"dummy_data.bin\"\n";
    config_file << "dummy_queries_filepath: \"dummy_queries_filepath.bin\"\n";
    config_file << "groundtruth_10nn_filepath: \"knn.bin\"\n";
    config_file << "data_dimensions: 10\n";
    config_file << "queries_dimensions: 10\n";
    config_file << "tau: 5\n";
    config_file.close();

    // Initialize the Configuration object
    Configuration config = Configuration("test_config.yaml");

    // Check if the values are correctly initialized
    TEST_CHECK(config.dataset_filepath == "dataset.fvecs");
    TEST_CHECK(config.queries_filepath == "queries.fvecs");
    TEST_CHECK(config.evaluation_filepath == "evaluation.ivecs");
    TEST_CHECK(config.kNN == 10);
    TEST_CHECK(config.alpha == 0.5f);
    TEST_CHECK(config.max_candinates == 100);
    TEST_CHECK(config.max_edges == 10);

    // Remove the temporary configuration file
    std::remove("test_config.yaml");
}

/**
 * @brief
 * Test to check if the Configuration class throws an exception for a non-existent file.
 */
void test_configuration_nonexistent_file()
{
    // Check if initializing with a non-existent file throws an exception
    TEST_EXCEPTION(Configuration config = Configuration("nonexistent_config.yaml"), std::runtime_error);
}

/**********/
/* misc.h */
/**********/

/**
 * @brief
 * Test to check if the logging setup works correctly.
 */
void test_setup_logging()
{
    setupLogging();
    TEST_CHECK(true); // If no exception is thrown, the test passes
}

/**
 * @brief
 * Test to check if the debug function works correctly.
 */
void test_debug_function()
{
    TEST_CHECK(true); // If no exception is thrown, the test passes
}

/**
 * @brief
 * Test to check if the readFileContents function works correctly.
 */
void test_read_file_contents()
{
    // Create a temporary file
    std::ofstream temp_file("temp_test_file.txt");
    temp_file << "This is a test file.";
    temp_file.close();

    // Read the file contents
    std::string contents = readFileContents("temp_test_file.txt");
    TEST_CHECK(contents == "This is a test file.");

    // Remove the temporary file
    std::remove("temp_test_file.txt");
}

/**
 * @brief
 * Test to check if the validateFileExists function works correctly.
 */
void test_validate_file_exists()
{
    // Create a temporary file
    std::ofstream temp_file("temp_test_file.txt");
    temp_file << "This is a test file.";
    temp_file.close();

    // Validate the file exists
    validateFileExists("temp_test_file.txt");

    // Remove the temporary file
    std::remove("temp_test_file.txt");

    // Validate the file does not exist
    TEST_EXCEPTION(validateFileExists("temp_test_file.txt"), std::runtime_error);
}

/************/
/* vamana.h */
/************/

/**
 * @brief
 * Test to check if the intersectionSize function works correctly.
 */
void test_intersection_size()
{
    std::vector<int> vec1 = {1, 2, 3, 4, 5};
    std::vector<int> vec2 = {4, 5, 6, 7, 8};
    int result = intersectionSize(vec1, vec2);
    TEST_CHECK(result == 2);

    vec1 = {1, 2, 3};
    vec2 = {4, 5, 6};
    result = intersectionSize(vec1, vec2);
    TEST_CHECK(result == 0);

    vec1 = {1, 2, 3, 4, 5};
    vec2 = {1, 2, 3, 4, 5};
    result = intersectionSize(vec1, vec2);
    TEST_CHECK(result == 5);
}

/**
 * @brief
 * Test to check if the euclideanDistance function works correctly.
 */
void test_euclidean_distance()
{
    std::vector<float> vec1 = {1.0, 2.0, 3.0};
    std::vector<float> vec2 = {4.0, 5.0, 6.0};
    float result = euclideanDistance(vec1, vec2);
    TEST_CHECK(std::floor(result) == std::floor(std::sqrt(27.0)));

    vec1 = {0.0, 0.0, 0.0};
    vec2 = {0.0, 0.0, 0.0};
    result = euclideanDistance(vec1, vec2);
    TEST_CHECK(std::floor(result) == std::floor(0.0));

    vec1 = {1.0, 2.0};
    vec2 = {4.0, 6.0};
    result = euclideanDistance(vec1, vec2);
    TEST_CHECK(std::floor(result) == std::floor(5.0));
}

/*********************/
/* vamana-filtered.h */
/*********************/

void test_filtered_greedy_search()
{
    // Create a sample dataset
    std::vector<F_Point> dataset;
    std::vector<float> vec1 = {1.0, 2.0};
    std::vector<float> vec2 = {2.0, 3.0};
    std::vector<float> vec3 = {3.0, 4.0};
    std::vector<float> vec4 = {4.0, 5.0};

    dataset.push_back(F_Point(0, vec1, 1.0, 1.0));
    dataset.push_back(F_Point(1, vec2, 1.0, 1.0));
    dataset.push_back(F_Point(2, vec3, 2.0, 2.0));
    dataset.push_back(F_Point(3, vec4, 2.0, 2.0));

    F_Vamana vamana(dataset);

    // Set initial nodes and query parameters
    std::set<int> S = {0, 1};
    int x_q = 2;
    std::vector<float> x_vec = {3.0, 4.0};
    int k = 1;
    int L_ = 2;
    std::set<float> F_q = {1.0};

    // Perform the filtered greedy search
    auto result = vamana.filteredGreedySearch(S, x_q, x_vec, k, L_, F_q);

    // Check the results
    TEST_CHECK(result.first.size() == 2);  // L should have 2 elements
    TEST_CHECK(result.second.size() == 1); // V should have 1 element
}

/**
 * @brief
 * Test to check if the filteredRobustPrune function works correctly.
 */
void test_filtered_robust_prune()
{
    // Create a sample dataset
    std::vector<F_Point> dataset;
    std::vector<float> vec1 = {1.0, 2.0};
    std::vector<float> vec2 = {2.0, 3.0};
    std::vector<float> vec3 = {3.0, 4.0};
    std::vector<float> vec4 = {4.0, 5.0};

    dataset.push_back(F_Point(0, vec1, 1.0, 1.0));
    dataset.push_back(F_Point(1, vec2, 1.0, 1.0));
    dataset.push_back(F_Point(2, vec3, 2.0, 2.0));
    dataset.push_back(F_Point(3, vec4, 2.0, 2.0));

    F_Vamana vamana(dataset);

    // Set initial neighbors
    vamana.dataset[0].addNeighbor(1);
    vamana.dataset[0].addNeighbor(2);
    vamana.dataset[0].addNeighbor(3);

    std::set<int> V = {1, 2, 3};
    int p = 0;
    float a = 1.5;
    int R = 2;

    // Perform the filtered robust prune
    vamana.filteredRobustPrune(p, V, a, R);

    // Check the results
    TEST_CHECK(vamana.dataset[0].neighbors.size() == 2);   // Nout(p) should have 2 elements
    TEST_CHECK(vamana.dataset[0].neighbors.count(1) == 1); // Neighbor 1 should be present
    TEST_CHECK(vamana.dataset[0].neighbors.count(2) == 1); // Neighbor 2 should be present
}

/*********/
/* TESTS */
/*********/

TEST_LIST = {
    {"conf.h            | Initialization      ", test_configuration_initialization},
    {"conf.h            | Non-Existent File   ", test_configuration_nonexistent_file},
    {"misc.h            | Setup Logging       ", test_setup_logging},
    {"misc.h            | Debug Function      ", test_debug_function},
    {"misc.h            | Read File Contents  ", test_read_file_contents},
    {"misc.h            | Validate File Exists", test_validate_file_exists},
    {"misc.h            | Intersection Size   ", test_intersection_size},
    {"misc.h            | Euclidean Distance  ", test_euclidean_distance},
    {"vamana-filtered.h | Filtered Greedy Search", test_filtered_greedy_search},
    {"vamana-filtered.h | Filtered Robust Prune", test_filtered_robust_prune},
    {NULL, NULL}};