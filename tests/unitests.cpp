#include <fstream>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <sys/stat.h>

// https://github.com/biojppm/rapidyaml
#define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml.h"

// https://github.com/mity/acutest
#include "acutest.h"

#include "conf.h"
#include "misc.h"
#include "vamana.h"

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
    config_file << "data_dimensions: 10\n";
    config_file << "queries_dimensions: 10\n";
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
    spdlog::info("This is a test log message.");
    TEST_CHECK(true); // If no exception is thrown, the test passes
}

/**
 * @brief
 * Test to check if the debug function works correctly.
 */
void test_debug_function()
{
    debug("test_debug_function", "This is a test debug message.");
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

/**********/
/* vamana.h */
/**********/

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

// run tests
TEST_LIST = {
    {"conf.h - Initialization", test_configuration_initialization},
    {"conf.h - Non-Existent File", test_configuration_nonexistent_file},
    {"misc.h - Setup Logging", test_setup_logging},
    {"misc.h - Debug Function", test_debug_function},
    {"misc.h - Read File Contents", test_read_file_contents},
    {"misc.h - Validate File Exists", test_validate_file_exists},
    {"vamana.h - Intersection Size", test_intersection_size},
    {"vamana.h - Euclidean Distance", test_euclidean_distance},
    {NULL, NULL} // {NULL, NULL} is marking the end of the list
};