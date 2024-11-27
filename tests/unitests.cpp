#include <fstream>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <sys/stat.h>

#define RYML_SINGLE_HDR_DEFINE_NOW // https://github.com/biojppm/rapidyaml
#include "rapidyaml.h"

#include "acutest.h"

#include "conf.h"
#include "misc.h"

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

// run tests
TEST_LIST = {
    {"conf.h - Initialization", test_configuration_initialization},
    {"conf.h - Non-Existent File", test_configuration_nonexistent_file},
    {"misc.h - Setup Logging", test_setup_logging},
    {"misc.h - Debug Function", test_debug_function},
    {"misc.h - Read File Contents", test_read_file_contents},
    {"misc.h - Validate File Exists", test_validate_file_exists},
    {NULL, NULL} // {NULL, NULL} is marking the end of the list
};