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
// #define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml.h"

// https://github.com/mity/acutest
#include "acutest.h"

/**********************/
/* Project Components */
/**********************/

#include "configuration.h"

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
    config_file << "k: 10\n";
    config_file << "a: 0.5\n";
    config_file << "L: 100\n";
    config_file << "R: 10\n";
    config_file << "dummy_data_filepath: \"dummy_data.bin\"\n";
    config_file << "dummy_queries_filepath: \"dummy_queries_filepath.bin\"\n";
    config_file << "groundtruth_nn_filepath: \"knn.bin\"\n";
    config_file << "data_dimensions: 10\n";
    config_file << "queries_dimensions: 10\n";
    config_file << "tau: 5\n";
    config_file << "L_small: 150\n";
    config_file << "R_small: 30\n";
    config_file << "R_stiched: 40\n";
    config_file.close();

    // Initialize the Configuration object
    Configuration config = Configuration();
    config.initialize("test_config.yaml");

    // Check if the values are correctly initialized
    TEST_CHECK(config.dataset_filepath == "dataset.fvecs");
    TEST_CHECK(config.queries_filepath == "queries.fvecs");
    TEST_CHECK(config.evaluation_filepath == "evaluation.ivecs");
    TEST_CHECK(config.k == 10);
    TEST_CHECK(config.a == 0.5f);
    TEST_CHECK(config.L == 100);
    TEST_CHECK(config.R == 10);

    // Remove the temporary configuration file
    std::remove("test_config.yaml");
}

/**
 * @brief
 * Test to check if the Configuration class throws an exception for a non-existent file.
 */
void test_configuration_nonexistent_file()
{
    Configuration config = Configuration();

    // Check if initializing with a non-existent file throws an exception
    TEST_EXCEPTION(config.initialize("najsfdjalsjfdlasdjf"), std::runtime_error);
}

/*********/
/* TESTS */
/*********/

TEST_LIST = {
    {"conf.h            | Initialization      ", test_configuration_initialization},
    {"conf.h            | Non-Existent File   ", test_configuration_nonexistent_file},
    {NULL, NULL}};