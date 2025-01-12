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

#include "data.h"
#include "configuration.h"
#include "math.h"
#include "sets.h"
#include "fileio.h"

/*******************/
/* Data Structures */
/*******************/

/**
 * @brief
 * Redirect the output of the functions,
 * in order to have a clear view of the terminal when running the unitests.
 */
class NullStream : public std::ostream {
public:
    NullStream() : std::ostream(nullptr) {}
    NullStream(const NullStream&) = delete;
    NullStream& operator=(const NullStream&) = delete;
};

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


/**********/
/* math.h */
/**********/

/**
 * @brief
 * Test to check the Euclidean Distance Calculation
 */
void test_euclidean_distance()
{
    // Test for 2-dimensional vectors
    std::vector<float> vec1 = {1.0, 2.0};
    std::vector<float> vec2 = {4.0, 6.0};
    float real_distance = 5.0;
    TEST_CHECK(std::abs(euclideanDistance(vec1, vec2) - real_distance) == 0);

    // Test for 3-dimensional vectors
    std::vector<float> vec3 = {1.0, 2.0, 3.0};
    std::vector<float> vec4 = {4.0, 5.0, 6.0};
    real_distance = std::sqrt(27); 
    TEST_CHECK(std::abs(euclideanDistance(vec3, vec4) - real_distance) == 0);

    // Test for identical vectors 
    TEST_CHECK(std::abs(euclideanDistance(vec1, vec1)) == 0);
}



/**
 * @brief
 * Test Finding the Medoid in a Dataset
 */
void test_find_medoid() {

    // Changing the cout to avoid the prints of the find medoid function
    // Save the original std::cout buffer
    std::streambuf* originalCoutBuffer = std::cout.rdbuf();

    // Redirect std::cout to a null stream
    NullStream nullStream;
    std::cout.rdbuf(nullStream.rdbuf());

    // Creating a small dataset
    std::vector<Point> dataset;

    Point P1(0 ,{0.0, 0.0});
    Point P2(1 ,{1.0, 1.0});
    Point P3(2 ,{5.0, 5.0}); 
    Point P4(3 ,{10.0, 10.0});
    Point P5(4 ,{12.0, 12.0});
    dataset.emplace_back(P1);
    dataset.emplace_back(P2);
    dataset.emplace_back(P3);
    dataset.emplace_back(P4);
    dataset.emplace_back(P5);


    Point medoid = findMedoid(dataset);

    // Check if the medoid matches the expected results
    TEST_CHECK((medoid.index == P3.index));

    // Test with a single point 
    std::vector<Point> single_point_dataset;
    single_point_dataset.emplace_back(P1);
    medoid = findMedoid(single_point_dataset);
    TEST_CHECK((medoid.index == P1.index));

    // Restore the original std::cout buffer
    std::cout.rdbuf(originalCoutBuffer);
}



/**********/
/* sets.h */
/**********/

/**
 * @brief 
 * Test for the `getSetDifference` function.
 */
void test_getSetDifference()
{
    // Case 1: Standard difference
    std::set<int> set_a = {1, 2, 3, 4, 5};
    std::set<int> set_b = {3, 4, 6};
    std::set<int> expected = {1, 2, 5};

    std::set<int> result = getSetDifference(set_a, set_b);

    TEST_CHECK(result == expected);

    // Case 2: Completely disjoint sets
    set_a = {1, 2, 3};
    set_b = {4, 5, 6};
    expected = {1, 2, 3};

    result = getSetDifference(set_a, set_b);

    TEST_CHECK(result == expected);

    // Case 3: Identical sets
    set_a = {1, 2, 3};
    set_b = {1, 2, 3};
    expected = {};

    result = getSetDifference(set_a, set_b);

    TEST_CHECK(result == expected);

    // Case 4: Empty set difference
    set_a = {};
    set_b = {1, 2, 3};
    expected = {};

    result = getSetDifference(set_a, set_b);

    TEST_CHECK(result == expected);
}


/**
 * @brief 
 * Test for the `getSetUnion` function.
 */
void test_getSetUnion()
{
    // Case 1: Standard union
    std::set<int> set_a = {1, 2, 3};
    std::set<int> set_b = {3, 4, 5};
    std::set<int> expected = {1, 2, 3, 4, 5};

    std::set<int> result = getSetUnion(set_a, set_b);

    TEST_CHECK(result == expected);

    // Case 2: Completely disjoint sets
    set_a = {1, 2, 3};
    set_b = {4, 5, 6};
    expected = {1, 2, 3, 4, 5, 6};

    result = getSetUnion(set_a, set_b);

    TEST_CHECK(result == expected);

    // Case 3: One empty set
    set_a = {};
    set_b = {1, 2, 3};
    expected = {1, 2, 3};

    result = getSetUnion(set_a, set_b);

    TEST_CHECK(result == expected);
}



/**
 * @brief 
 * Test for the `getIntersectionSize` function.
 */
void test_getIntersectionSize()
{
    // Case 1: Standard intersection
    std::set<int> set_a = {1, 2, 3, 4, 5};
    std::set<int> set_b = {3, 4, 6};
    int expected = 2;

    int result = getIntersectionSize(set_a, set_b);

    TEST_CHECK(result == expected);

    // Case 2: No intersection
    set_a = {1, 2, 3};
    set_b = {4, 5, 6};
    expected = 0;

    result = getIntersectionSize(set_a, set_b);

    TEST_CHECK(result == expected);

    // Case 3: One empty set
    set_a = {};
    set_b = {1, 2, 3};
    expected = 0;

    result = getIntersectionSize(set_a, set_b);

    TEST_CHECK(result == expected);

    // Case 4: Identical sets
    set_a = {1, 2, 3};
    set_b = {1, 2, 3};
    expected = 3;

    result = getIntersectionSize(set_a, set_b);

    TEST_CHECK(result == expected);
}


/************/
/* fileio.h */
/************/

/**
 * @brief 
 * Test reading a file with contents.
 */
void test_readFileContents()
{
    // Create a temporary file with test content
    std::ofstream testFile("test_file.txt");
    testFile << "This is a test file.\n";
    testFile.close();

    // Call the function and verify the output
    std::string content = readFileContents("test_file.txt");
    TEST_CHECK(content == "This is a test file.\n");

    // Clean up the temporary file
    std::remove("test_file.txt");
}


/*********/
/* TESTS */
/*********/

TEST_LIST = {
    {"conf.h            | Initialization      ", test_configuration_initialization},
    {"conf.h            | Non-Existent File   ", test_configuration_nonexistent_file},
    {"math.h            | Euclidean Distance  ", test_euclidean_distance},
    {"math.h            | Find Medoid         ", test_find_medoid},
    {"sets.h            | Set Difference      ", test_getSetDifference},
    {"sets.h            | Set Union           ", test_getSetUnion},
    {"sets.h            | Set Intersection    ", test_getIntersectionSize},
    {"fileio.h          | Read file           ", test_readFileContents},
    {NULL, NULL}};