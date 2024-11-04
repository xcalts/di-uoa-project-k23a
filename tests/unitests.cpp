#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "acutest.h"
#include "vamana.h"

bool verbose_enabled;

// Tests functions on small problems with easily calculated solutions

// test to check if the edge class save information correctly
void test_edge_initialization()
{
    Edge e(5, 10);
    TEST_CHECK(e.to_index == 5);
    TEST_CHECK(e.weight == 10);
}

// test to check if the point class save information correctly
void test_point_initialization()
{
    std::vector<float> data = {1.0, 2.0, 3.0};
    Point p(1, data);
    TEST_CHECK(p.index == 1);
    TEST_CHECK(p.dimensions == 3);
    TEST_CHECK(p.vec.size() == 3);
    TEST_CHECK(p.vec[0] == 1.0f);
    TEST_CHECK(p.vec[1] == 2.0f);
    TEST_CHECK(p.vec[2] == 3.0f);
}

// check euclidean distance between 2 vectors
void test_euclidean_distance()
{
    std::vector<float> a = {0.0, 0.0};
    std::vector<float> b = {3.0, 4.0};
    float distance = euclideanDistance(a, b);
    // the euclidean distance between point a and point b is 5.0
    TEST_CHECK(distance == 5.0f);
}

// test the function calculate_Medoid on a given graph
void test_calculateMedoid()
{
    std::vector<Point> dataset = {
        Point(0, {1.0, 1.0}),
        Point(1, {2.0, 2.0}),
        Point(2, {3.0, 3.0})};
    Vamana vamana(dataset);
    vamana.calculateMedoid();

    TEST_CHECK(vamana.medoid_idx == 1);
}

// run tests
TEST_LIST = {
    {"Edge Initialization", test_edge_initialization},
    {"Point Initialization", test_point_initialization},
    {"Euclidean Distance", test_euclidean_distance},
    {"Calculate Medoid", test_calculateMedoid},
    {NULL, NULL} // {NULL, NULL} is marking the end of the list
};