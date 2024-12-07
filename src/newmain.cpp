#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>

#include "argh.h" // https://github.com/adishavit/argh

#define RYML_SINGLE_HDR_DEFINE_NOW // https://github.com/biojppm/rapidyaml
#include "rapidyaml.h"

#include "spdlog/spdlog.h" // https://github.com/gabime/spdlog
#include "spdlog/stopwatch.h"

#include "conf.h"
#include "misc.h"
#include "vamana.h"
#include "filtered_vamana.h"

/*
make filtered_vamana 
./bin/filtered_vamana
or
g++ -std=c++14 -I./inc -I./libs newmain.cpp -o newmain
*/

int main(int argc, char **argv) {
    // Default file paths
    std::string source_path = "./.github/sample/dummy/dummy-data.bin";
    std::string query_path = "./.github/sample/dummy/dummy-queries.bin";


    // Number of dimensions for the data and queries
    const int num_data_dimensions = 102;  // Replace with actual number of dimensions for the dataset
    const int num_query_dimensions = 104; // Assuming queries have 2 additional dimensions


    // Read and parse the data points
    std::vector<Point> data_points = parse_dummy_data(source_path, num_data_dimensions);
    std::cout << "Number of data points: " << data_points.size() << std::endl;

    // Read and parse the query points
    std::vector<Point> query_points = parse_query_file(query_path, num_query_dimensions);
    std::cout << "Number of query points: " << query_points.size() << std::endl;

    
    // Initializing the `Vamana` object.
    FilteredVamana vamana = FilteredVamana(data_points , query_points );
    Point &p = vamana.dataset[20] , &q = vamana.queryset[25];
    int compatible = check_filters(p , q);

    std::cout << "compatibillity of p and q is " << compatible << "\n";

    p = vamana.dataset[21];
    q = vamana.queryset[25];

    compatible = check_filters(p , q);

    std::cout << "compatibillity of p and q is " << compatible << "\n\n";

    std::map <int ,  int> Mf = vamana.FindMedoid(data_points , 5);
    for (auto medoid : Mf) {
        std::cout << "filter category : "<< medoid.first << "\twith medoid: \t" << medoid.second <<std::endl;
    }
    
    return 0;
}