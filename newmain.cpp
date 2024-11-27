#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <sstream>  // for std::stringstream and std::ostringstream
#include <random>   // for std::random_device and std::mt19937


#include "argh.h" // https://github.com/adishavit/argh

#define RYML_SINGLE_HDR_DEFINE_NOW // https://github.com/biojppm/rapidyaml
#include "rapidyaml.h"

#include "spdlog/spdlog.h" // https://github.com/gabime/spdlog
#include "spdlog/stopwatch.h"

#include "conf.h"
#include "vamana.h"


/*
g++ -std=c++14 -I./inc -I./libs newmain.cpp -o newmain
./newmain
*/

int main(int argc, char **argv) {
    // Default file paths
    std::string source_path = "./.sample/dummy/dummy-data.bin";
    std::string query_path = "./.sample/dummy/dummy-queries.bin";


    // Number of dimensions for the data and queries
    const int num_data_dimensions = 102;  // Replace with actual number of dimensions for the dataset
    const int num_query_dimensions = 104; // Assuming queries have 2 additional dimensions


    // Read and parse the data points
    std::vector<Point> data_points = parse_dummy_data(source_path, num_data_dimensions);
    std::cout << "Number of data points: " << data_points.size() << std::endl;

    // Read and parse the query points
    std::vector<Point> query_points = parse_query_file(query_path, num_query_dimensions);
    std::cout << "Number of query points: " << query_points.size() << std::endl;

    // Print the first 10 data points for testing
    std::cout << "\nFirst 10 data points:\n";
    for (int i = 0; i < std::min(25, static_cast<int>(data_points.size())); ++i) {
        const Point &p = data_points[i];
        std::cout << "Point " << p.index << ": \n";
        std::cout << "Category :  " << p.category << "\t";
        std::cout << "Timestamp : " << p.lower_timestamp << "\n";
        //for (float value : p.vec) {
        //    std::cout << value << " ";
        //}
        std::cout << "\n";
    }

    // Print the first 5 query points for testing
    std::cout << "\nFirst 5 query points:\n";
    for (int i = 0; i < std::min(40, static_cast<int>(query_points.size())); ++i) {
        const Point &q = query_points[i];
        std::cout << "Query " << q.index << ": ";
        std::cout << "Query_type :  " << q.query_type << "\t";
        std::cout << "Query_category : " << q.category << "\t";
        std::cout << "lower bound : " << q.lower_timestamp << "\t";
        std::cout << "upper bound : " << q.upper_timestamp << "\n";
        //for (float value : q.vec) {
        //    std::cout << value << " ";
        //}
        std::cout << "\n";
    }
    
    // Initializing the `Vamana` object.
    Vamana vamana = Vamana(data_points , query_points );
    Point &p = vamana.dataset[20] , &q = vamana.queryset[25];
    int compatible = vamana.check_filters(p , q);

    std::cout << "compatibillity of p and q is " << compatible << "\n";

    p = vamana.dataset[21];
    q = vamana.queryset[25];

    compatible = vamana.check_filters(p , q);

    std::cout << "compatibillity of p and q is " << compatible << "\n";

    std::cout << "\n";

    return 0;
}