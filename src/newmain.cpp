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
    
    // _medoid_idx = 5234
    //vamana.calculateMedoid();
    vamana.medoid_idx = 5234;
    vamana.StichedVamanaIndex(1, 10 , 10  , 20);
    int stop=0;
    for(Point & p  : vamana.dataset){
        std::cout << "Point : " << p.index <<"with " << p.outgoing_edges.size() <<" edges \n\n";
        stop++;
        if(stop==100)
            break;
    }
    
    //vamana.FilteredVamanaIndex(1, conf.max_candinates, conf.max_edges);

    
    return 0;
}
