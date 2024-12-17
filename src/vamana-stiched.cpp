/**********************/
/* Standard Libraries */
/**********************/

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>

/**********************/
/* External Libraries */
/**********************/

/* https://github.com/adishavit/argh */
#include "argh.h"
#include "progressbar.h"

/* https://github.com/biojppm/rapidyaml */
#define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml.h"

/* https://github.com/gabime/spdlog */
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"

/************************/
/* Project's Components */
/************************/

//#include "brute.h"
#include "conf.h"
#include "misc.h"
#include "vamana-stiched.h"

int main(int argc, char **argv)
{
    try
    {
        std::string conf_filepath;
        spdlog::stopwatch sw;

        setupLogging();

        spdlog::info("[+] Parsing the command-line arguments.");
        argh::parser cmdl(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);

        spdlog::info("[+] Checking for insufficient amount of arguments.");
        if (cmdl({"-h", "--help"}) || argc == 1)
        {
            std::cout << "Usage: ./vamana-filtered --conf ./conf.yaml" << std::endl;
            return EXIT_SUCCESS;
        }

        spdlog::info("[+] Validating the command-line arguments.");
        cmdl({"-c", "--conf"}) >> conf_filepath;
        validateFileExists(conf_filepath);

        spdlog::info("[+] Parsing and validating the YAML configuration file.");
        Configuration conf = Configuration(conf_filepath);
        validateFileExists(conf.dummy_data_filepath);
        validateFileExists(conf.dummy_queries_filepath);
        validateFileExists(conf.groundtruth_nn_filepath);

        spdlog::info("[+] Parsing the dummy dataset & queries.");
        std::vector<F_Point> dummyData = parseDummyData(conf.dummy_data_filepath, conf.data_dimensions);
        std::vector<F_Query> dummyQueries = parseDummyQueries(conf.dummy_queries_filepath, conf.queries_dimensions);

        spdlog::info("[+] Printing all the parsed arguements.");
        spdlog::info("    [i] Dummy Data: {} nodes ({})", dummyData.size(), conf.dummy_data_filepath);
        spdlog::info("    [i] Queries: {} nodes ({})", dummyQueries.size(), conf.dummy_queries_filepath);
        spdlog::info("    [i] K Nearest Neighbors: {}", conf.kNN);
        spdlog::info("    [i] Alpha: {}", conf.alpha);
        spdlog::info("    [i] Max Candinates: {}", conf.max_candinates);
        spdlog::info("    [i] Max Edges: {}", conf.max_edges);
        spdlog::info("    [i] τ: {}", conf.tau);

        spdlog::info("[+] Initializing the filtered vanana algorithm.");
        F_Vamana fvamana = F_Vamana(dummyData);

        spdlog::info("[+] Initializing the brute-force algorithm.");
        //Brute brute = Brute(dummyData);

        // spdlog::info("[+] Calculating the GroundThruth kNNs.");
        // brute.calculateDummyGroundTruth(dummyQueries, 100);
        // brute.save("groundtruth-1000nn.txt");
        // return EXIT_SUCCESS;


        spdlog::info("[+] Running the stiched vamana indexing.");
        fvamana.StichedVamanaIndex(conf.alpha , conf.max_candinates , conf.max_edges, conf.max_edges, conf.tau);

        
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[!] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}