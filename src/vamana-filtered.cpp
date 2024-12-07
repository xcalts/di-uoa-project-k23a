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

/* https://github.com/biojppm/rapidyaml */
#define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml.h"

/* https://github.com/gabime/spdlog */
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"

/************************/
/* Project's Components */
/************************/

#include "conf.h"
#include "misc.h"
#include "vamana-filtered.h"

int main(int argc, char **argv)
{
    try
    {
        std::string conf_filepath;
        spdlog::stopwatch sw;

        setupLogging();

        spdlog::info("[+] Parsing the command-line arguments.");
        argh::parser cmdl(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);
        cmdl({"-c", "--conf"}) >> conf_filepath;

        if (cmdl({"-h", "--help"}) || argc == 1)
        {
            std::cout << "Usage: ./vamana-filtered --conf ./conf.yaml" << std::endl;
            return EXIT_FAILURE;
        }

        spdlog::info("[+] Validating that the configuration filepath exists.");
        validateFileExists(conf_filepath);

        spdlog::info("[+] Parsing the YAML configuration file.");
        Configuration conf = Configuration(conf_filepath);

        spdlog::info("[+] Validating that the YAML configuration file's parameters.");
        validateFileExists(conf.dummy_data_filepath);
        validateFileExists(conf.dummy_queries_filepath);

        spdlog::info("[+] Parsing the dummy dataset/queries and printing the parsed parameters.");
        std::vector<F_Point> dummyData = parseDummyData(conf.dummy_data_filepath, conf.data_dimensions);
        std::vector<F_Point> dummyQueries = parseDummyQueries(conf.dummy_queries_filepath, conf.queries_dimensions);

        spdlog::info("    [i] Dummy Data: {} nodes ({})", dummyData.size(), conf.dummy_data_filepath);
        spdlog::info("    [i] Queries: {} nodes ({})", dummyQueries.size(), conf.dummy_queries_filepath);
        spdlog::info("    [i] K Nearest Neighbors: {}", conf.kNN);
        spdlog::info("    [i] Alpha: {}", conf.alpha);
        spdlog::info("    [i] Max Candinates: {}", conf.max_candinates);
        spdlog::info("    [i] Max Edges: {}", conf.max_edges);

        spdlog::info("[+] Initializing Vamana.");
        F_Vamana fvamana = F_Vamana(dummyData);

        spdlog::info("[+] Calculating the Medoid of the dataset.");
        sw.reset();
        fvamana.calculateMedoid();
        spdlog::info("    [i] Time Elapsed: {} seconds.", sw);
        spdlog::info("    [i] Medoid's Index: {}", fvamana.medoid_idx);

        // spdlog::info("[+] Indexing the graph using the Vamana algorithm.");
        // sw.reset();
        // vamana.index(conf.alpha, conf.max_candinates, conf.max_edges);
        // spdlog::info("    [i] Time Elapsed: {} seconds.", sw);

        // spdlog::info("[+] Evaluating the algorithm.");
        // int total = 0;
        // std::vector<int> kNNs;
        // double vamana_time = 0.0;
        // double brute_time = 0.0;
        // for (size_t idx = 0; idx < query_points.size(); ++idx)
        // {
        //     Point &q = query_points[idx];

        //     if (idx == 0)
        //     {
        //         sw.reset();
        //         kNNs = vamana.greedySearchNearestNeighbors(vamana.medoid_idx, q, conf.kNN, conf.max_candinates);
        //         vamana_time = sw.elapsed().count();
        //         spdlog::info("    [i] Vamana K-NN Request: {} seconds.", vamana_time);

        //         sw.reset();
        //         kNNs = vamana.bruteForceNearestNeighbors(q, conf.kNN);
        //         brute_time = sw.elapsed().count();
        //         spdlog::info("    [i] Brute-force K-NN Request: {} seconds.", brute_time);

        //         double speedup = brute_time / vamana_time;
        //         spdlog::info("    [i] Vamana K-NN is {:.2f} times faster than brute-force.", speedup);
        //     }

        //     kNNs = vamana.greedySearchNearestNeighbors(vamana.medoid_idx, q, conf.kNN, conf.max_candinates);
        //     std::vector<int> gt_k(ground_truth[idx].begin(), ground_truth[idx].begin() + conf.kNN);

        //     total += intersectionSize(kNNs, gt_k);
        // }

        // spdlog::info("[+] Calculating the recall percentage..");
        // spdlog::info("    [i] correct: {}", total);
        // spdlog::info("    [i] query_points: {}", query_points.size());
        // spdlog::info("    [i] k: {}", conf.kNN);
        // spdlog::info("    [i] *: {}", (conf.kNN * (query_points.size() + 1)) * 100);
        // float recall = static_cast<float>(total) / (conf.kNN * (query_points.size() + 1)) * 100;
        // spdlog::info("    [i] Recall@{}: {:.2f}%.", conf.kNN, recall);

        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[!] " << e.what() << std::endl;
    }
}