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

        spdlog::info("[+] Parsing and validating the command-line arguments.");
        argh::parser cmdl(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);
        if (cmdl({"-h", "--help"}) || argc == 1)
        {
            std::cout << "Usage: ./vamana-filtered --conf ./conf.yaml" << std::endl;
            return EXIT_FAILURE;
        }
        cmdl({"-c", "--conf"}) >> conf_filepath;
        validateFileExists(conf_filepath);

        spdlog::info("[+] Parsing and Validating the YAML configuration file.");
        Configuration conf = Configuration(conf_filepath);
        validateFileExists(conf.dummy_data_filepath);
        validateFileExists(conf.dummy_queries_filepath);

        spdlog::info("[+] Parsing the dummy dataset/queries and printing the parsed parameters.");
        std::vector<F_Point> dummyData = parseDummyData(conf.dummy_data_filepath, conf.data_dimensions);
        std::vector<F_Query> dummyQueries = parseDummyQueries(conf.dummy_queries_filepath, conf.queries_dimensions);
        spdlog::info("    [i] Dummy Data: {} nodes ({})", dummyData.size(), conf.dummy_data_filepath);
        spdlog::info("    [i] Queries: {} nodes ({})", dummyQueries.size(), conf.dummy_queries_filepath);
        spdlog::info("    [i] K Nearest Neighbors: {}", conf.kNN);
        spdlog::info("    [i] Alpha: {}", conf.alpha);
        spdlog::info("    [i] Max Candinates: {}", conf.max_candinates);
        spdlog::info("    [i] Max Edges: {}", conf.max_edges);
        spdlog::info("    [i] τ: {}", conf.tau);

        spdlog::info("[+] Initializing Vamana.");
        F_Vamana fvamana = F_Vamana(dummyData);

        spdlog::info("[+] Calculating the GroundThruth kNNs.");
        sw.reset();
        std::map<int, std::vector<int>> ground_truth;
        progressbar bar(dummyQueries.size());
        for (F_Query &q : dummyQueries)
        {
            bar.update();
            std::vector<int> kNNs = fvamana.bruteForceNearestNeighbors(q, conf.kNN);
            ground_truth[q.index] = kNNs;
        }
        // // =Debug GroundThruth=
        // for (auto &gt : ground_truth)
        //     spdlog::info("{}: {}", gt.first, toString(gt.second));

        spdlog::info("[+] Mapping the points to the corresponding filters.");
        sw.reset();
        fvamana.mapFilters();
        spdlog::info("    [i] Time Elapsed: {} seconds.", sw);

        spdlog::info("[+] Calculating the unfiltered medoid of the dummy data.");
        sw.reset();
        // fvamana.calculateMedoid();
        fvamana.medoid_idx = 5234;
        spdlog::info("    [i] Medoid Index: {}.", fvamana.medoid_idx);
        spdlog::info("    [i] Time Elapsed: {} seconds.", sw);

        spdlog::info("[+] Finding the medoids per filter of the dummy data.");
        sw.reset();
        fvamana.calculateFilterMedoids(conf.tau);
        spdlog::info("    [i] Time Elapsed: {} seconds.", sw);

        spdlog::info("[+] Running the Filtered Vamana Indexing.");
        sw.reset();
        fvamana.filteredVamanaIndexing(conf.alpha, conf.max_candinates, conf.max_edges);
        spdlog::info("");
        spdlog::info("    [i] Time Elapsed: {} seconds.", sw);

        int total = 0;
        std::vector<int> kNNs;
        for (F_Query &q : dummyQueries)
        {
            auto result = fvamana.filteredGreedySearch(fvamana.medoid_idx, q, conf.kNN, conf.max_candinates);
            kNNs = std::vector<int>(result.first.begin(), result.first.end());
            std::vector<int> gt_k = ground_truth[q.index];

            total += intersectionSize(kNNs, gt_k);
        }

        spdlog::info("[+] Calculating the recall percentage..");
        spdlog::info("    [i] correct: {}", total);
        spdlog::info("    [i] query_points: {}", dummyQueries.size());
        spdlog::info("    [i] k: {}", conf.kNN);
        spdlog::info("    [i] *: {}", (conf.kNN * dummyQueries.size()) * 100);
        float recall = static_cast<float>(total) / (conf.kNN * dummyQueries.size()) * 100;
        spdlog::info("    [i] Recall@{}: {:.2f}%.", conf.kNN, recall);

        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[!] " << e.what() << std::endl;
    }
}