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

#include "brute.h"
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
        Brute brute = Brute(dummyData);

        // spdlog::info("[+] Calculating the GroundThruth kNNs.");
        // brute.calculateDummyGroundTruth(dummyQueries, 100);
        // brute.save("groundtruth-1000nn.txt");
        // return EXIT_SUCCESS;

        spdlog::info("[+] Loading the ground-truth nearest neighbors of the dummy queries.");
        brute.load(conf.groundtruth_nn_filepath);

        spdlog::info("[+] Running the filtered vamana indexing.");
        fvamana.filteredVamanaIndexing(conf.tau, conf.alpha, conf.max_candinates, conf.max_edges);
        std::cout << std::endl;

        spdlog::info("[+] Evaluating the algorithm.");
        std::set<int> S;

        for (auto f : fvamana.F)
            S.insert(fvamana.st[f]);
        int intersection_total = 0;
        int query_count = 0;
        int true_total = 0;
        progressbar pbar(dummyQueries.size());
        for (F_Query &q : dummyQueries)
        {
            pbar.update();
            if (q.query_type == 0 || q.query_type == 2 || q.query_type == 3)
                continue;

            std::set<float> F_q = {q.v};

            auto r = fvamana.filteredGreedySearch(S, q.index, q.vec, conf.kNN, conf.max_candinates, F_q);

            std::vector<int> nn = brute.getGtNNs(q.index, std::min(static_cast<size_t>(r.first.size()), static_cast<size_t>(conf.kNN)));

            std::set<int> nn_set(nn.begin(), nn.end());

            int intersection = intersectionBetweenSetsSize(r.first, nn_set);

            query_count++;
            intersection_total += intersection;
            true_total += r.first.size();
        }
        std::cout << std::endl;
        spdlog::info("    [i] recall@{}: {:.2f}%", conf.kNN, static_cast<float>(intersection_total) * 100.0f / static_cast<float>(true_total));
        spdlog::info("");

        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[!] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}