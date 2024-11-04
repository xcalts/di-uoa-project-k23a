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
#include "vamana.h"
#include "misc.h"

#pragma region HELP_MESSAGE
const char *help_msg = R"""(
K23a features the Vamana Indexing algorithm.

Usage:
K23a [options]

Options:
-h, --help                     Print the help message.
-c, --conf VALUE               The filepath to the YAML configuration file.
-k, --nearest-neighbors VALUE  Number of nearest neighbors to find.
-a, --alpha VALUE              Scaling factor to prune outgoing eges of a node (alpha).
-L, --max-candinates VALUE     Maximum list of search candidates to use in graph traversal.
-R, --max-edges VALUE          Maximum number of outgoing edges of a node. Must be less than log(N) for good results.

Description:
=TBD=

Example Usage:
./bin/k23a -c ./conf.yaml -a 1 -L 10 -R 10 -k 10

)""";
#pragma endregion

int main(int argc, char *argv[])
{
    try
    {
        float a = 0.0f;
        int k, L, R = 0;
        std::string conf_filepath;
        spdlog::stopwatch sw;

        // Setup logging.
        setupLogging();

        // Parsing the arguments.
        argh::parser cmdl(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);
        cmdl({"-c", "--conf"}) >> conf_filepath;
        cmdl({"-k", "--nearest-neighbors"}) >> k;
        cmdl({"-a", "--alpha"}) >> a;
        cmdl({"-L", "--max-candinates"}) >> L;
        cmdl({"-R", "--max-edges"}) >> R;

        if (cmdl({"-h", "--help"}) || conf_filepath.empty() || (k < 1 || k > 100) || (a < 1 || a > 10) || (L < 1 || L > 300) || (R < 1 || R > 300))
        {
            std::cout << help_msg << std::endl;
            return EXIT_FAILURE;
        }

        // Validating the parsed arguments.
        validateFileExists(conf_filepath);

        // Parsing the YAML configuration file.
        Configuration conf = Configuration(conf_filepath);

        // Validating the YAML configuration file's parameters.
        validateFileExists(conf.dataset_filepath);
        validateFileExists(conf.queries_filepath);
        validateFileExists(conf.evaluation_filepath);

        // Parsing the dataset, queries and ground-truth data.
        std::vector<Point> dataset_points = parseFvecsFile(conf.dataset_filepath);
        std::vector<Point> query_points = parseFvecsFile(conf.queries_filepath);
        std::vector<std::vector<int>> ground_truth = parseIvecsFile(conf.evaluation_filepath);

        spdlog::info("Dataset: {} nodes ({})", dataset_points.size(), conf.dataset_filepath);
        spdlog::info("Queries: {} nodes ({})", query_points.size(), conf.queries_filepath);
        spdlog::info("Ground Truth: {} matrices of {} NNs ({})", ground_truth.size(), ground_truth[0].size(), conf.evaluation_filepath);

        // Initializing the `Vamana` object.
        Vamana vamana = Vamana(dataset_points);

        // Calculating the medoid of the dataset.
        sw.reset();
        vamana.calculateMedoid();
        // vamana.medoid_idx = 8736;
        spdlog::info("Medoid Calculation: @{} ({} seconds)", vamana.medoid_idx, sw);

        // Indexing the graph using the Vamana algorithm.
        sw.reset();
        vamana.index(a, L, R);
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%l]%$ %v");
        spdlog::info("Vamana Indexing: {} seconds.", sw);

        // Evaluating the algorithm.
        int total = 0;
        std::vector<int> kNNs;
        double vamana_time = 0.0;
        double brute_time = 0.0;
        for (size_t idx = 0; idx < query_points.size(); ++idx)
        {
            Point &q = query_points[idx];

            if (idx == 0)
            {
                sw.reset();
                kNNs = vamana.greedySearchNearestNeighbors(vamana.medoid_idx, q, k, L);
                vamana_time = sw.elapsed().count();
                spdlog::info("Vamana K-NN Request: {} seconds.", vamana_time);

                sw.reset();
                kNNs = vamana.bruteForceNearestNeighbors(q, k);
                brute_time = sw.elapsed().count();
                spdlog::info("Brute-force K-NN Request: {} seconds.", brute_time);

                double speedup = brute_time / vamana_time;
                spdlog::info("Vamana K-NN is {:.2f} times faster than brute-force.", speedup);
            }

            kNNs = vamana.greedySearchNearestNeighbors(vamana.medoid_idx, q, k, L);

            total += intersectionSize(kNNs, ground_truth[idx]);
        }

        // Calculate recall percentage.
        float recall = static_cast<float>(total) / (k * (query_points.size() + 1)) * 100;
        spdlog::info("Recall@{}: {:.2f}%.", k, recall);

        return EXIT_SUCCESS;
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << "[Exception] " << e.what() << std::endl;
    }
}