#include <iostream>
#include <vector>

/* https://github.com/adishavit/argh */
#include "argh.h"

/* https://github.com/biojppm/rapidyaml */
#define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml.h"

#include "conf.h"
#include "data.h"
#include "log.h"
#include "misc.h"
#include "timer.h"
#include "validation.h"

#pragma region HELP_MESSAGE
const char *help_msg = R"""(
K23 features a plethora of KNN algorithms.

Usage:
K23a [options]

Options:
-h, --help                       Print the help message.
-c, --conf <conf_filepath>       The filepath to the YAML configuration file.

Description:
=TBD=

Example Usage:
=TBD=

)""";
#pragma endregion

bool verbose_enabled = false;

int main(int argc, char *argv[])
{
    try
    {
        std::string conf_filepath;
        float alpha = 0.0f;
        int max_candinates, max_neighbours = 0;
        Timer timer;

        // Parsing the arguments.
        argh::parser cmdl(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);
        cmdl({"-c", "--conf"}) >> conf_filepath;
        cmdl({"-a", "--alpha"}) >> alpha;
        cmdl({"-l", "--max-candinates"}) >> max_candinates;
        cmdl({"-n", "--max-neighbors"}) >> max_neighbours;

        // Printing the help message, if user does not pass enough arguments.
        if (cmdl({"-h", "--help"}) || conf_filepath.empty() || alpha == 0 || max_candinates == 0 || max_neighbours == 0)
        {
            std::cout << help_msg << std::endl;
            return EXIT_FAILURE;
        }

        // Validating the parsed arguments.
        validateFileExists(conf_filepath);

        // Parsing the YAML configuration file.
        Configuration conf = Configuration(conf_filepath);

        // Deciding whether to prinf verbose messages or not.
        verbose_enabled = conf.verbose;

        // Validating the configuration.
        validateFileExists(conf.dataset_filepath);
        validateFileExists(conf.queries_filepath);
        validateFileExists(conf.evaluation_filepath);

        // Parsing the dataset, queries and ground-truth data.
        std::vector<Point> dataset_points = parseFvecsFile(conf.dataset_filepath);
        print_verbose("(main.cpp) Dataset: " + std::to_string(dataset_points.size()) + " Points loaded.");

        std::vector<Point> query_points = parseFvecsFile(conf.queries_filepath);
        print_verbose("(main.cpp) Queries: " + std::to_string(query_points.size()) + " Points loaded.");

        std::vector<std::vector<int>> evaluation_matrix = parseIvecsFile(conf.evaluation_filepath);
        print_verbose("(main.cpp) Evaluation: " + std::to_string(evaluation_matrix[0].size()) + " Matrices loaded.");

        // Initializing the graph.
        Graph graph = Graph(dataset_points);

        // Calculating the Mediod of the dataset.
        // graph.calculateMedoid();
        graph.medoid = dataset.at(5762);
        print_verbose("(data.h) (vamanaIndex) Medoid's Index: " + std::to_string(graph.medoid.index) + ".");

        // // Constructing the dataset graph using Vamana indexing.
        // graph.vamanaIndex(alpha, max_candinates, max_neighbours);

        // // Evaluating the algorithm.
        // GreedySearchResults r;
        // for (Point q : query_points)
        // {
        //     r = graph.greedySearch(graph.medoid, q, max_neighbours, max_candinates);
        //     r.knn_points[0].printHistogram();
        //     dataset_points[evaluation_matrix[q.index][0]].printHistogram();
        //     print_verbose("---");
        // }

        return EXIT_SUCCESS;
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << "[Exception] " << e.what() << std::endl;
    }
}