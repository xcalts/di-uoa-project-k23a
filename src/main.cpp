#include <iostream>

/* https://github.com/adishavit/argh */
#include "argh.h"

/* https://github.com/biojppm/rapidyaml */
#define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml.h"

#include "conf.h"
#include "data.h"
#include "log.h"
#include "validation.h"

#pragma region HELP_MESSAGE
const char *help_msg = R"""(
K23 features a plethora of KNN algorithms.

Usage:
K23a [options]

Options:
-h, --help                       Print the help message.
-c, --conf <conf_filepath>       The filepath to the YAML configuration file.
-d, --dataset <dataset_filepath> The filepath to the FVECS dataset of image vectors.
-q, --queries <queries_filepath> The filepath to the FVECS queries of image vectors.
-e, --eval <eval_filepath>       The filepath to the IVECS evaluation metrics.
--verbose                        Enable verbose debug output.

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
        std::string dataset_filepath;
        std::string queries_filepath;
        std::string evaluation_filepath;
        Configuration conf;

        argh::parser cmdl(argc, argv, argh::parser::PREFER_PARAM_FOR_UNREG_OPTION);
        verbose_enabled = cmdl["--verbose"] ? true : false;

        verbose("(main.cpp) Parsing the arguments.");
        cmdl({"-c", "--conf"}) >> conf_filepath;
        cmdl({"-d", "--dataset"}) >> dataset_filepath;
        cmdl({"-q", "--queries"}) >> queries_filepath;
        cmdl({"-e", "--evaluation"}) >> evaluation_filepath;

        verbose("(main.cpp) Printing the help message, if user does not pass enough arguments.");
        if (cmdl({"-h", "--help"}) || conf_filepath.empty() || dataset_filepath.empty() || queries_filepath.empty() || evaluation_filepath.empty())
        {
            std::cout << help_msg << std::endl;
            return EXIT_FAILURE;
        }

        verbose("(main.cpp) Validating the parsed arguments.");
        validateFileExists(conf_filepath);
        validateFileExists(dataset_filepath);
        validateFileExists(queries_filepath);
        validateFileExists(evaluation_filepath);

        verbose("(main.cpp) Parsing the YAML configuration file.");
        conf = Configuration(conf_filepath);

        verbose("(main.cpp) Parsing the dataset-images, queries-images and the ground truth NNs.");
        ImageDatabase dataset = ImageDatabase(dataset_filepath);
        ImageDatabase queries = ImageDatabase(queries_filepath);
        NearestNeighboursDatabase nn = NearestNeighboursDatabase(evaluation_filepath);

        verbose("(main.cpp) Initializing the dataset graph.");
        Graph dataset_graph = Graph(dataset.getImages());
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << "[Exception] " << e.what() << std::endl;
    }
}