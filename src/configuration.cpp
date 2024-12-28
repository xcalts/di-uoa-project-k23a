/**********************/
/* Standard Libraries */
/**********************/
#include <string>

/**********************/
/* External Libraries */
/**********************/
#define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml.h"

/**********************/
/* Project Components */
/**********************/
#include "configuration.h"
#include "validation.h"
#include "fileio.h"

Configuration::Configuration() {}

void Configuration::initialize(const std::string &filepath)
{
    // Validating that the file exists.
    validateFileExists(filepath);

    std::string _contents = readFileContents(filepath);

    // Initializing the YAML tree.
    ryml::Tree tree = ryml::parse_in_place(ryml::to_substr(_contents));
    ryml::ConstNodeRef root = tree.rootref();

    // Deserializing the YAML tree into the configuration parameters.
    root["dataset_filepath"] >> dataset_filepath;
    root["queries_filepath"] >> queries_filepath;
    root["evaluation_filepath"] >> evaluation_filepath;
    root["k"] >> k;
    root["a"] >> a;
    root["L"] >> L;
    root["R"] >> R;
    root["dummy_data_filepath"] >> dummy_data_filepath;
    root["dummy_queries_filepath"] >> dummy_queries_filepath;
    root["groundtruth_nn_filepath"] >> groundtruth_nn_filepath;
    root["data_dimensions"] >> data_dimensions;
    root["queries_dimensions"] >> queries_dimensions;
    root["tau"] >> tau;
    root["L_small"] >> L_small;
    root["R_small"] >> R_small;
    root["R_stiched"] >> R_stiched;
}