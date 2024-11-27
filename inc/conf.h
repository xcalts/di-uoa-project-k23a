#ifndef CONF_H
#define CONF_H

#include <string>

#include "rapidyaml.h"

#include "misc.h"

/**
 * @brief This class controls all the configuration logic.
 */
class Configuration
{
private:
    std::string _filepath;

    /**
     * @brief
     * This functions initializes the `Configuration` object.
     * It parses the contents of the file pointed by `filepath`.
     * Then, it initializes the YAML tree using the `rapidyaml.h` library.
     * Finally, it will deserialize the value to the respected variables.
     *
     * @param filepath
     * The path to the YAML configuration file.
     */
    void initialize()
    {
        // Validating that the file exists.
        validateFileExists(_filepath);

        std::string _contents = readFileContents(_filepath);

        // Initializing the YAML tree.
        ryml::Tree tree = ryml::parse_in_place(ryml::to_substr(_contents));
        ryml::ConstNodeRef root = tree.rootref();

        // Deserializing the YAML tree into the configuration parameters.
        root["dataset_filepath"] >> dataset_filepath;
        root["queries_filepath"] >> queries_filepath;
        root["evaluation_filepath"] >> evaluation_filepath;
        root["kNN"] >> kNN;
        root["alpha"] >> alpha;
        root["max_candinates"] >> max_candinates;
        root["max_edges"] >> max_edges;
    }

public:
    /**
     * @brief
     * The filepath to the FVECS dataset of image vectors.
     */
    std::string dataset_filepath;

    /**
     * @brief
     * The filepath to the FVECS queries of image vectors.
     */
    std::string queries_filepath;

    /**
     * @brief
     * The filepath to the IVECS evaluation metrics.
     */
    std::string evaluation_filepath;

    /**
     * @brief
     * The number of nearest neighbors to find.
     */
    int kNN;

    /**
     * @brief
     * Scaling factor to prune outgoing eges of a node (alpha).
     */
    float alpha;

    /**
     * @brief
     * Maximum list of search candidates to use in graph traversal.
     */
    int max_candinates;

    /**
     * @brief
     * Maximum number of outgoing edges of a node. Must be less than log(N) for good results.
     */
    int max_edges;

    /**
     * @brief
     * Construct a new `Configuration` object.
     * @param filepath
     * The path to the YAML configuration file.
     */
    Configuration(const std::string &filepath)
    {
        _filepath = filepath;

        initialize();
    }
};

#endif // CONF_H