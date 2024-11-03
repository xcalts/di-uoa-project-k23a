#ifndef CONF_H
#define CONF_H

#include <string>

#include "rapidyaml.h"

#include "file.h"
#include "log.h"

/**
 * @brief This class controls all the configuration logic.
 */
class Configuration
{
private:
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
    void initialize(const std::string &filepath)
    {
        std::string _contents = readFileContents(filepath);

        print_verbose("(conf.h) Initializing the YAML tree.");
        ryml::Tree tree = ryml::parse_in_place(ryml::to_substr(_contents));
        ryml::ConstNodeRef root = tree.rootref();

        print_verbose("(conf.h) Deserializing the YAML tree into the configuration parameters.");
        root["no_queries"] >> no_queries;
        root["dataset_filepath"] >> dataset_filepath;
        root["queries_filepath"] >> queries_filepath;
        root["evaluation_filepath"] >> evaluation_filepath;
        root["verbose"] >> verbose;

        print_verbose("(conf.h) no_queries: " + std::to_string(no_queries));
        print_verbose("(conf.h) dataset_filepath: " + dataset_filepath);
        print_verbose("(conf.h) queries_filepath: " + queries_filepath);
        print_verbose("(conf.h) evaluation_filepath: " + evaluation_filepath);
        print_verbose("(conf.h) verbose: " + std::to_string(verbose));
    }

public:
    /**
     * @brief The number of queries to calculate results for.
     */
    int no_queries;

    /**
     * @brief The filepath to the FVECS dataset of image vectors.
     */
    std::string dataset_filepath;

    /**
     * @brief The filepath to the FVECS queries of image vectors.
     */
    std::string queries_filepath;

    /**
     * @brief The filepath to the IVECS evaluation metrics.
     */
    std::string evaluation_filepath;

    /**
     * @brief Whether to print `verbose` debug messages or not.
     */
    bool verbose;

    /**
     * @brief Base contructor.
     */
    Configuration() {}

    /**
     * @brief Construct a new `Configuration` object.
     * @param filepath The path to the YAML configuration file.
     */
    Configuration(const std::string &filepath)
    {
        initialize(filepath);
    }
};

#endif // CONF_H