#ifndef CONF_H
#define CONF_H

#include <string>

#include "rapidyaml.h"

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
        std::string _contents = readFileContents(_filepath);

        // Initializing the YAML tree.
        ryml::Tree tree = ryml::parse_in_place(ryml::to_substr(_contents));
        ryml::ConstNodeRef root = tree.rootref();

        // Deserializing the YAML tree into the configuration parameters.
        root["dataset_filepath"] >> dataset_filepath;
        root["queries_filepath"] >> queries_filepath;
        root["evaluation_filepath"] >> evaluation_filepath;
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