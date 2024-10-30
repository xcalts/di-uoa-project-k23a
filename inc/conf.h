#ifndef CONF_H
#define CONF_H

#include <string>

#include "rapidyaml.h"

#include "file.h"
#include "log.h"

/**
 * @brief
 * This class controls all the configuration logic. It is used to parse a YAML configuration file,
 * and exports its parameters as part of its public interface.
 *
 */
class Configuration
{
private:
    std::string _filepath;
    std::string _contents;
    int _no_queries;

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
        _filepath = filepath;
        _contents = readFileContents(filepath);

        verbose("(conf.h) Initializing the YAML tree.");
        ryml::Tree tree = ryml::parse_in_place(ryml::to_substr(_contents));
        ryml::ConstNodeRef root = tree.rootref();

        verbose("(conf.h) Deserializing the YAML tree into the configuration parameters.");
        root["no_queries"] >> _no_queries;
    }

public:
    /**
     * @brief
     * Base contructor.
     *
     */
    Configuration() {}

    /**
     * @brief
     * Construct a new `Configuration` object.
     * Note that there are no validation checks for the `filepath`.
     *
     * @param filepath
     * The path to the YAML configuration file.
     */
    Configuration(const std::string &filepath)
    {
        initialize(filepath);
    }

    /**
     * @brief Get the number of queries that results will be calculated for.
     *
     * @return int
     */
    int getNoQueries()
    {
        return _no_queries;
    }
};

#endif // CONF_H