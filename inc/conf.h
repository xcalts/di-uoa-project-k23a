#ifndef CONF_H
#define CONF_H

#include <string>

#include "rapidyaml.h"

#include "file.h"
#include "log.h"

class Configuration
{
private:
    /* Fields */
    std::string _filepath;
    std::string _contents;
    int _no_queries;

    /* Functions */
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
    /* Constructors */
    Configuration() {}
    Configuration(const std::string &filepath)
    {
        initialize(filepath);
    }

    /* Properties */
    int getNoQueries()
    {
        return _no_queries;
    }
};

#endif // CONF_H