/**********************/
/* Standard Libraries */
/**********************/
#include <iostream>
#include <fstream>
#include <string>
#include <map>

/**********************/
/* Project Components */
/**********************/
#include "utils.h"

void appendResultsToFile(const std::string &filepath, const std::string &command, const Configuration &conf, const std::map<std::string, double> &timings)
{
    std::ofstream results_file(filepath, std::ios::app);
    if (!results_file.is_open())
    {
        std::cerr << "Failed to open " << filepath << " for writing." << std::endl;
        return;
    }

    results_file << "{";
    results_file << "\"command\":\"" << command << "\",";
    results_file << "\"configuration\":{";
    results_file << "\"dataset_filepath\":\"" << conf.dataset_filepath << "\",";
    results_file << "\"queries_filepath\":\"" << conf.queries_filepath << "\",";
    results_file << "\"evaluation_filepath\":\"" << conf.evaluation_filepath << "\",";
    results_file << "\"k\":" << conf.k << ",";
    results_file << "\"a\":" << conf.a << ",";
    results_file << "\"L\":" << conf.L << ",";
    results_file << "\"R\":" << conf.R << ",";
    results_file << "\"dummy_data_filepath\":\"" << conf.dummy_data_filepath << "\",";
    results_file << "\"dummy_queries_filepath\":\"" << conf.dummy_queries_filepath << "\",";
    results_file << "\"groundtruth_nn_filepath\":\"" << conf.groundtruth_nn_filepath << "\",";
    results_file << "\"data_dimensions\":" << conf.data_dimensions << ",";
    results_file << "\"queries_dimensions\":" << conf.queries_dimensions << ",";
    results_file << "\"tau\":" << conf.tau << ",";
    results_file << "\"L_small\":" << conf.L_small << ",";
    results_file << "\"R_small\":" << conf.R_small << ",";
    results_file << "\"R_stiched\":" << conf.R_stiched;
    results_file << "},";
    results_file << "\"timings\":{";
    for (auto it = timings.begin(); it != timings.end(); ++it)
    {
        results_file << "\"" << it->first << "\":" << it->second;
        if (std::next(it) != timings.end())
        {
            results_file << ",";
        }
    }
    results_file << "}";
    results_file << "}\n";

    results_file.close();
}