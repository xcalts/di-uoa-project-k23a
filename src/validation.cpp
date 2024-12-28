/**********************/
/* Standard Libraries */
/**********************/
#include <stdexcept>
#include <string>
#include <sys/stat.h>

/**********************/
/* Project Components */
/**********************/
#include "constants.h"
#include "validation.h"

void validateFileExists(const std::string &filepath)
{
    struct stat buffer;
    int fileExists = (stat(filepath.c_str(), &buffer) == 0);

    if (fileExists == 0)
        throw std::runtime_error("There is no existing file @\"" + filepath + "\"");
}

void validateCommand(const std::string &cli_command)
{
    if (cli_command != CLI_INIT && cli_command != CLI_EVAL && cli_command != CLI_GTNN)
        throw std::runtime_error("The only valid commands are: \"" + CLI_INIT + "\"" + ", \"" + CLI_EVAL + "\" and \"" + CLI_GTNN + "\"");
}

void validateAlgorithm(const std::string &algorithm)
{
    if (algorithm != VAMANA || algorithm != FILTERED_VAMANA || algorithm != STICHED_VAMANA)
        throw std::runtime_error("The only valid algorithms are: \"" + VAMANA + "\"" + ", \"" + FILTERED_VAMANA + "\"" + ", and \"" + STICHED_VAMANA + "\"");
}