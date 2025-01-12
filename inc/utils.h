#ifndef UTILS_H
#define UTILS_H

/**********************/
/* Standard Libraries */
/**********************/
#include <string>
#include <map>

/**********************/
/* Project Components */
/**********************/
#include "configuration.h"

void appendResultsToFile(const std::string &filepath, const std::string &command, const Configuration &conf, const std::map<std::string, double> &timings);

#endif // UTILS_H