#ifndef VALIDATION_H
#define VALIDATION_H

#include <string>
#include <sys/stat.h>
#include <stdexcept>

#include "rapidyaml.h"

#include "file.h"
#include "log.h"

/**
 * @brief
 * This function validates whether the file at the provided `filepath` exists or not.
 * If it does not exist, then it raises a `std::runtime_error`.
 *
 * @param filepath
 * The filepath to the file in question.
 */
void validateFileExists(const std::string &filepath)
{
    struct stat buffer;
    int fileExists = (stat(filepath.c_str(), &buffer) == 0);

    if (fileExists == 0)
    {
        throw std::runtime_error("There is no file at: " + filepath);
    }
}

#endif // VALIDATION_H