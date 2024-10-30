#ifndef FILE_H
#define FILE_H

#include <string>
#include <fstream>
#include <sstream>

/**
 * @brief
 * This function reads the contents of the file at `filePath`.
 * Note that the function does not perform validation checks.
 *
 * @param filePath
 * The filepath to the file that will be parsed.
 *
 * @return std::string
 */
std::string readFileContents(const std::string &filePath)
{
    std::ifstream file(filePath);
    std::stringstream buffer;

    buffer << file.rdbuf();

    return buffer.str();
}

#endif // FILE_H