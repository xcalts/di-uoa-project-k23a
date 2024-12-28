/**********************/
/* Standard Libraries */
/**********************/
#include <string>
#include <fstream>
#include <sstream>

/**********************/
/* Project Components */
/**********************/
#include "fileio.h"

std::string readFileContents(const std::string &filePath)
{
    std::ifstream file(filePath);
    std::stringstream buffer;

    buffer << file.rdbuf();

    return buffer.str();
}

void deleteFile(const std::string &filepath)
{
    //
}