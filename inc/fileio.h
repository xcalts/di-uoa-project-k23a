#ifndef FILEIO_H
#define FILEIO_H

/**********************/
/* Standard Libraries */
/**********************/
#include <string>

/**
 * @brief
 * Read the contents of the file at `filePath`.
 * Note: the function does not perform validation checks.
 *
 * @param filepath
 * The filepath to the file that will be parsed.
 *
 * @return std::string
 */
std::string readFileContents(const std::string &filepath);

/**
 * @brief
 * Delete the file at the provided `filepath`.
 * @param filepath
 * The path of the file to delete.
 */
void deleteFile(const std::string &filepath);

#endif // FILEIO_H