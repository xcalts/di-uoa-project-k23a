#ifndef VALIDATION_H
#define VALIDATION_H

/**********************/
/* Standard Libraries */
/**********************/
#include <string>

/**
 * @brief
 * This function validates whether the file at the provided `filepath` exists or not.
 * If it does not exist, then it raises a `std::runtime_error`.
 * @param filepath
 * The filepath to the file in question.
 */
void validateFileExists(const std::string &filepath);

/**
 * @brief
 * Validate whether the command input is valid or not.
 * @param command
 * The command input to validate.
 */
void validateCommand(const std::string &command);

/**
 * @brief
 * Validate whether the algorithm input is valid or not.
 * @param algorithm
 * The algorithm input to validate.
 */
void validateAlgorithm(const std::string &algorithm);

#endif // VALIDATION_H