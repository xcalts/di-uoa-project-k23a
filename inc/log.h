#ifndef LOG_H
#define LOG_H

#include <string>
#include <iostream>

/**
 * @brief
 * A global variable that controls whether verbose messages should be printed or not.
 */
extern bool verbose_enabled;

/**
 * @brief
 * If the `--verbose` flag is passed to the program, then this function will output debug messages.
 *
 * @param message
 * The message to print.
 */
void verbose(const std::string &message)
{
    if (verbose_enabled)
        std::cout << "[Verbose] " << message << std::endl;
}

#endif // LOG_H