#ifndef MISC_H
#define MISC_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

/**
 * @brief
 * Setup file and console logging.
 */
void setupLogging()
{
    // Setup two sinks: console & file.
    std::vector<spdlog::sink_ptr> sinks{
        std::make_shared<spdlog::sinks::stdout_color_sink_mt>(),
        std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs.txt"),
    };

    // Create a logger to use those two sinks.
    auto spd_logger = std::make_shared<spdlog::logger>("combined_logger", begin(sinks), end(sinks));
    spdlog::register_logger(spd_logger);

    // Set it up as the default logger.
    spdlog::set_default_logger(spd_logger);

    // Change the 'printing' pattern.
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%l]%$ %v");
}

/**
 * @brief
 * If the `-DDEBUG` flag is passed to the compiler, then this function will output debug messages.
 * @param location
 * The location in the code where the debug message is being printed.
 * @param message
 * The message to print.
 */
void debug(const std::string &location, const std::string &message)
{
#ifdef DEBUG
    std::cout << location << " - " << message << std::endl;
#endif
}

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

/**
 * @brief
 * This function validates whether the file at the provided `filepath` exists or not.
 * If it does not exist, then it raises a `std::runtime_error`.
 * @param filepath
 * The filepath to the file in question.
 */
void validateFileExists(const std::string &filepath)
{
    struct stat buffer;
    int fileExists = (stat(filepath.c_str(), &buffer) == 0);

    if (fileExists == 0)
        throw std::runtime_error("There is no existing file at: " + filepath);
}

#endif // MISC_H