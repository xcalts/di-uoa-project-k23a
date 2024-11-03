#ifndef TIMER_H
#define TIMER_H

#include <chrono>

/**
 * @brief Calculate the elapsed time between operations.
 */
class Timer
{
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> endTime;
    bool isRunning = false;

public:
    /**
     * @brief Base constructor.
     */
    Timer() {}

    /**
     * @brief Starts the timer.
     */
    void start()
    {
        startTime = std::chrono::high_resolution_clock::now();
        isRunning = true;
    }

    /**
     * @brief Stops the timer.
     */
    void stop()
    {
        if (isRunning)
        {
            endTime = std::chrono::high_resolution_clock::now();
            isRunning = false;
        }
    }

    /**
     * @brief Resets the timer.
     */
    void reset()
    {
        isRunning = false;
        startTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
        endTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    }

    /**
     * @brief Returns the elapsed time in seconds.
     * @return double
     */
    double elapsed()
    {
        if (isRunning)
        {
            // Timer is still running
            return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startTime).count();
        }
        else
        {
            // Timer has been stopped
            return std::chrono::duration<double>(endTime - startTime).count();
        }
    }
};

#endif