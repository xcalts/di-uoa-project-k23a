#ifndef MISC_H
#define MISC_H

#include <fstream>
#include <vector>

#include "data.h"

/**
 * @brief Parse a `.fvecs` file.
 *
 * @param fvecs_filepath The path to the `.fvecs` file.
 * @return std::vector<Point>
 */
std::vector<Point> parseFvecsFile(const std::string &fvecs_filepath)
{
    std::vector<Point> points;
    std::ifstream fvecs_stream(fvecs_filepath, std::ios::binary);
    int index = 0;

    while (fvecs_stream)
    {
        int vector_no_dimensions;

        // 1. The first four bytes(int) represent the number of dimensions of the vector data.
        fvecs_stream.read(reinterpret_cast<char *>(&vector_no_dimensions), sizeof(int));

        if (!fvecs_stream)
            break;

        std::vector<float> v(vector_no_dimensions);

        // 2. Read the rest of the values as a vector of size `vector_no_dimensions`.
        fvecs_stream.read(reinterpret_cast<char *>(v.data()), vector_no_dimensions * sizeof(float));

        // 3. Create a new `Image` object.
        Point new_point = Point(index, v);

        // 4. Add it to the database.
        points.push_back(new_point);

        index++;
    }

    fvecs_stream.close();

    return points;
}

/**
 * @brief Parse a `.ivecs` file.
 * @param ivecs_filepath The path to the `.ivecs` file.
 * @return std::vector<std::vector<int>>
 */
std::vector<std::vector<int>> parseIvecsFile(const std::string &ivecs_filepath)
{
    std::vector<std::vector<int>> results;

    std::ifstream ivecs_stream(ivecs_filepath, std::ios::binary);
    while (ivecs_stream)
    {
        // 1. The first four bytes(int) represent the number of dimensions of the vector data.
        int dim = 0;
        ivecs_stream.read(reinterpret_cast<char *>(&dim), sizeof(int));

        if (!ivecs_stream)
            break;

        std::vector<int> v(dim);

        // 2. Read the rest of the values as a vector of size `_dimensions`.
        ivecs_stream.read(reinterpret_cast<char *>(v.data()), dim * sizeof(int));

        // 4. Add it to the database.
        results.push_back(v);
    }

    ivecs_stream.close();

    return results;
}

#endif // MISC_H